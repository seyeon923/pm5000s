#ifndef PM5000S_PM5000S_H_
#define PM5000S_PM5000S_H_

#include <iostream>
#include <string>
#include <utility>
#include <string_view>
#include <ios>
#include <sstream>

#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "pm5000s_version.h"

namespace pm5000s {

void LogError(int err, std::string_view func_name,
              std::string_view additional_msg = "") {
    std::cerr << "Error " << err << " from " << func_name << ": "
              << strerror(err);
    if (additional_msg != "") {
        std::cerr << '\n' << additional_msg;
    }
    std::cerr << std::endl;
}

class SerialPort {
public:
    SerialPort() : fd_(-1) {}
    SerialPort(const std::string& device_path) : device_path_(device_path) {
        Open();
    }
    SerialPort(std::string&& device_path)
        : device_path_(std::move(device_path)) {
        Open();
    }
    ~SerialPort() { Close(); }

    constexpr static speed_t BAUD_RATE = B9600;
    enum class ErrorCode {
        OK = 0,

        RX_HEAD_ERR = 1,          // when received data's head isn't 0x16
        NOT_ENOUGH_DATA_ERR = 2,  // when not all data's received
        CHECKSUM_ERR = 3,         // when checksum failed
        NOT_OPENED_ERR = 4,       // when device not opend

        MEASURE_NOT_OPENED_ERR = 11,  // When fail to open particle measurement
        MEASURE_NOT_CLOSED_ERR = 12,  // When fail to close particle measurement

        CALIB_COEF_NOT_SET_ERR = 21,
        INVALID_CALIB_COEF_ERR = 22,

        UNKNOWN_ERR = -1
    };

    static char const* StrError(ErrorCode err_code) {
        switch (err_code) {
            case ErrorCode::OK:
                return "No Error";
            case ErrorCode::RX_HEAD_ERR:
                return "Invalid Rx Header Received";
            case ErrorCode::NOT_ENOUGH_DATA_ERR:
                return "Not Enough Data Received";
            case ErrorCode::CHECKSUM_ERR:
                return "Checksum Error";
            case ErrorCode::NOT_OPENED_ERR:
                return "Device Not Opened";
            case ErrorCode::MEASURE_NOT_OPENED_ERR:
                return "Particle Measurement Not Opened";
            case ErrorCode::MEASURE_NOT_CLOSED_ERR:
                return "Particle Measurment Not Closed";
            case ErrorCode::CALIB_COEF_NOT_SET_ERR:
                return "Particle Calibration Coefficient Not Set Correctly";
            case ErrorCode::INVALID_CALIB_COEF_ERR:
                return "Invalid Calibration Coefficient Set Value";
            case ErrorCode::UNKNOWN_ERR:
                [[fallthrough]];
            default:
                return "Unknown Error";
        }
    }

    bool Open(const std::string& device_path) {
        if (IsOpened()) {
            Close();
        }

        device_path_ = device_path;

        Open();

        return IsOpened();
    }
    bool Open(std::string&& device_path) {
        if (IsOpened()) {
            Close();
        }
        device_path_ = std::move(device_path);

        Open();

        return IsOpened();
    }

    bool IsOpened() const { return fd_ >= 0; }

    // return true if success
    bool Close() {
        if (!IsOpened()) {
            return true;
        }

        if (close(fd_)) {
            LogError(errno, "close",
                     "Failed to close serial device " + device_path_);
            return false;
        }
        fd_ = -1;
        device_path_.clear();

        return true;
    }

    ErrorCode ReadSerialNo(std::string& serial_no) const {
        serial_no.clear();

        if (!IsOpened()) {
            return ErrorCode::NOT_OPENED_ERR;
        }

        write(fd_, READ_SERIAL_NO_SEND_MSG_, sizeof(READ_SERIAL_NO_SEND_MSG_));

        int nread = Read();

        auto err = CheckReceivedData(nread);
        if (err != ErrorCode::OK) {
            return err;
        }

        if (nread < READ_SERIAL_NO_RECV_MSG_LEN_) {
            return ErrorCode::NOT_ENOUGH_DATA_ERR;
        }

        std::stringstream ss;
        for (int i = 0; i < 5; ++i) {
            int num = READ_BUF_[POS_DATA_START_ + i * 2] * 256 +
                      READ_BUF_[POS_DATA_START_ + i * 2 + 1];
            if (num) {
                ss << num;
            }
        }
        serial_no = ss.str();
        if (serial_no.empty()) {
            serial_no = "0";
        }

        return ErrorCode::OK;
    }

    ErrorCode ReadSwVersion(std::string& sw_ver) const {
        sw_ver.clear();

        if (!IsOpened()) {
            return ErrorCode::NOT_OPENED_ERR;
        }

        write(fd_, READ_SW_VER_SEND_MSG_, sizeof(READ_SW_VER_SEND_MSG_));

        int nread = Read();

        auto err = CheckReceivedData(nread);
        if (err != ErrorCode::OK) {
            return err;
        }

        if (nread < READ_SW_VER_RECV_MSG_LEN_) {
            return ErrorCode::NOT_ENOUGH_DATA_ERR;
        }

        sw_ver.resize(READ_SW_VER_LEN_);
        for (int i = 0; i < READ_SW_VER_LEN_; ++i) {
            sw_ver[i] = READ_BUF_[POS_DATA_START_ + i];
        }

        return ErrorCode::OK;
    }

    ErrorCode OpenParticleMeasurement() const {
        if (!IsOpened()) {
            return ErrorCode::NOT_OPENED_ERR;
        }

        write(fd_, OPEN_PRTCL_MSR_SEND_MSG_, sizeof(OPEN_PRTCL_MSR_SEND_MSG_));

        int nread = Read();

        auto err = CheckReceivedData(nread);
        if (err != ErrorCode::OK) {
            return err;
        }

        if (nread < OPEN_CLOSE_PRTCL_RECV_MSG_LEN_) {
            return ErrorCode::NOT_ENOUGH_DATA_ERR;
        }

        if (READ_BUF_[POS_DATA_START_] != OPEN_CLOSE_PRTCL_OPEN_DATA_) {
            return ErrorCode::MEASURE_NOT_OPENED_ERR;
        }

        return ErrorCode::OK;
    }

    ErrorCode CloseParticleMeasurement() const {
        if (!IsOpened()) {
            return ErrorCode::NOT_OPENED_ERR;
        }

        write(fd_, CLOSE_PRTCL_MSR_SEND_MSG_,
              sizeof(CLOSE_PRTCL_MSR_SEND_MSG_));

        int nread = Read();

        auto err = CheckReceivedData(nread);
        if (err != ErrorCode::OK) {
            return err;
        }

        if (nread < OPEN_CLOSE_PRTCL_RECV_MSG_LEN_) {
            return ErrorCode::NOT_ENOUGH_DATA_ERR;
        }

        if (READ_BUF_[POS_DATA_START_] != OPEN_CLOSE_PRTCL_CLOSE_DATA_) {
            return ErrorCode::MEASURE_NOT_CLOSED_ERR;
        }

        return ErrorCode::OK;
    }

    ErrorCode ReadCalibCoeff(unsigned char& coeff) const {
        coeff = 0;

        if (!IsOpened()) {
            return ErrorCode::NOT_OPENED_ERR;
        }

        write(fd_, READ_CALIB_COEF_SEND_MSG_,
              sizeof(READ_CALIB_COEF_SEND_MSG_));

        int nread = Read();

        auto err = CheckReceivedData(nread);
        if (err != ErrorCode::OK) {
            return err;
        }

        if (nread < SETUP_READ_CALIB_COEF_RECV_MSG_LEN_) {
            return ErrorCode::NOT_ENOUGH_DATA_ERR;
        }

        coeff = READ_BUF_[POS_DATA_START_];
        return ErrorCode::OK;
    }

    // valid coeff range: 10 ~ 25
    ErrorCode SetupCalibCoeff(unsigned char coeff,
                              unsigned char& read_coeff) const {
        read_coeff = 0;

        if (coeff < 10 || coeff > 25) {
            return ErrorCode::INVALID_CALIB_COEF_ERR;
        }

        if (!IsOpened()) {
            return ErrorCode::NOT_OPENED_ERR;
        }

        unsigned char checksum = static_cast<unsigned char>(
            256 - ((unsigned char)0x11 + (unsigned char)0x02 +
                   (unsigned char)0x07 + coeff));
        unsigned char send_msg[] = {0x11, 0x02, 0x07, coeff, checksum};

        write(fd_, send_msg, sizeof(send_msg));

        int nread = Read();

        auto err = CheckReceivedData(nread);
        if (err != ErrorCode::OK) {
            return err;
        }

        if (nread < SETUP_READ_CALIB_COEF_RECV_MSG_LEN_) {
            return ErrorCode::NOT_ENOUGH_DATA_ERR;
        }

        read_coeff = READ_BUF_[POS_DATA_START_];

        if (coeff != read_coeff) {
            return ErrorCode::CALIB_COEF_NOT_SET_ERR;
        }

        return ErrorCode::OK;
    }

    ErrorCode ReadParticleMeasurement(
        uint32_t& prtcl_0_3_um, uint32_t& prtcl_0_5_um, uint32_t& prtcl_1_0_um,
        uint32_t& prtcl_2_5_um, uint32_t& prtcl_5_0_um, uint32_t& prtcl_10_0_um,
        unsigned char& alarm) {
        if (!IsOpened()) {
            return ErrorCode::NOT_OPENED_ERR;
        }

        write(fd_, READ_PRTCL_MSR_SEND_MSG_, sizeof(READ_PRTCL_MSR_SEND_MSG_));

        int nread = Read();

        auto err = CheckReceivedData(nread);
        if (err != ErrorCode::OK) {
            return err;
        }

        if (nread < READ_PRTCL_MSG_RECV_MSG_LEN_) {
            return ErrorCode::NOT_ENOUGH_DATA_ERR;
        }

        prtcl_0_3_um =
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_0_3_UM_OFFSET_]
                << 24 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_0_3_UM_OFFSET_ + 1]
                << 16 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_0_3_UM_OFFSET_ + 2]
                << 8 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_0_3_UM_OFFSET_ + 3];
        prtcl_0_5_um =
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_0_5_UM_OFFSET_]
                << 24 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_0_5_UM_OFFSET_ + 1]
                << 16 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_0_5_UM_OFFSET_ + 2]
                << 8 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_0_5_UM_OFFSET_ + 3];
        prtcl_1_0_um =
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_1_0_UM_OFFSET_]
                << 24 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_1_0_UM_OFFSET_ + 1]
                << 16 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_1_0_UM_OFFSET_ + 2]
                << 8 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_1_0_UM_OFFSET_ + 3];
        prtcl_2_5_um =
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_2_5_UM_OFFSET_]
                << 24 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_2_5_UM_OFFSET_ + 1]
                << 16 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_2_5_UM_OFFSET_ + 2]
                << 8 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_2_5_UM_OFFSET_ + 3];
        prtcl_5_0_um =
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_5_0_UM_OFFSET_]
                << 24 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_5_0_UM_OFFSET_ + 1]
                << 16 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_5_0_UM_OFFSET_ + 2]
                << 8 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_5_0_UM_OFFSET_ + 3];
        prtcl_10_0_um =
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_10_0_UM_OFFSET_]
                << 24 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_10_0_UM_OFFSET_ + 1]
                << 16 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_10_0_UM_OFFSET_ + 2]
                << 8 |
            (uint32_t)READ_BUF_[POS_DATA_START_ +
                                READ_PRTCL_MSG_DATA_10_0_UM_OFFSET_ + 3];

        alarm = READ_BUF_[POS_DATA_START_ + READ_PRTCL_MSG_DATA_ALARM_OFFSET_];

        return ErrorCode::OK;
    }

    void FlushReceivedBuffer() const { Read(); }

    const std::string& GetDevicePath() const { return device_path_; }

private:
    void Open() {
        fd_ = open(device_path_.c_str(), O_RDWR);
        if (!IsOpened()) {
            LogError(errno, "open", "Failed to open device " + device_path_);

            fd_ = -1;
            device_path_.clear();

            return;
        }

        termios tty;

        if (tcgetattr(fd_, &tty) != 0) {
            LogError(errno, "tcgetattr");
            Close();
            return;
        }

        // these configurations are guided by
        // https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

        // Control Modes
        tty.c_cflag &= ~CSTOPB;   // one stop bit
        tty.c_cflag &= ~PARENB;   // disable praity
        tty.c_cflag &= ~CSIZE;    // Clear all the size bits
        tty.c_cflag |= CS8;       // 8bits per byte
        tty.c_cflag &= ~CRTSCTS;  // disable RTS/CTS hardware flow control
        tty.c_cflag |= CREAD | CLOCAL;

        // Local Modes
        tty.c_lflag &= ~ICANON;  // non-canonical mode
        // Disable echo just in case
        tty.c_lflag &= ~ECHO;    // Disable echo
        tty.c_lflag &= ~ECHOE;   // Disable erasure
        tty.c_lflag &= ~ECHONL;  // Disable new-line echo
        tty.c_lflag &= ~ISIG;    // Disable signal chars(INTR, QUIT, SUSP)

        // Input Modes
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
        tty.c_iflag &=
            ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
              ICRNL);  // Disable any special handling of received bytes

        // Output Modes
        tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output
                                // bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage
                                // return/line feed

        tty.c_cc[VTIME] = 1;  // Wait for up to 0.1s(10 deciseconds)
        tty.c_cc[VMIN] = 56;

        // Set in/out baud rate
        cfsetispeed(&tty, BAUD_RATE);
        cfsetospeed(&tty, BAUD_RATE);

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            LogError(errno, "tcsetattr");

            Close();
            return;
        }
    }

    ErrorCode CheckReceivedData(int read_len) const {
        int data_len = READ_BUF_[POS_LEN_];
        int response_len = data_len + 3;
        if (read_len < 2 || read_len < response_len) {
            return ErrorCode::NOT_ENOUGH_DATA_ERR;
        }

        if (READ_BUF_[POS_HEAD_] != RX_HEAD_) {
            return ErrorCode::RX_HEAD_ERR;
        }

        int checksum_pos = data_len + 2;
        unsigned char checksum = 0;
        for (int pos = 0; pos < response_len - 1; ++pos) {
            checksum += READ_BUF_[pos];
        }
        if (READ_BUF_[checksum_pos] !=
            static_cast<unsigned char>(256 - checksum)) {
            return ErrorCode::CHECKSUM_ERR;
        }

        return ErrorCode::OK;
    }

    int Read() const {
        if (!IsOpened()) {
            return 0;
        }
        return read(fd_, READ_BUF_, sizeof(READ_BUF_));
    }

    int fd_;
    std::string device_path_;
    mutable unsigned char READ_BUF_[256];

    constexpr static unsigned char RX_HEAD_ = 0x16;
    constexpr static int POS_HEAD_ = 0;
    constexpr static int POS_LEN_ = 1;
    constexpr static int POS_CMD_ = 2;
    constexpr static int POS_DATA_START_ = 3;

    // Read Serial Number Message Related
    constexpr static unsigned char READ_SERIAL_NO_SEND_MSG_[] = {0x11, 0x01,
                                                                 0x1F, 0xCF};
    constexpr static int READ_SERIAL_NO_RECV_MSG_LEN_ = 14;

    // Read SW Version Message Related
    constexpr static unsigned char READ_SW_VER_SEND_MSG_[] = {0x11, 0x01, 0x1E,
                                                              0xD0};
    constexpr static int READ_SW_VER_RECV_MSG_LEN_ = 17;
    constexpr static int READ_SW_VER_LEN_ = 13;

    constexpr static unsigned char OPEN_PRTCL_MSR_SEND_MSG_[] = {
        0x11, 0x03, 0x0C, 0x02, 0x1E, 0xC0};
    // Open/Close Particle Measurement Message Related
    constexpr static unsigned char CLOSE_PRTCL_MSR_SEND_MSG_[] = {
        0x11, 0x03, 0x0C, 0x01, 0x1E, 0xC1};
    constexpr static int OPEN_CLOSE_PRTCL_RECV_MSG_LEN_ = 5;
    constexpr static unsigned char OPEN_CLOSE_PRTCL_OPEN_DATA_ = 0x02;
    constexpr static unsigned char OPEN_CLOSE_PRTCL_CLOSE_DATA_ = 0x01;

    // Read/Setup Particle Calibration Coefficient Message Related
    constexpr static unsigned char READ_CALIB_COEF_SEND_MSG_[] = {0x11, 0x01,
                                                                  0x07, 0xE7};
    constexpr static int SETUP_READ_CALIB_COEF_RECV_MSG_LEN_ = 5;

    // Read Particle Measurement Result Related
    constexpr static unsigned char READ_PRTCL_MSR_SEND_MSG_[] = {
        0x11, 0x02, 0x0B, 0x07, 0xDB};
    constexpr static int READ_PRTCL_MSG_RECV_MSG_LEN_ = 56;
    constexpr static int READ_PRTCL_MSG_DATA_0_3_UM_OFFSET_ = 24;
    constexpr static int READ_PRTCL_MSG_DATA_0_5_UM_OFFSET_ = 24;
    constexpr static int READ_PRTCL_MSG_DATA_1_0_UM_OFFSET_ = 32;
    constexpr static int READ_PRTCL_MSG_DATA_2_5_UM_OFFSET_ = 36;
    constexpr static int READ_PRTCL_MSG_DATA_5_0_UM_OFFSET_ = 40;
    constexpr static int READ_PRTCL_MSG_DATA_10_0_UM_OFFSET_ = 44;
    constexpr static int READ_PRTCL_MSG_DATA_ALARM_OFFSET_ = 48;
};
}  // namespace pm5000s

#endif  // PM5000S_PM5000S_H_
