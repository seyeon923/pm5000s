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

    ErrorCode GetSerialNo(std::string& serial_no) const {
        serial_no.clear();

        if (!IsOpened()) {
            return ErrorCode::NOT_OPENED_ERR;
        }

        write(fd_, READ_SERIAL_NO_SEND_MSG_, sizeof(READ_SERIAL_NO_SEND_MSG_));

        int nread = read(fd_, READ_BUF_, sizeof(READ_BUF_));

        auto err = CheckReceivedData(nread);
        if (err != ErrorCode::OK) {
            return err;
        }

        if (nread < READ_SERIAL_NO_RECV_MSG_LEN) {
            return ErrorCode::NOT_ENOUGH_DATA_ERR;
        }

        std::stringstream ss;
        for (int i = 0; i < 5; ++i) {
            int num = READ_BUF_[POS_DATA_START + i * 2] * 256 +
                      READ_BUF_[POS_DATA_START + i * 2 + 1];
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

        tty.c_cc[VTIME] = 5;  // Wait for up to 1s(10 deciseconds)
        tty.c_cc[VMIN] = 56;  // Wait until getting 2 characters at least, which
                              // is (HEAD, LEN)

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
        int data_len = READ_BUF_[POS_LEN];
        int response_len = data_len + 3;
        if (read_len < 2 || read_len < response_len) {
            return ErrorCode::NOT_ENOUGH_DATA_ERR;
        }

        if (READ_BUF_[POS_HEAD] != RX_HEAD) {
            return ErrorCode::RX_HEAD_ERR;
        }

        int checksum_pos = data_len + 2;
        char checksum = 0;
        for (int pos = 0; pos < response_len - 1; ++pos) {
            checksum += READ_BUF_[pos];
        }
        if (READ_BUF_[checksum_pos] != static_cast<char>(256 - checksum)) {
            return ErrorCode::CHECKSUM_ERR;
        }

        return ErrorCode::OK;
    }

    int fd_;
    std::string device_path_;
    mutable char READ_BUF_[256];

    constexpr static char RX_HEAD = 0x16;
    constexpr static int POS_HEAD = 0;
    constexpr static int POS_LEN = 1;
    constexpr static int POS_CMD = 2;
    constexpr static int POS_DATA_START = 3;

    constexpr static char READ_SERIAL_NO_SEND_MSG_[] = {0x11, 0x01, 0x1F, 0xCF};
    constexpr static int READ_SERIAL_NO_RECV_MSG_LEN = 14;
};
}  // namespace pm5000s

#endif  // PM5000S_PM5000S_H_
