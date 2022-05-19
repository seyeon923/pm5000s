#ifndef PM5000S_PM5000S_H_
#define PM5000S_PM5000S_H_

#include <iostream>
#include <string>
#include <utility>
#include <string_view>

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

        tty.c_cc[VTIME] = 10;  // Wait for up to 1s(10 deciseconds)
        tty.c_cc[VMIN] = 2;  // Wait until getting 2 characters at least, which
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

    int fd_;
    std::string device_path_;
};
}  // namespace pm5000s

#endif  // PM5000S_PM5000S_H_
