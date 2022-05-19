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

    bool IsOpen() const { return fd_ >= 0; }

    // return true if success
    bool Close() {
        if (!IsOpen()) {
            return true;
        }

        if (close(fd_)) {
            LogError(errno, "close",
                     "Failed to close serial device " + device_path_);
            return false;
        }

        return true;
    }

private:
    void Open() {
        fd_ = open(device_path_.c_str(), O_RDWR);
        if (!IsOpen()) {
            LogError(errno, "open", "Failed to open device " + device_path_);
            return;
        }
    }

    int fd_;
    std::string device_path_;
};
}  // namespace pm5000s

#endif  // PM5000S_PM5000S_H_
