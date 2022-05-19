#include <iostream>
#include <string>

#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

int main(int argc, char const* const* const argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <Serial Device>" << std::endl;
        return -1;
    }

    char const* const serial_device = argv[1];

    int serial_fd = open(serial_device, O_RDWR);

    if (serial_fd < 0) {
        std::cerr << "Failed to open device " << serial_device << "\nError "
                  << errno << " from open: " << strerror(errno) << std::endl;
        return errno;
    }

    if (int ret = close(serial_fd); ret) {
        std::cerr << "Failed to close serial device\n"
                  << "Error " << errno << "from close: " << strerror(errno)
                  << std::endl;
    }
    return 0;
}
