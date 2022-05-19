#include "pm5000s.h"

#include <iostream>

int main(int argc, char const* const* const argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <Serial Device>" << std::endl;
        return -1;
    }

    pm5000s::SerialPort serial(argv[1]);

    return 0;
}
