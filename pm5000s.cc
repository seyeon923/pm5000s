#include "pm5000s.h"

#include <iostream>
#include <ios>
#include <limits>

using namespace pm5000s;

constexpr SerialPort::ErrorCode OK = SerialPort::ErrorCode::OK;

inline void ClearCin() {
    if (!std::cin.good()) {
        std::cin.clear();
    }

    std::cin.ignore(std::numeric_limits<std::streamsize>::max());
}

enum class Command {
    READ_SERIAL_NO = 1,
    READ_SW_VER = 2,
    READ_PRTCL_COEF = 3,
    SETUP_PRTCL_COEF = 4,
    OPEN_PRTCL_MSR = 5,
    CLOSE_PRTCL_MSR = 6,
    READ_PRTCL_MSR = 7,
    READ_DEVICE_PATH = 8,
    EXIT = 9
};

inline void ShowPrompt() {
    std::cout << "Enter Command Number you want to execute\n"
                 "\n"
              << static_cast<int>(Command::READ_SERIAL_NO)
              << ". Read Serial Number\n"
              << static_cast<int>(Command::READ_SW_VER)
              << ". Read SW Version Number\n"
              << static_cast<int>(Command::READ_PRTCL_COEF)
              << ". Read Particle Calibration Coefficient\n"
              << static_cast<int>(Command::SETUP_PRTCL_COEF)
              << ". Set up Particle Calibration Coefficient\n"
              << static_cast<int>(Command::OPEN_PRTCL_MSR)
              << ". Open Particle Measurement\n"
              << static_cast<int>(Command::CLOSE_PRTCL_MSR)
              << ". Close Particle Measurement\n"
              << static_cast<int>(Command::READ_PRTCL_MSR)
              << ". Read Particle Measurement\n"
              << static_cast<int>(Command::READ_DEVICE_PATH)
              << ". Read Device Path\n"
              << static_cast<int>(Command::EXIT) << ". Exit" << std::endl;
}

inline void ReadSerialNumber(const SerialPort& serial) {
    std::string serial_no;
    auto err = serial.ReadSerialNo(serial_no);
    if (err != OK) {
        std::cout << "Failed to read Serial Number: "
                  << SerialPort::StrError(err) << std::endl;
        return;
    }
    std::cout << "Serial Number: " << serial_no << std::endl;
}

inline void ReadSwVersion(const SerialPort& serial) {
    std::string sw_ver;
    auto err = serial.ReadSwVersion(sw_ver);
    if (err != OK) {
        std::cout << "Failed to read SW Version: " << SerialPort::StrError(err)
                  << std::endl;
        return;
    }
    std::cout << "SW Version Number: " << sw_ver << std::endl;
}

inline void ReadParticleCalibrationCoefficient(const SerialPort& serial) {
    unsigned char coeffi;
    auto err = serial.ReadCalibCoeff(coeffi);
    if (err != OK) {
        std::cout << "Failed to read Calibration Coefficient: "
                  << SerialPort::StrError(err) << std::endl;
        return;
    }
    std::cout << "Particle Calibration Coefficient: " << coeffi << "/100"
              << std::endl;
}
inline void SetupParticleCalibrationCoefficient(const SerialPort& serial) {
    std::cout << "Enter Coefficient(10 ~ 25): ";
    std::cout.flush();

    ClearCin();

    unsigned char coeffi;
    std::cin >> coeffi;

    if (std::cin.fail()) {
        std::cout << "Invalid Input for Coefficient" << std::endl;
        return;
    }

    unsigned char read_coeffi;
    auto err = serial.SetupCalibCoeff(coeffi, read_coeffi);
    if (err != OK) {
        std::cout << "Failed to set Calibration Coefficient: "
                  << SerialPort::StrError(err) << std::endl;
        return;
    }
    std::cout << "Set Calibration Coefficient to " << read_coeffi << "/100"
              << std::endl;
}
inline void OpenParticleMeasurement(const SerialPort& serial) {
    auto err = serial.OpenParticleMeasurement();
    if (err != OK) {
        std::cout << "Failed to open particle measurement: "
                  << SerialPort::StrError(err) << std::endl;
        return;
    }

    std::cout << "Success to open particle measurment" << std::endl;
}
inline void CloseParticleMeasurment(const SerialPort& serial) {
    auto err = serial.CloseParticleMeasurement();
    if (err != OK) {
        std::cout << "Failed to close particle measurement: "
                  << SerialPort::StrError(err) << std::endl;
        return;
    }
    std::cout << "Success to close particle measurment" << std::endl;
}
inline void ReadParticleMeasurment(const SerialPort& serial) {
    uint32_t pn_0_3_um, pn_0_5_um, pn_1_0_um, pn_2_5_um, pn_5_0_um, pn_10_0_um;
    unsigned char alarm;
    auto err =
        serial.ReadParticleMeasurement(pn_0_3_um, pn_0_5_um, pn_1_0_um,
                                       pn_2_5_um, pn_5_0_um, pn_10_0_um, alarm);
    if (err != OK) {
        std::cout << "Failed to read particle measurement: "
                  << SerialPort::StrError(err) << std::endl;
        return;
    }

    std::cout << ">0.3um: " << pn_0_3_um << " pcs/L\n"
              << ">0.5um: " << pn_0_5_um << " pcs/L\n"
              << ">1.0um: " << pn_1_0_um << " pcs/L\n"
              << ">2.5um: " << pn_2_5_um << " pcs/L\n"
              << ">5.0um: " << pn_5_0_um << " pcs/L\n"
              << ">10.0um: " << pn_10_0_um << " pcs/L\n";
    if (alarm) {
        std::cout << SerialPort::AlarmToString(alarm) << '\n';
    }
    std::cout.flush();
}
inline void ReadDevicePath(const SerialPort& serial) {
    std::cout << "Device Path: " << serial.GetDevicePath();
    if (serial.IsOpened()) {
        std::cout << "(Opened)" << std::endl;
    } else {
        std::cout << "(Not Opened)" << std::endl;
    }
}

int main(int argc, char const* const* const argv) {
    std::cout << "pm5000s : v" << PM5000S_VERSION_MAJOR << '.'
              << PM5000S_VERSION_MINOR << '.' << PM5000S_VERSION_PATCH
              << std::endl;

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <Serial Device>" << std::endl;
        return -1;
    }

    std::string device_path{argv[1]};
    SerialPort serial(device_path);

    if (!serial.IsOpened()) {
        std::cout << "Failed to open " << device_path << std::endl;
        return -1;
    }

    int cmd;
    bool loop = true;
    while (loop) {
        ShowPrompt();

        ClearCin();
        std::cin >> cmd;

        if (std::cin.fail()) {
            std::cout << "Invalid Command!" << std::endl;
            continue;
        }

        switch (cmd) {
            case static_cast<int>(Command::READ_SERIAL_NO):
                ReadSerialNumber(serial);
                break;
            case static_cast<int>(Command::READ_SW_VER):
                ReadSwVersion(serial);
                break;
            case static_cast<int>(Command::READ_PRTCL_COEF):
                ReadParticleCalibrationCoefficient(serial);
                break;
            case static_cast<int>(Command::SETUP_PRTCL_COEF):
                SetupParticleCalibrationCoefficient(serial);
                break;
            case static_cast<int>(Command::OPEN_PRTCL_MSR):
                OpenParticleMeasurement(serial);
                break;
            case static_cast<int>(Command::CLOSE_PRTCL_MSR):
                CloseParticleMeasurment(serial);
                break;
            case static_cast<int>(Command::READ_PRTCL_MSR):
                ReadParticleMeasurment(serial);
                break;
            case static_cast<int>(Command::READ_DEVICE_PATH):
                ReadDevicePath(serial);
                break;
            case static_cast<int>(Command::EXIT):
                loop = false;
                break;
            default:
                std::cout << "Invalid Command " << cmd << std::endl;
                continue;
        }
    }

    return 0;
}
