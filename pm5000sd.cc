#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <utility>
#include <algorithm>
#include <cctype>
#include <thread>
#include <chrono>
#include <ctime>
#include <iomanip>

#include "pm5000s.h"

using namespace pm5000s;
using namespace std::literals;
namespace fs = std::filesystem;

constexpr SerialPort::ErrorCode OK = SerialPort::ErrorCode::OK;

constexpr char CONFIG_FILE_PATH[] = "/usr/local/pm5000s/pm5000sd.config";

struct Config {
    static constexpr char KEY_MEASURE_INTERVAL_MS[] = "measure_interval_ms";
    static constexpr int MIN_MEASURE_INTERVAL_MS = 10;
    static constexpr int MAX_MEASURE_INTERVAL_MS = 5 * 60 * 1000;  // 5min
    int measure_interval_ms = 1000;

    static constexpr char KEY_RETRY_LIMIT[] = "retry_limit";
    static constexpr int MIN_RETRY_LIMIT = 1;
    static constexpr int MAX_RETRY_LIMIT = 100;
    int retry_limit = 10;
};

inline std::string ConfigToString(const Config& config) {
    return "    measure_interval_ms = " +
           std::to_string(config.measure_interval_ms) + "\n" +
           "    retry_limit = " + std::to_string(config.retry_limit);
}
inline std::ostream& operator<<(std::ostream& os, const Config& config) {
    os << ConfigToString(config);
    return os;
}

inline void Trim(std::string& str) {
    auto end =
        std::remove_if(std::begin(str), std::end(str), [](char ch) -> bool {
            return std::isspace(static_cast<unsigned char>(ch));
        });
    str.erase(std::begin(str), end);
}

inline Config GetConfig() {
    if (!fs::exists(CONFIG_FILE_PATH)) {
        std::cout << "No configuration file " << CONFIG_FILE_PATH
                  << "\nWill use default configuration" << std::endl;
        return {};
    }

    std::ifstream ifs{CONFIG_FILE_PATH};
    if (!ifs.is_open()) {
        std::cerr << "Failed to open file " << CONFIG_FILE_PATH
                  << "\nWill use default configuration" << std::endl;
        return {};
    }

    std::string line;
    Config config;
    while (std::getline(ifs, line)) {
        std::stringstream ss{std::move(line)};

        std::string key, val;
        if (!std::getline(ss, key, '=')) {
            continue;
        }
        std::getline(ss, val);

        Trim(key);
        Trim(val);

        if (key == Config::KEY_MEASURE_INTERVAL_MS) {
            try {
                int measure_interval_ms = std::stoi(val);
                measure_interval_ms =
                    measure_interval_ms < Config::MIN_MEASURE_INTERVAL_MS
                        ? Config::MIN_MEASURE_INTERVAL_MS
                        : measure_interval_ms;
                measure_interval_ms =
                    measure_interval_ms > Config::MAX_MEASURE_INTERVAL_MS
                        ? Config::MAX_MEASURE_INTERVAL_MS
                        : measure_interval_ms;
                config.measure_interval_ms = measure_interval_ms;
            } catch (...) {
                std::cerr << "Invalid value for key "
                          << Config::KEY_MEASURE_INTERVAL_MS << ": " << val
                          << std::endl;
            }
        } else if (key == Config::KEY_RETRY_LIMIT) {
            try {
                int retry_limit = std::stoi(val);
                retry_limit = retry_limit < Config::MIN_RETRY_LIMIT
                                  ? Config::MIN_RETRY_LIMIT
                                  : retry_limit;
                retry_limit = retry_limit > Config::MAX_RETRY_LIMIT
                                  ? Config::MAX_RETRY_LIMIT
                                  : retry_limit;
                config.retry_limit = retry_limit;
            } catch (...) {
                std::cerr << "Invalid value for key " << Config::KEY_RETRY_LIMIT
                          << ": " << val << std::endl;
            }
        }
    }

    return config;
}

inline void LogSwVer(const std::string& serial_no, const std::string& sw_ver,
                     const fs::path& log_dir) {
    fs::path log_path = log_dir / fs::path(serial_no + "_sw_ver.txt");
    std::ofstream ofs{log_path};
    if (!ofs.is_open()) {
        std::cerr << "Failed to open " << log_path << std::endl;
        return;
    }
    ofs << "SW Version: " << sw_ver;
}

inline void Log(const std::string& serial_no, const fs::path& log_dir,
                uint32_t pm_0_3, uint32_t pm_0_5, uint32_t pm_1_0,
                uint32_t pm_2_5, uint32_t pm_5_0, uint32_t pm_10_0, int alarm) {
    auto now = std::chrono::system_clock::now();
    auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now.time_since_epoch())
                              .count();
    auto ms = ms_since_epoch % 1000;
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    std::stringstream ss_date_until_hour, ss_date_until_ms;
    ss_date_until_hour << std::put_time(&tm, "%Y%m%d%H");
    ss_date_until_ms << std::put_time(&tm, "%Y/%m/%d %H:%M:%S.") << ms;
    std::string date_str_until_ms{ss_date_until_ms.str()};

    fs::path log_path =
        log_dir / fs::path(serial_no + "_" + ss_date_until_hour.str() + ".csv");

    std::cout << "[" << date_str_until_ms << "] "
              << "PM-0.3: " << pm_0_3 << ", "
              << "PM-0.5: " << pm_0_5 << ", "
              << "PM-1.0: " << pm_1_0 << ", "
              << "PM-2.5: " << pm_2_5 << ", "
              << "PM-5.0: " << pm_5_0 << ", "
              << "PM-10.0: " << pm_10_0 << ", "
              << "Alarm: 0x" << std::hex << std::setfill('0') << std::setw(2)
              << alarm << std::dec << std::endl;

    std::ofstream ofs;
    if (fs::exists(log_path)) {
        ofs.open(log_path, std::ios::app);
    } else {
        ofs.open(log_path);
        if (!ofs.is_open()) {
            std::cerr << "Failed to open " << log_path << std::endl;
            return;
        }
        ofs << "Time,PM-0.3,PM-0.5,PM-1.0,PM-2.5,PM-5.0,PM-10.0,Alarm\n";
    }
    if (!ofs.is_open()) {
        std::cerr << "Failed to open " << log_path << std::endl;
        return;
    }
    ofs << date_str_until_ms << ',' << pm_0_3 << ',' << pm_0_5 << ',' << pm_1_0
        << ',' << pm_2_5 << ',' << pm_5_0 << ',' << pm_10_0 << ',' << alarm
        << '\n';
}

int main(int argc, char const* const* const argv) {
    std::cout << "pm5000s : v" << PM5000S_VERSION_MAJOR << '.'
              << PM5000S_VERSION_MINOR << '.' << PM5000S_VERSION_PATCH
              << std::endl;

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <Device Path> <Save Log Dir>"
                  << std::endl;
        return -1;
    }

    std::string device_path{argv[1]};
    fs::path log_dir{argv[2]};

    if (!fs::exists(log_dir)) {
        if (!fs::create_directories(log_dir)) {
            std::cerr << "Failed to create directory " << log_dir << std::endl;
            return -1;
        }
    }

    SerialPort serial{device_path};
    if (!serial.IsOpened()) {
        std::cerr << "Failed to open " << device_path << std::endl;
        return -1;
    }

    Config config = GetConfig();
    std::cout << "Used Configuration = \n" << config << std::endl;

    std::string serial_no, sw_ver;

    SerialPort::ErrorCode err;
    if (err = serial.ReadSerialNo(serial_no); err != OK) {
        std::cerr << "Failed to read Serial Number: "
                  << SerialPort::StrError(err) << std::endl;
        return static_cast<int>(err);
    }
    if (err = serial.ReadSwVersion(sw_ver); err != OK) {
        std::cerr << "Failed to read sensor's SW version: "
                  << SerialPort::StrError(err) << std::endl;
        return static_cast<int>(err);
    }
    unsigned char coeffi;
    if (err = serial.ReadCalibCoeff(coeffi); err != OK) {
        std::cerr << "Failed to read particle calibration coefficient: "
                  << SerialPort::StrError(err) << std::endl;
        return static_cast<int>(err);
    }

    std::cout << "Sensor's Serial Number: " << serial_no << '\n';
    std::cout << "Sensor's SW Version: " << sw_ver << '\n';
    std::cout << "Sensor's Calibration Coefficient: "
              << static_cast<int>(coeffi) << std::endl;
    LogSwVer(serial_no, sw_ver, log_dir);

    if (err = serial.OpenParticleMeasurement(); err != OK) {
        std::cerr << "Failed to open particle measurment: "
                  << SerialPort::StrError(err) << std::endl;
        return static_cast<int>(err);
    }
    std::cout << "Opened Particle Measurment" << std::endl;

    uint32_t pm_0_3, pm_0_5, pm_1_0, pm_2_5, pm_5_0, pm_10_0;
    unsigned char alarm;
    int num_retry = 0;
    while (true) {
        if (err = serial.ReadParticleMeasurement(pm_0_3, pm_0_5, pm_1_0, pm_2_5,
                                                 pm_5_0, pm_10_0, alarm);
            err != OK) {
            std::cerr << "Failed to read particle measurement (retry: "
                      << num_retry << ")" << std::endl;

            if (++num_retry > config.retry_limit) {
                std::cerr << "Exceeded retry limit, Exit the program"
                          << std::endl;
                return static_cast<int>(err);
            }
        } else {
            num_retry = 0;

            Log(serial_no, log_dir, pm_0_3, pm_0_5, pm_1_0, pm_2_5, pm_5_0,
                pm_10_0, alarm);
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds{config.measure_interval_ms});
    }

    return 0;
}