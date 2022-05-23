cmake_minimum_required(VERSION 3.18)

project(pm5000s VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

configure_file(pm5000s_version.h.in pm5000s_version.h)

add_compile_options(-Wall -Wextra -Wshadow -Wformat=2 -Wunused)

add_executable(pm5000s pm5000s.cc pm5000s.h)
target_include_directories(pm5000s PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

add_executable(pm5000sd pm5000sd.cc pm5000s.h)
target_include_directories(pm5000sd PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

install(TARGETS pm5000s pm5000sd
    DESTINATION bin)
install(FILES pm5000sd.config
    DESTINATION etc/pm5000s)

install(FILES pm5000s@.service
    DESTINATION lib/systemd/system)

execute_process(COMMAND uname -m 
    OUTPUT_VARIABLE architecture OUTPUT_STRIP_TRAILING_WHITESPACE)

include(InstallRequiredSystemLibraries)
set(CPACK_PACKAGE_VENDOR "LG Innotek")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${architecture}")
include(CPack)
