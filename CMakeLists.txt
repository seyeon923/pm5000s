cmake_minimum_required(VERSION 3.18)

project(pm5000s VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

configure_file(pm5000s_version.h.in pm5000s_version.h)

add_compile_options(-Wall -Wextra -Wshadow -Wformat=2 -Wunused)

add_executable(pm5000s pm5000s.cc pm5000s.h)
target_include_directories(pm5000s PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

install(TARGETS pm5000s 
    DESTINATION bin)
