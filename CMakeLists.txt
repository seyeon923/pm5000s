cmake_minimum_required(VERSION 3.18)

project(pm5000s VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_executable(pm5000s pm5000s.cc)