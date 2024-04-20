cmake_minimum_required(VERSION 3.10)
set(CMAKE_CPP_STANDARD 21)
set(CMAKE_CPP_STANDARD_REQUIRED True)


project(EvoRob VERSION 0.1)
configure_file(config.h.in config.h)
add_executable(EvoRob main.cpp)
target_include_directories(EvoRob PUBLIC "${PROJECT_BINARY_DIR}")
