cmake_minimum_required(VERSION 4.0)
project(FPGA)

set(CMAKE_CXX_STANDARD 20)

add_executable(FPGA
        src/main.cpp
        src/parser.cpp src/parser.h
        src/placer.cpp src/placer.h
        src/types.h
)

target_include_directories(FPGA PRIVATE src)
