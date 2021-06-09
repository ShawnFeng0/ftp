cmake_minimum_required(VERSION 3.19)
project(mavlink_ftp)

set(CMAKE_CXX_STANDARD 14)

include_directories(src/lib)
add_executable(mavlink_ftp main.cpp src/lib/ftp/mavlink_ftp.cpp)
