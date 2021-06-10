cmake_minimum_required(VERSION 3.19)
project(mavlink_ftp)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(src/lib/ulog)

include_directories(src/lib)
add_executable(mavlink_ftp main.cpp src/lib/ftp/file_server.cc src/lib/crc32/crc32.c)
target_link_libraries(mavlink_ftp ulog)
