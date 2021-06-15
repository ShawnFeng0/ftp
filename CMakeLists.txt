cmake_minimum_required(VERSION 3.19)
project(mavlink_ftp)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(src/lib/ulog)

include_directories(src/lib)
add_executable(mavlink_ftp src/ftp_server_test.cc src/lib/ftp/file_server.cc src/lib/ftp/crc32.c src/lib/ftp/md5.cc)
target_link_libraries(mavlink_ftp ulog)
