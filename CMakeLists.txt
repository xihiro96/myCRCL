cmake_minimum_required(VERSION 3.5)
project(my_CRCL)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp CRCLSocketInterface.cpp CRCLSocketInterface.h crcl_status_monitor.cpp ServerConfiguration.h shared_ptr.h)
add_executable(my_CRCL ${SOURCE_FILES})