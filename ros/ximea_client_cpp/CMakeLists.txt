cmake_minimum_required(VERSION 2.8)

project (ximea_client_cpp)
set(CMAKE_BUILD_TYPE Release)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(libximeaclient.so SHARED receiver.cpp)
