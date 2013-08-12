cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(um6_driver src/main.cpp src/comms.cpp src/registers.cpp serial/src/serial.cc serial/src/impl/unix.cc)
include_directories(serial/include)

rosbuild_add_boost_directories()
