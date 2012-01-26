cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(sonar_node src/serial.cpp src/sonar.cpp src/sonar_node.cpp)
rosbuild_add_executable(wheelchair_node
    src/wheelchair_node.cpp
    src/protocol_handlers.cpp
    src/protocol_dispatcher.cpp
    src/serial.cpp)
