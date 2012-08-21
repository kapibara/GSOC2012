cmake_minimum_required(VERSION 2.6)

project(BasicVideoReader)

set(HOME ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)

set(ILIBPATH /home/kuznetso/Development/KinectOpenNIVideocapture)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_INSTALL_PREFIX ${HOME}/bin)

include_directories(${HOME}/include)
include_directories(${OPENCV})
include_directories(${ILIBPATH}/include)

add_subdirectory(${HOME}/src)
