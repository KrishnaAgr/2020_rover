cmake_minimum_required(VERSION 2.8.3)
project(controls)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

#find_package(OpenCV 3 REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

#############
# LIBRARIES #
#############

############
# BINARIES #
############
cs_add_executable(val
  src/hard.cpp
)

##########
# EXPORT #
##########
cs_install()
cs_export()
