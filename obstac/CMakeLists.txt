cmake_minimum_required(VERSION 2.8.3)
project(obstac)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

# FIND_LIBRARY_USE_LIB32_PATHS(GeographicLib REQUIRED)
# include_directories(${GeographicLib_INCLUDE_DIRS})

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
# cs_add_library(${PROJECT_NAME}

#   src/gpss.cpp

# )
# cs_add_library(${PROJECT_NAME}

#   src/gpss.cpp

# )

############
# BINARIES #
############
cs_add_executable(val
  src/path.cpp
)

# target_link_libraries(gpss  ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()
cs_export()
