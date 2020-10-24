# Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/CMakeLists.txt
cmake_minimum_required(VERSION 2.8)
project(pc_ransac)

add_definitions(-std=c++11)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})



set(CMAKE_CXX_FLAGS "-fpermissive")

include_directories(./)
include_directories(./MiscLib/)
FILE(GLOB SRC "./*.cpp")
FILE(GLOB MISC_SRC "./MiscLib/*.cpp")

add_library(libransac STATIC  ${SRC} ${MISC_SRC})
add_executable(main main.cpp)
target_link_libraries(main libransac ${PCL_LIBRARIES})
