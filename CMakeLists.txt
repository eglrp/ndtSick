cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(NDTRegistration)

FIND_PACKAGE(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(NDTRegistration src/NDTRegistration.cpp )
target_link_libraries (NDTRegistration ${PCL_LIBRARIES})
