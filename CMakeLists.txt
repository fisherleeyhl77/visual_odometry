cmake_minimum_required(VERSION 2.8)
project( VisualOdometry )

# set compiler
set( CMAKE_CXX_COMPILER "g++")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3")

# Set the directory of binaries
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 

# Set the directory of header files
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add the subdirectory for source files
add_subdirectory( ${PROJECT_SOURCE_DIR}/src)
