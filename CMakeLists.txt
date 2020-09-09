cmake_minimum_required(VERSION 3.15)  # CMake version check
project(RRTSTAR)               # Create project "simple_example"
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard


# set the working directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})



# Add executable target with source files 
add_executable(RRTstar src/Main.cpp src/RRTstar.cpp src/World.cpp) 


# add the include directories to the build
include_directories( ${PROJECT_SOURCE_DIR}/Include )
include_directories( Mfiles)

