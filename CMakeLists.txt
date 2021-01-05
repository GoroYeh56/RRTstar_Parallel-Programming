cmake_minimum_required(VERSION 3.15)  # CMake version check
project(RRTSTAR)               # Create project 
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard


# set the working directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})



# Add executable target with source files 
add_executable(RRTstar src/Main.cpp src/RRTstar.cpp src/World.cpp) 

# Link OpenMP library
find_package(OpenMP REQUIRED)
target_link_libraries(RRTstar PRIVATE OpenMP::OpenMP_CXX)


# add the include directories to the build
include_directories( ${PROJECT_SOURCE_DIR}/Include )
include_directories( Mfiles)

