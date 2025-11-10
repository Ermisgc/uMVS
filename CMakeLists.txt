cmake_minimum_required(VERSION 3.10)
project(MyU3D)

set(CMAKE_CXX_STANDARD 17)

include_directories(${PROJECT_SOURCE_DIR}/inc)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)