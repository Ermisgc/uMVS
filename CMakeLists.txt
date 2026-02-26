cmake_minimum_required(VERSION 3.16)
project(MyU3D VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # 关闭 GNU 扩展

# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel" FORCE)
endif()

if(MSVC)
    # 启用 C++ 异常处理
    add_compile_options(/Zi /EHsc /arch:AVX2)
    add_link_options(/DEBUG /OPT:REF /OPT:ICF)
    add_compile_definitions(
                        GLOG_NO_ABBREVIATED_SEVERITIES
                        NOMINMAX
                        _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
                        _CRT_SECURE_NO_WARNINGS 
                        _CRT_NONSTDC_NO_DEPRECATE
                        _HAS_STD_BYTE=0
                        )
else()
    # GCC/Clang配置
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    add_compile_options(${OpenMP_CXX_FLAGS})
    add_link_options(${OpenMP_CXX_FLAGS})
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/inc)
include_directories(${PROJECT_SOURCE_DIR}/third_party/eigen3)

set(ALL_LIBS ${OpenCV_LIBS})

 # TODO:设置现在的可执行文件直接放到项目文件里，后续发布时再放到CMAKE_BINARY_DIR
set(U3D_CURRENT_OUTPUT ${PROJECT_SOURCE_DIR}) 
set(LIBRARY_OUTPUT_DIRECTORY ${U3D_CURRENT_OUTPUT}/lib)

link_directories(${LIBRARY_OUTPUT_DIRECTORY})

option(BUILD_TESTS "Build test programs" ON)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/test)
