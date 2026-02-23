cmake_minimum_required(VERSION 3.10)
project(MyU3D)

set(CMAKE_CXX_STANDARD 17)

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
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/inc)
include_directories(${PROJECT_SOURCE_DIR}/third_party/eigen3)


set(ALL_LIBS ${OpenCV_LIBS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
link_directories(${LIBRARY_OUTPUT_DIRECTORY})

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/test)
