# cmake/FindOpenCV.cmake
# Find OpenCV library
#
# This module defines:
#  OpenCV_FOUND - true if OpenCV is found
#  OpenCV_INCLUDE_DIRS - OpenCV include directories
#  OpenCV_LIBS - OpenCV libraries to link against

# 设置OpenCV的根目录
if(NOT OpenCV_ROOT)
set(OpenCV_ROOT "D:/Program Files/opencv_450_vs2019")
endif()

find_path( 
    OpenCV_INCLUDE_DIR
    NAMES opencv2/opencv.hpp
    PATHS 
        "${OpenCV_ROOT}/include"
    DOC "OpenCV include directory"
)

set(OpenCV_LIB_NAMES opencv_aruco450
                    opencv_bgsegm450
                    opencv_bioinspired450
                    opencv_calib3d450
                    opencv_ccalib450
                    opencv_core450
                    opencv_datasets450
                    opencv_dnn450
                    opencv_dnn_objdetect450
                    opencv_dnn_superres450
                    opencv_dpm450
                    opencv_face450
                    opencv_features2d450
                    opencv_flann450
                    opencv_fuzzy450
                    opencv_gapi450
                    opencv_hfs450
                    opencv_highgui450
                    opencv_imgcodecs450
                    opencv_imgproc450
                    opencv_img_hash450
                    opencv_intensity_transform450
                    opencv_line_descriptor450
                    opencv_mcc450
                    opencv_ml450
                    opencv_objdetect450
                    opencv_optflow450
                    opencv_phase_unwrapping450
                    opencv_photo450
                    opencv_plot450
                    opencv_quality450
                    opencv_rapid450
                    opencv_reg450
                    opencv_rgbd450
                    opencv_saliency450
                    opencv_shape450
                    opencv_stereo450
                    opencv_stitching450
                    opencv_structured_light450
                    opencv_superres450
                    opencv_surface_matching450
                    opencv_text450
                    opencv_tracking450
                    opencv_video450
                    opencv_videoio450
                    opencv_videostab450
                    opencv_xfeatures2d450
                    opencv_ximgproc450
                    opencv_xobjdetect450
                    opencv_xphoto450
)

set(OpenCV_LIBS)
foreach(lib ${OpenCV_LIB_NAMES})
    find_library(
        OpenCV_LIBRARY_${lib}
        NAMES ${lib}
        PATHS 
            "${OpenCV_ROOT}/x64/vc16/lib"
        DOC "OpenCV library ${lib}"
    )
    list(APPEND OpenCV_LIBS ${OpenCV_LIBRARY_${lib}})
endforeach()

# 处理查找结果
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenCV
    FOUND_VAR OpenCV_FOUND
    REQUIRED_VARS 
        OpenCV_INCLUDE_DIR
        OpenCV_LIBS
)

if(OpenCV_FOUND)
    # 设置输出变量
    set(OpenCV_INCLUDE_DIRS 
        ${OpenCV_INCLUDE_DIR}
    )

    set(OpenCV_LIBRARIES ${OpenCV_LIBS})
    
    # 创建导入目标（现代CMake方式）
    if(NOT TARGET OpenCV::OpenCV)
        add_library(OpenCV::OpenCV UNKNOWN IMPORTED)
        set_target_properties(OpenCV::OpenCV PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${OpenCV_INCLUDE_DIRS}"
            IMPORTED_LOCATION "${OpenCV_LIBRARIES}"
        )
    endif()
    
    # 打印状态信息
    message(STATUS "Found OpenCV:")
    message(STATUS "  Includes: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${OpenCV_LIBRARIES}")
endif()

mark_as_advanced(  
    OpenCV_FOUND
    OpenCV_INCLUDE_DIR
    OpenCV_LIBRARY
)