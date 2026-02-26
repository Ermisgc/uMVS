add_subdirectory(camera)
add_subdirectory(optics)

set(ALL_LIB_SOURCES 
    ${CAMERA_SOURCES} 
    ${OPTICS_SOURCES})
add_library(${PROJECT_NAME} ${ALL_LIB_SOURCES})
target_link_libraries(${PROJECT_NAME} ${ALL_LIBS})

set(EXECUTABLE_OUTPUT_PATH ${U3D_CURRENT_OUTPUT}/bin)

add_executable(calibrateMono calibrateMono.cpp)
target_link_libraries(calibrateMono ${PROJECT_NAME} ${ALL_LIBS})

add_executable(calibrateBino calibrateBino.cpp)
target_link_libraries(calibrateBino ${PROJECT_NAME} ${ALL_LIBS})

add_executable(calibrateBinoUW calibrateBinoUW.cpp)
target_link_libraries(calibrateBinoUW ${PROJECT_NAME} ${ALL_LIBS} OpenMP::OpenMP_CXX)

add_executable(rectify rectify.cpp)
target_link_libraries(rectify ${PROJECT_NAME} ${ALL_LIBS})

add_executable(epipolar_uw epipolar_uw.cpp)
target_link_libraries(epipolar_uw ${PROJECT_NAME} ${ALL_LIBS})

add_executable(rectifyUW rectifyUW.cpp)
target_link_libraries(rectifyUW ${PROJECT_NAME} ${ALL_LIBS})
