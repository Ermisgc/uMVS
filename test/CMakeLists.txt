add_executable(00_test_camera_model 00_test_camera_model.cpp)
target_link_libraries(00_test_camera_model ${PROJECT_NAME} ${ALL_LIBS})

add_executable(01_test_calibration 01_test_calibration.cpp)
target_link_libraries(01_test_calibration ${PROJECT_NAME} ${ALL_LIBS})

add_executable(02_test_uwcamera 02_test_uwcamera.cpp)
target_link_libraries(02_test_uwcamera ${PROJECT_NAME} ${ALL_LIBS})

add_executable(03_test_rectify 03_test_rectify.cpp)
target_link_libraries(03_test_rectify ${PROJECT_NAME} ${ALL_LIBS})

add_executable(04_test_uw_epipolar 04_test_uw_epipolar.cpp)
target_link_libraries(04_test_uw_epipolar ${PROJECT_NAME} ${ALL_LIBS})

add_executable(99_test_svd 99_test_svd.cpp)
target_link_libraries(99_test_svd ${PROJECT_NAME} ${ALL_LIBS})