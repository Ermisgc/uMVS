add_executable(rectify rectify.cpp)
target_link_libraries(rectify ${PROJECT_NAME} ${ALL_LIBS})

add_executable(epipolar_uw epipolar_uw.cpp)
target_link_libraries(epipolar_uw ${PROJECT_NAME} ${ALL_LIBS})

add_executable(rectifyUW rectifyUW.cpp)
target_link_libraries(rectifyUW ${PROJECT_NAME} ${ALL_LIBS})

add_executable(test_vertical_error_of_rectify test_vertical_error_of_rectify.cpp)
target_link_libraries(test_vertical_error_of_rectify ${PROJECT_NAME} ${ALL_LIBS})