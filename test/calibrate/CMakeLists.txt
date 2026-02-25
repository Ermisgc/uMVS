add_executable(calibrateMono calibrateMono.cpp)
target_link_libraries(calibrateMono ${PROJECT_NAME} ${ALL_LIBS})

add_executable(calibrateBino calibrateBino.cpp)
target_link_libraries(calibrateBino ${PROJECT_NAME} ${ALL_LIBS})