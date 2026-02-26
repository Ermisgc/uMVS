if(BUILD_TESTS)
    set(EXECUTABLE_OUTPUT_PATH ${U3D_CURRENT_OUTPUT}/tests_bin)

    add_subdirectory(solver)
    add_subdirectory(stereo)
    add_subdirectory(camera)
endif()