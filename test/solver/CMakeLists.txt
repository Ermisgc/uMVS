add_executable(test_quartic_solver_scale test_quartic_solver_scale.cpp)
target_link_libraries(test_quartic_solver_scale ${PROJECT_NAME} ${ALL_LIBS})

add_executable(test_single_solver test_single_solver.cpp)
target_link_libraries(test_single_solver ${PROJECT_NAME} ${ALL_LIBS})

add_executable(test_svd test_svd.cpp)
target_link_libraries(test_svd ${PROJECT_NAME} ${ALL_LIBS})

# 针对NSGA-II优化器的测试
add_executable(test_nsga2_zdt1 test_nsga2_zdt1.cpp)
target_link_libraries(test_nsga2_zdt1 ${PROJECT_NAME} ${ALL_LIBS})

add_executable(test_nsga2_zdt2 test_nsga2_zdt2.cpp)
target_link_libraries(test_nsga2_zdt2 ${PROJECT_NAME} ${ALL_LIBS})

add_executable(test_nsga2_zdt3 test_nsga2_zdt3.cpp)
target_link_libraries(test_nsga2_zdt3 ${PROJECT_NAME} ${ALL_LIBS})

add_executable(test_nsga2_zdt4 test_nsga2_zdt4.cpp)
target_link_libraries(test_nsga2_zdt4 ${PROJECT_NAME} ${ALL_LIBS})

add_executable(test_nsga2_zdt6 test_nsga2_zdt6.cpp)
target_link_libraries(test_nsga2_zdt6 ${PROJECT_NAME} ${ALL_LIBS})

add_executable(test_nsga2_dtlz1 test_nsga2_dtlz1.cpp)
target_link_libraries(test_nsga2_dtlz1 ${PROJECT_NAME} ${ALL_LIBS})

add_executable(test_nsga2_dtlz2 test_nsga2_dtlz2.cpp)
target_link_libraries(test_nsga2_dtlz2 ${PROJECT_NAME} ${ALL_LIBS})

add_executable(test_nsga2_bnh test_nsga2_bnh.cpp)
target_link_libraries(test_nsga2_bnh ${PROJECT_NAME} ${ALL_LIBS})
