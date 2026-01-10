file(GLOB_RECURSE SRC_FILES "*.cpp")

add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${ALL_LIBS})
