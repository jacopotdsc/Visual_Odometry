cmake_minimum_required(VERSION 3.10)

project(visual_odometry)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -O2")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

set(SRC_DIR src)
set(INCLUDE_DIR include)
set(TEST_DIR src/test_file)
set(EXEC_DIR exec)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../${EXEC_DIR})

add_executable(
    test_utils_file 
    ${TEST_DIR}/test_utils_file.cpp 
    ${SRC_DIR}/utils_file.cpp
    )

target_include_directories(test_utils_file PRIVATE ${INCLUDE_DIR})
target_link_libraries(test_utils_file Eigen3::Eigen)