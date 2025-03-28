cmake_minimum_required(VERSION 3.10)

project(visual_odometry)

set(CMAKE_BUILD_PARALLEL_LEVEL 0) 
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -O2")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(GDAL REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/include/kdtree)

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(KD_TREE_DIR ${CMAKE_SOURCE_DIR}/include/kdtree)  
set(TEST_DIR ${CMAKE_SOURCE_DIR}/src/test_file)
set(EXEC_DIR ../exec)  

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXEC_DIR})

# Define executable for test_utils_file: functions on files
add_executable(
    test_utils_file 
    ${TEST_DIR}/test_utils_file.cpp 
    ${SRC_DIR}/utils_file.cpp
)

# Define executable for test_kd_tree: functions for kd-tree
add_executable(
    test_kd_tree
    ${TEST_DIR}/test_kdtree.cpp
    ${SRC_DIR}/utils_file.cpp
)

# Define executable for test_utils_file: functions on files
add_executable(
    test_correspondences 
    ${TEST_DIR}/test_correspondences.cpp 
    ${SRC_DIR}/utils_file.cpp
)

# Define executable for test_epipolar_geometry: functions for triangulation
add_executable(
    test_epipolar_geometry
    ${TEST_DIR}/test_epipolar_geometry.cpp
    ${SRC_DIR}/utils_file.cpp
    ${SRC_DIR}/epipolar_geometry.cpp
)


# Set include directories for targets
target_include_directories(test_utils_file PRIVATE ${INCLUDE_DIR} ${KD_TREE_DIR})
target_include_directories(test_kd_tree PRIVATE ${INCLUDE_DIR} ${KD_TREE_DIR})
target_include_directories(test_correspondences PRIVATE ${INCLUDE_DIR} ${KD_TREE_DIR})
target_include_directories(test_epipolar_geometry PRIVATE ${GDAL_INCLUDE_DIRS})

target_link_libraries(test_utils_file Eigen3::Eigen)
target_link_libraries(test_kd_tree Eigen3::Eigen)
target_link_libraries(test_epipolar_geometry Eigen3::Eigen ${OpenCV_LIBS} ${GDAL_LIBRARIES})