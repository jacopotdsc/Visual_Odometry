cmake_minimum_required(VERSION 3.10)

project(visual_odometry)

set(CMAKE_BUILD_PARALLEL_LEVEL 0) 
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -O2")
set(CMAKE_INSTALL_RPATH "/usr/lib")

# Find and include OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find and include GDAL and Eigen
find_package(GDAL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/include/kdtree)

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(KD_TREE_DIR ${CMAKE_SOURCE_DIR}/include/kdtree)  
set(TEST_DIR ${CMAKE_SOURCE_DIR}/src/test_file)
set(EXEC_DIR ../exec)  

# Set executables directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXEC_DIR})

add_executable(
    test_utils_file 
    ${TEST_DIR}/test_utils_file.cpp 
    ${SRC_DIR}/utils_file.cpp
    ${SRC_DIR}/camera.cpp
)

# Define executable for test_kd_tree: functions for kd-tree
add_executable(
    test_kd_tree
    ${TEST_DIR}/test_kdtree.cpp
    ${SRC_DIR}/utils_file.cpp
    ${SRC_DIR}/camera.cpp
)

# Define executable for test_utils_file: functions on files
add_executable(
    test_correspondences 
    ${TEST_DIR}/test_correspondences.cpp 
    ${SRC_DIR}/utils_file.cpp
    ${SRC_DIR}/camera.cpp
    ${SRC_DIR}/picp_solver.cpp
    ${SRC_DIR}/utils_world.cpp
    ${SRC_DIR}/epipolar_geometry.cpp
)

# Define executable for test_epipolar_geometry: functions for triangulation
add_executable(
    test_epipolar_geometry
    ${TEST_DIR}/test_epipolar_geometry.cpp
    ${SRC_DIR}/utils_file.cpp
    ${SRC_DIR}/epipolar_geometry.cpp
    ${SRC_DIR}/camera.cpp
)

# Define executable for test_camera: 
#add_executable(
#    test_camera
#    ${TEST_DIR}/test_camera.cpp
#    ${SRC_DIR}/utils_file.cpp
#    ${SRC_DIR}/epipolar_geometry.cpp
#    ${SRC_DIR}/camera.cpp
#)

# Define executable for test_camera: 
#add_executable(
#    test_picp
#    ${TEST_DIR}/test_picp.cpp
#    ${SRC_DIR}/utils_file.cpp
#    ${SRC_DIR}/epipolar_geometry.cpp
#    ${SRC_DIR}/camera.cpp
#    ${SRC_DIR}/picp_solver.cpp
#    ${SRC_DIR}/utils_world.cpp
#)

# Define executable for test_utils_evaluation: 
#add_executable(
#    test_utils_evaluation
#    ${TEST_DIR}/test_utils_evaluation.cpp
#    ${SRC_DIR}/utils_file.cpp
#    ${SRC_DIR}/epipolar_geometry.cpp
#    ${SRC_DIR}/camera.cpp
#    ${SRC_DIR}/picp_solver.cpp
#    ${SRC_DIR}/utils_world.cpp
#    ${SRC_DIR}/utils_evaluation.cpp
#)

add_executable(
    vo_system_fake
    ${TEST_DIR}/vo_system_fake.cpp
    ${SRC_DIR}/utils_file.cpp
    ${SRC_DIR}/epipolar_geometry.cpp
    ${SRC_DIR}/camera.cpp
    ${SRC_DIR}/picp_solver.cpp
    ${SRC_DIR}/utils_world.cpp
    ${SRC_DIR}/utils_evaluation.cpp
)

add_executable(
    vo_system
    ${TEST_DIR}/vo_system.cpp
    ${SRC_DIR}/utils_file.cpp
    ${SRC_DIR}/epipolar_geometry.cpp
    ${SRC_DIR}/camera.cpp
    ${SRC_DIR}/picp_solver.cpp
    ${SRC_DIR}/utils_world.cpp
    ${SRC_DIR}/utils_evaluation.cpp
)


# Linka le librerie OpenCV
find_package(OpenCV REQUIRED core)
link_libraries(${OpenCV_LIBS})

# Includi le directory per i target
target_include_directories(test_utils_file PRIVATE ${INCLUDE_DIR} ${KD_TREE_DIR})
target_include_directories(test_kd_tree PRIVATE ${INCLUDE_DIR} ${KD_TREE_DIR})
target_include_directories(test_correspondences PRIVATE ${INCLUDE_DIR} ${KD_TREE_DIR})
target_include_directories(test_epipolar_geometry PRIVATE ${GDAL_INCLUDE_DIRS})
#target_include_directories(test_camera PRIVATE ${GDAL_INCLUDE_DIRS})
#target_include_directories(test_picp PRIVATE ${GDAL_INCLUDE_DIRS})
#target_include_directories(test_utils_evaluation PRIVATE ${GDAL_INCLUDE_DIRS})
target_include_directories(vo_system_fake PRIVATE ${GDAL_INCLUDE_DIRS})
target_include_directories(vo_system PRIVATE ${GDAL_INCLUDE_DIRS})


# Linka le librerie necessarie per ciascun target
target_link_libraries(test_utils_file Eigen3::Eigen ${OpenCV_LIBS} opencv_core)
target_link_libraries(test_kd_tree Eigen3::Eigen ${OpenCV_LIBS} opencv_core)
target_link_libraries(test_correspondences Eigen3::Eigen ${OpenCV_LIBS} opencv_core)
target_link_libraries(test_epipolar_geometry Eigen3::Eigen ${OpenCV_LIBS} opencv_core ${GDAL_LIBRARIES})
#target_link_libraries(test_camera Eigen3::Eigen ${OpenCV_LIBS} opencv_core ${GDAL_LIBRARIES})
#target_link_libraries(test_picp Eigen3::Eigen ${OpenCV_LIBS} opencv_core ${GDAL_LIBRARIES})
#target_link_libraries(test_utils_evaluation Eigen3::Eigen ${OpenCV_LIBS} opencv_core ${GDAL_LIBRARIES})
target_link_libraries(vo_system_fake Eigen3::Eigen ${OpenCV_LIBS} opencv_core ${GDAL_LIBRARIES})
target_link_libraries(vo_system Eigen3::Eigen ${OpenCV_LIBS} opencv_core ${GDAL_LIBRARIES})


# Aggiungi il link alle librerie FLANN (se necessario)
#find_package(FLANN REQUIRED)
#target_link_libraries(test_utils_file ${FLANN_LIBRARIES})
#target_link_libraries(test_kd_tree ${FLANN_LIBRARIES})
#target_link_libraries(test_correspondences ${FLANN_LIBRARIES})
#target_link_libraries(test_epipolar_geometry ${FLANN_LIBRARIES})
