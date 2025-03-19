#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>

template <int dim>
using Vectorf = Eigen::Matrix<float, dim, 1>;
//using ContainerType = std::vector<Vectorf<11>, Eigen::aligned_allocator<Vectorf<11>>>;
using ContainerType = std::vector<Vectorf<11>>;

struct CameraParameters {
    Eigen::Matrix3f K;  // Intrinsic Matrix
    Eigen::Matrix4f T_cam_robot;  // Homogeneous transform of camera
};

ContainerType read_meas_file(const std::string& file_path);
CameraParameters read_camera_file(const std::string& file_path);

bool all_same_values(const std::vector<float>& values); 
bool are_identical(const std::vector<float>& v1, const std::vector<float>& v2);