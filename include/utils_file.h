#pragma once
#include "defs.h"

struct CameraParameters {
    Eigen::Matrix3f K;  // Intrinsic Matrix
    Eigen::Matrix4f T_cam_robot;  // Homogeneous transform of camera
};

ContainerType read_meas_file(const std::string& file_path);
CameraParameters read_camera_file(const std::string& file_path);

CorresponcesPairVector compute_correspondences(const std::string& input_file, const std::string& output_file);
CorresponcesPairVector perform_correspondences(std::string file_meas_prev, std::string file_meas_next );