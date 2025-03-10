#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>

// Definizione di un vettore generico di dimensione dim
template <int dim>
using Vectorf = Eigen::Matrix<float, dim, 1>;

// Struttura per un punto con appearance
struct PointDataMeasurement {
    Eigen::Vector2f image_point;  // IMAGE_POINT
    Vectorf<10> appearance;  // APPEARANCE
};

struct CameraParameters {
    Eigen::Matrix3f K;  // Intrinsic Matrix
    Eigen::Matrix4f T_cam_robot;  // Homogeneous transform of camera
};

std::vector<PointDataMeasurement> read_meas_file(const std::string& file_path);
CameraParameters read_camera_file(const std::string& file_path);

