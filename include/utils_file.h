#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>

template <int dim>
using Vectorf = Eigen::Matrix<float, dim, 1>;

struct PointDataMeasurement {
    using Scalar = float;
    static constexpr int RowsAtCompileTime = 10;

    int point_id_current_measurement;
    int actual_point_id; 
    Eigen::Vector2f image_point;  // IMAGE_POINT
    Vectorf<11> appearance;  // APPEARANCE

    PointDataMeasurement operator-(const PointDataMeasurement& other) const {
        PointDataMeasurement result = *this;
        result.appearance = appearance - other.appearance;
        return result;
    }

    PointDataMeasurement operator+(const PointDataMeasurement& other) const {
        PointDataMeasurement result = *this;
        result.appearance = appearance + other.appearance;
        return result;
    }

    float dot(const PointDataMeasurement& other) const {
        return appearance.dot(other.appearance);
    }

    Vectorf<11> transpose() const {
        return appearance.transpose();
    }
};

struct CameraParameters {
    Eigen::Matrix3f K;  // Intrinsic Matrix
    Eigen::Matrix4f T_cam_robot;  // Homogeneous transform of camera
};

std::vector<Vectorf<11>> read_meas_file(const std::string& file_path);
CameraParameters read_camera_file(const std::string& file_path);

