#pragma once
#include "defs.h"
#include "utils_file.h"

/**
 * @param camera The camera parameters, including intrinsic parameters like the focal length and principal point.
 * @param point_cloud The point cloud containing the 2D measurements (image points) to be normalized.
 * @return Eigen::Matrix3f A 3x3 matrix containing the matrix which normalized image_pooint
 */
Eigen::Matrix3f normalize_measurement(const CameraParameters& camera, PointCloud& point_cloud);

/**
 * @param camera_params The camera parameters, including intrinsic parameters, used for normalization.
 * @param point_cloud_1 The first point cloud, containing measurements (image points) from the first camera view.
 * @param point_cloud_2 The second point cloud, containing measurements (image points) from the second camera view.
 * @param correspondence_vector A vector of pairs of corresponding points between the two images.
 * @return Eigen::Matrix3f A 3x3 fundamental matrix that relates the two images via epipolar geometry.
 */
Eigen::Matrix3f eight_point_algorithm(CameraParameters& camera_params, PointCloud& point_cloud_1, PointCloud& point_cloud_2, CorresponcesPairVector correspondence_vector);

/**
 * @param camera_params The camera parameters, including the intrinsic camera matrix (K), used to transform the fundamental matrix into the essential matrix.
 * @param F The fundamental matrix computed from the 8-point algorithm.
 * @return Eigen::Matrix3f A 3x3 essential matrix that describes the epipolar geometry after applying the camera's intrinsic parameters.
 */
Eigen::Matrix3f compute_essential_matrix(CameraParameters& camera_params, Eigen::Matrix3f F);