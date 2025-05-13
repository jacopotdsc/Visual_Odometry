#pragma once
#include "defs.h"
#include "PointCloud.h"
#include "camera.h"

/**
 * @param file_path Path to a meas-xxxxx.dat file
 * @return PointCloud class, it manage the meas-xxxxx.dat informations
*/
PointCloud read_meas_file(const std::string& file_path);

/**
 * @param file_path Path to a camera.dat file
 * @return Camera class, it manage the camera.dat informations
*/
Camera read_camera_file(const std::string& file_path);

/**
 * @param file_path Path to a trajectory.dat file
 * @return Vector containing information of trajectory.dat
*/
Vector7fVector read_trajectory_file(const std::string& file_path);

/**
 * @brief write on file ground thruth point and estimated onees
 */
void write_trajectory_on_file(Vector3fVector gt_points, Vector3fVector estimated_points, std::string gt_file_name, std::string est_file_name);
