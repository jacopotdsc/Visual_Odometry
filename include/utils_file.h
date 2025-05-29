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
 * @param file_path Path to a world.dat file
 * @return Vector containing information of world.dat
*/
Vector14fVector read_world_file(const std::string& file_path);

/**
 * @brief write on file ground thruth point and estimated ones
 */
void write_trajectory_on_file(Vector3fVector gt_points, Vector3fVector estimated_points, std::string gt_file_name, std::string est_file_name);

/**
 * @brief write on file ground thruth point and scaled estimated onee
 */
void write_trajectory_on_file(Vector3fVector gt_points, Vector3fVector estimated_points, Vector3fVector ratio_glob, std::string gt_file_name, std::string est_file_name);

/**
 * @brief write on file world points
 */
void write_world_on_file(   CustomVector<Vector3fVector> vector_world_glob, CustomVector<Vector10fVector> vector_world_appearances, const std::string& filename);

/**
 * @brief write on file world points
 */
void write_world_on_file(   CustomVector<Vector3fVector> vector_world_glob, CustomVector<Vector10fVector> vector_world_appearances, Vector3fVector ratio_glob, const std::string& filename);

/**
 * @brief estimate map and write it
 */
void match_appearance_and_write(const std::string& file1_path, const std::string& file2_path, const std::string& filename);
 