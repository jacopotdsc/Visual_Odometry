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
 * @param input_file File where correspondences find by kd-tree are written
 * @param output_file File where the previous is cleaned from some errors
 * @return A vector which contain std::tuple<.,.> representing the correspondences
*/
IntPairVector compute_correspondences(const std::string& input_file, const std::string& output_file);

/**
 * @brief This function implement a pipeline to compute correspondence given two meas-xxxxx.dat files. It call compute_correspondences(....)
 * @param file_meas_prev First meas-xxxxx.dat file
 * @param file_meas_next Second meas-xxxxx.dat file ( ideally should be the next one of the file_meas_prev )
 * @return A vector which contain std::tuple<.,.> representing the correspondences
*/
std::pair<CorresponcesPairVector, IntPairVector> perform_correspondences(std::string file_meas_prev, std::string file_meas_next );