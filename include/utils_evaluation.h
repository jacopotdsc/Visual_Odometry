#include "defs.h"

/**
 * @param relative_poses IsometryVector of estimated poses with PICP
 * @param global_poses Output vector with global poses consider also the RF of the camera
 */
void rel2Glob(IsometryVector& relative_poses, IsometryVector& global_poses);

/**
 * Converts global poses to relative poses
 * @param global_poses Vector of global poses
 * @param relative_poses Output vector of relative poses
 */
void glob2Rel(IsometryVector& global_poses, IsometryVector& relative_poses);

/**
 * Evaluate pose estimation error using translation and rotation
 * @param gt_relative_poses Ground truth relative poses
 * @param estimated_relative_poses Estimated relative poses
 * @return sum_i( norm(rel_T(1:3, 4))/norm(rel_GT(1:3, 4)) )
 */
float evaluate_relative_translation(IsometryVector& gt_relative_poses, IsometryVector& est_relative_poses);

/**
Evaluate pose estimation error using translation and rotation
 * @param gt_relative_poses Ground truth relative poses
 * @param estimated_relative_poses Estimated relative poses
 * @return sum_i( Trace(eye(3)-error_T(1:3, 1:3)) ), with error_T = inv(rel_T)*rel_GT
 */
float evaluate_relative_rotation(IsometryVector& gt_relative_poses, IsometryVector& est_relative_poses);

/**
 * @param gt_global_poses Vector of ground truth global poses (Isometry3f)
 * @param est_global_poses Vector of estimated global poses (Isometry3f)
 * @return Eigen::Vector3f A vector containing the average error along the (x, y, z) components
 */
Eigen::Vector3f evaluate_global_translation_error(const IsometryVector& gt_global_poses, const IsometryVector& est_global_poses);

/**
 * @param gt_global_poses Vector of ground truth global poses (Isometry3f)
 * @param est_global_poses Vector of estimated global poses (Isometry3f)
 * @param mean_translation_error The mean translation error vector (x, y, z)
 * @return Eigen::Vector3f A vector containing the variance of the error along the (x, y, z) components
 */
Eigen::Vector3f evaluate_global_translation_variance(const IsometryVector& gt_global_poses, const IsometryVector& est_global_poses, const Eigen::Vector3f& mean_translation_error);
    
/**
 * @brief Write the delta (relative transform) between estimated and ground truth poses to a file.
 *
 * @param est_pose_glob Vector of estimated global poses (Isometry3f)
 * @param gt_pose_glob Vector of ground truth global poses (Isometry3f)
 * @param output_filename Name of the output file where deltas will be saved (default: "delta_comparison.txt")
 */
void write_pose_deltas(const IsometryVector& est_pose_glob, const IsometryVector& gt_pose_glob, const std::string& output_filename);

/**
 * @brief print evaluation
 */
void print_evaluations(float translation_evaluation, float rotation_evaluation, 
    const Eigen::Vector3f& translation_component_wise_error, 
    const Eigen::Vector3f& translation_variance, 
    const IsometryVector& est_pose_glob, 
    const IsometryVector& gt_pose_glob,
    bool write_to_file, const std::string& filename = "");