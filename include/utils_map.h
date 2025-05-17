#include "defs.h"

/**
 * F1: chiamata nel loop prinicpale
 * del vo system, deve prendere i punti triangolati .
 * Tali punti triangualti sono i punti della mappa che 
 * interessano.
 * 
 * F2: confronta il file txt generato 
 * dalla funzioen sopra con la gt del file RMSE
 * 
 * F3: ultima funzione che fa print utili per la mappa
 */

/**
 * @brief Transforms the points into the global reference frame using the estimated poses.
 *         w_T_(pw) = w_T_(pr) * (pr)_T_c * c_T_(pw)
 *         pw = pose_world, pr = pose_relative, pw = point_world 
 * 
 * @param vector_world_rel_c_T_pw CustomVector of relative 3D points, each containing a set of 3D points
 *  to be transformed into the global reference frame.
 * @param est_pose_glob_w_T_pr IsometryVector of the estimated poses, representing the transformations 
 *  for each set of relative 3D points to obtain the global position.
 * @param vector_world_glob_w_T_pw: CustomVector to store the transformed 3D points in the global reference frame.
 */
void rel2Glob(CustomVector<Vector3fVector>& vector_world_rel, IsometryVector& est_pose_glob, CustomVector<Vector3fVector>& vector_world_glob);

/**
 * @param filename Path to the file containing landmark data.
 * @return Vector of 3D positions (x, y, z) parsed from file.
 */
Vector4fVector load_positions(const std::string& filename);

/**
 * @param gt_file  Path to ground truth file (.dat).
 * @param est_file Path to estimated file (.txt).
 * @return RMSE between ground truth and estimated 3D positions.
 */
Eigen::Vector2f evaluate_map_rmse(const std::string& gt_file, const std::string& est_file);


/**
 * @param gt_file  Path to ground truth file (.dat).
 * @param est_file Path to estimated file (.txt).
 * @return Vector4f: [cumulative_dx, cumulative_dy, cumulative_dz, matched_count]
*/
Eigen::Vector3f evaluate_map_cumulative_error(const std::string& gt_file, const std::string& est_file, int matched_point);