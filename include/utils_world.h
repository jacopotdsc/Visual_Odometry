#include "defs.h"

/**
 * @brief Generates fake correspondences between two sets of 2D image points.
 *        Assumes that points in both vectors are pre-aligned and ordered.
 * 
 * @param correspondences Output vector that will contain index pairs (i, i) representing matches.
 * @param reference_image_points 2D points from the reference image (e.g., frame 0).
 * @param current_image_points 2D points from the current image (e.g., moving frame).
 */
void computeFakeCorrespondences(IntPairVector& correspondences,const Vector2fVector reference_image_points, const Vector2fVector current_image_points);

/**
 * @brief Adds evenly spaced points along a 3D segment defined by two endpoints.
 * 
 * @param dest Output vector where new 3D points will be appended.
 * @param start Starting point of the segment.
 * @param end Ending point of the segment.
 * @param density Number of points per unit length to be placed along the segment.
 */
void putPointsOnSegment3D(Vector3fVector& dest,const Eigen::Vector3f& start, const Eigen::Vector3f& end, float density);

/**
 * @brief Generates a synthetic 3D world by creating random line segments within a bounding box
 *        and populating them with points.
 * 
 * @param world_points Output vector to store the generated 3D points.
 * @param lower_left_bottom The minimum corner of the 3D bounding box.
 * @param upper_right_top The maximum corner of the 3D bounding box.
 * @param num_segments Number of random segments to generate.
 * @param density Number of points per unit length for each segment.
 */
void makeWorld(Vector3fVector& world_points, const Eigen::Vector3f& lower_left_bottom, const Eigen::Vector3f& upper_right_top, int num_segments, float density);

/**
 * @brief Generates a random 3D isometric transformation (rotation + translation).
 * @param X Output isometry (Eigen::Isometry3f) containing the generated transformation.
 */
void generate_isometry3f(Eigen::Isometry3f& X);

/**
*/
std::pair<int, int> counter_equal(CorresponcesPairVector point_pairs);