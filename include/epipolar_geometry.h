#pragma once
#include "defs.h"
#include "utils_file.h"
#include "camera.h"
#include <Eigen/Dense>

/** 
 * triangulate points given their projections on two images, and the relative pose between the cameras. Computes also
 * a new set of correspondances between the triangulated points and the points in the second image
 * @param k: 3x3 camera matrix
 * @param X: relative pose of the first camera expressed in the frame of the second
 * @param correspondences: correspondences (first: idx of the point in the first image, second: idx of the corresponding point in the second image)
 * @param p1_img: points in the first image
 * @param p2_img: points in the second image
 * @param triangulated: this vector will contain the triangulated points
 * @param correspondences_new: this vector will contain the new correspondances (first: id of the measurement, second: id of the triangulated point)
 * @returns the number of successfully triangulated points
 */
int triangulate_points(const Eigen::Matrix3f& k, const Eigen::Isometry3f& X, const IntPairVector& correspondences,
    const Vector2fVector& p1_img, const Vector2fVector& p2_img, Vector3fVector& triangulated, IntPairVector& correspondences_new);

/**
 * @param p image_point to be normalized
 * @param T normalizing matrix
 * @return normalized points
 */
Vector2fVector normalize_measurement(const Vector2fVector& p,Eigen::Matrix3f& T);

/**
 * Estimates the relative pose of the first camera expressed in the frame of the second
 * @param k 3x3 camera matrix
 * @param correspondences correspondences (first: idx of the point in the first image, second: idx of the corresponding point in the second image)
 * @param p1_img points in the first image
 * @param p2_img points in the second image
 * @returns Estimatated pose
 */ 
const Eigen::Isometry3f estimate_transform(const Camera& camera, const IntPairVector& correspondences, 
    const Vector2fVector& p1_img, const Vector2fVector& p2_img);
