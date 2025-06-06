#include "utils_file.h"
#include "camera.h"
#include "picp_solver.h"
#include "epipolar_geometry.h"
#include "utils_evaluation.h"
#include "utils_map.h"

#define TOTAL_MEAS 120

int main(int argc, char* argv[]) {

    // --------------------- CAMERA ---------------------
    Camera cam = read_camera_file("../data/camera.dat");
    cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());


    // --------------------- INIT SOME VARIABLES ---------------------
    Vector3fVector world_points;
    CustomVector<Vector3fVector> vector_world_rel;
    CustomVector<IntPairVector> vector_world_correspondences;
    CustomVector<Vector10fVector> vector_world_appearances;

    // --------------------- CORRESPONDENCES ---------------------
    IntPairVector correspondences_image = perform_correspondences("../data/meas-00000.dat", "../data/meas-00001.dat");
    
    // --------------------- ESTIMATING POSE ---------------------
    PointCloud pc_prev = read_meas_file("../data/meas-00000.dat");
    PointCloud pc_curr = read_meas_file("../data/meas-00001.dat");

    Vector10fVector appearances_prev = pc_prev.extractAppearance();
    Vector10fVector appearances_curr = pc_curr.extractAppearance();
    Vector2fVector image_points_prev = pc_prev.extractImagePoints();
    Vector2fVector image_points_curr = pc_curr.extractImagePoints();

    vector_world_correspondences.push_back(correspondences_image);
    vector_world_appearances.push_back(appearances_curr);

    const Eigen::Isometry3f pose_init = estimate_transform( cam,
                                                            correspondences_image, 
                                                            image_points_prev, 
                                                            image_points_curr);
    // --------------------- TRIANGULATION ---------------------

    IntPairVector correspondences_world;
    triangulate_points( cam.cameraMatrix(), pose_init,
                        correspondences_image,
                        image_points_prev,
                        image_points_curr,
                        world_points,
                        correspondences_world);

    // Adding data for map
    vector_world_rel.push_back(world_points);
    vector_world_correspondences.push_back(correspondences_world);
    
    // --------------------- SOLVER ---------------------
    PICPSolver solver;
    solver.setKernelThreshold(10000);

    // --------------------- LOOP: Initialization ---------------------
    IsometryVector est_pose_rel;
    est_pose_rel.push_back(Eigen::Isometry3f::Identity()); // Frame 0
    est_pose_rel.push_back(pose_init); // Frame 1
    Eigen::Isometry3f pose_curr = pose_init;
    
    image_points_prev = image_points_curr;
    appearances_prev  = appearances_curr;

    Eigen::Isometry3f rf_camera_rotation = Eigen::Isometry3f::Identity();
    rf_camera_rotation.linear() = Ry( 90 * 3.14159/180 ) * Rz( -90 * 3.14159/180 );

    // --------------------- LOOP: pose estimation ---------------------
    for (int i = 2; i <= TOTAL_MEAS; i++){

        // Reading next measurement
        std::ostringstream path_ref, path_curr;
        path_ref  << "../data/meas-" << std::setw(5) << std::setfill('0') << i-1  << ".dat";
        path_curr << "../data/meas-" << std::setw(5) << std::setfill('0') << i  << ".dat";

        pc_curr = read_meas_file(path_curr.str() );
        appearances_curr   = pc_curr.extractAppearance();

        // Finding correspondences between measurements
        correspondences_image = perform_correspondences(path_ref.str(), path_curr.str());

        vector_world_correspondences.push_back(correspondences_image);
        vector_world_appearances.push_back(appearances_curr);

        // Finding correspondences world_pointswith world points
        image_points_curr = pc_curr.extractImagePoints(); 
        correspondences_world = perform_correspondences_world(correspondences_image,correspondences_world);

        // Transforming world point in the new reference frame ( reference_frame_prev )
        // It able me to set the camera pose to an indentity
        for(auto& p : world_points){
            p = pose_curr * p;          // c_p_w^i = c_T_w^(i-1) * c_p_w^(i-1) 
        }

        // Estimating pose with PICP
        cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());
        solver.init(cam, world_points, image_points_curr);
        for(int i=0; i<1000; i++){
            solver.oneRound(correspondences_world, false);
        }

        // Refreshing: saving data
        cam = solver.camera();
        pose_curr = cam.worldInCameraPose();
        est_pose_rel.push_back(pose_curr);

        // Refreshing: Triangulating points
        triangulate_points( cam.cameraMatrix(),
                            cam.worldInCameraPose(),
                            correspondences_image,
                            image_points_prev,
                            image_points_curr,
                            world_points,
                            correspondences_world);

        // Adding data for map
        vector_world_rel.push_back(world_points);
        vector_world_correspondences.push_back(correspondences_world);

        // Refreshing: overloading data
        image_points_prev = image_points_curr;
        appearances_prev = appearances_curr;
    }

    // --------------------- SAVING DATA ---------------------
    Vector7fVector gt_file = read_trajectory_file("../data/trajectory.dat");

    Vector3fVector gt_point_glob;
    for (const auto& row : gt_file) {
        gt_point_glob.push_back(Eigen::Vector3f(row[4], row[5], row[6]));
    }

    IsometryVector est_pose_glob;
    rel2Glob(est_pose_rel, est_pose_glob);

    Vector3fVector est_point_glob;
    for(const auto& pose: est_pose_glob){
        est_point_glob.push_back(pose.translation());
    }


    Vector3fVector ratio_glob;
    write_trajectory_on_file(gt_point_glob, est_point_glob, "trajectory_gt.txt", "trajectory_complete.txt");

    // --------------------- TRAJECTORY EVALUATION ---------------------
    // Augmenting groundthruth to SE(3)
    IsometryVector gt_pose_glob;
    for (const auto& point : gt_point_glob) {
        Eigen::Isometry3f gt_iso = Eigen::Isometry3f::Identity();
        gt_iso.translation() = point;
        gt_pose_glob.push_back(gt_iso);
    }

    // Converting groundthruth with relative poses
    IsometryVector gt_pose_rel;
    glob2Rel(gt_pose_glob, gt_pose_rel);

    
    // Evaluation estimated poses
    float translation_evaluation = evaluate_relative_translation(gt_pose_rel, est_pose_rel);
    float rotation_evaluation = evaluate_relative_rotation(gt_pose_rel, est_pose_rel);
    Eigen::Vector3f translation_component_wise_error = evaluate_global_translation_error(gt_pose_glob, est_pose_glob);
    Eigen::Vector3f translation_variance = evaluate_global_translation_variance(gt_pose_glob, est_pose_glob, translation_component_wise_error);

    write_pose_deltas(gt_pose_glob, est_pose_glob, ratio_glob, "delta_comparison.txt");
    write_trajectory_on_file(gt_point_glob, est_point_glob, ratio_glob, "trajectory_gt.txt", "trajectory_complete_scaled.txt");

    //print_evaluations(translation_evaluation, rotation_evaluation, translation_component_wise_error, 
    //    translation_variance, est_pose_glob, gt_pose_glob, true, "evaluation.txt");

    // --------------------- MAP EVALUATION ---------------------
    Vector14fVector world_file = read_world_file("../data/world.dat");

    CustomVector<Vector3fVector> vector_world_glob;
    rel2Glob(vector_world_rel, est_pose_rel, vector_world_glob);

    //write_world_on_file(vector_world_glob, vector_world_appearances, "world.txt");
    write_world_on_file(vector_world_glob, vector_world_appearances, ratio_glob, "world_scaled.txt");
    match_appearance_and_write("../data/world.dat", "world_scaled.txt", "result_map_scaled.txt");
    
    Eigen::Vector2f map_rmse = evaluate_map_rmse("../data/world.dat", "result_map_scaled.txt");
    int matched_points = map_rmse[1];
    Eigen::Vector3f map_error = evaluate_map_cumulative_error("../data/world.dat", "result_map_scaled.txt", matched_points); 


    std::cout << "\nRMSE: " << map_rmse[0] << ", matched point: " << map_rmse[1] << std::endl;
    std::cout << "Map cumulative error: " << map_error[0] << ", matched point: " << map_error[1] << std::endl;
    std::cout << "Mean error (x, y, z): " << map_error[0] / matched_points << ", " << map_error[1] / matched_points << ", " << map_error[2] / matched_points << std::endl;

    std::ofstream outfile("evaluation_map.txt");

    if (outfile.is_open()) {
        outfile << "\nRMSE: " << map_rmse[0] << ", matched point: " << map_rmse[1] << std::endl;
        outfile << "Map cumulative error: " << map_error[0] << ", matched point: " << map_error[1] << std::endl;
        outfile << "Mean error (x, y, z): " << map_error[0] / matched_points << ", " << map_error[1] / matched_points << ", "  << map_error[2] / matched_points << std::endl;
        outfile.close();

        std::cout << "Evaluation map written in: evaluation_map.txt" << std::endl;

    }
}