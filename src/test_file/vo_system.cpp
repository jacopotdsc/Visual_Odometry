#include "defs.h"
#include "camera.h"
#include "epipolar_geometry.h"
#include "picp_solver.h"
#include "utils_evaluation.h"

#define TOTAL_MEAS 120

int main (int argc, char** argv) {

    // Reading camera parameters an first measurement
    std::string file_path_camera = "../data/camera.dat";
    Camera cam = read_camera_file(file_path_camera);

    // Reading first two frames
    std::string file_path_meas_0 = "../data/meas-00000.dat";
    std::string file_path_meas_1 = "../data/meas-00001.dat";

    //auto [point_pairs, index_pairs] = perform_correspondences(file_path_meas_0, file_path_meas_1);

    std::pair<CorresponcesPairVector, IntPairVector> result = perform_correspondences(file_path_meas_0, file_path_meas_1);
    CorresponcesPairVector point_pairs = result.first;
    IntPairVector index_pairs = result.second;

    Vector2fVector coordinates_meas_0, coordinates_meas_1;
    for (const auto& [m0, m1] : point_pairs) {
        coordinates_meas_0.push_back(m0.segment<2>(1)); // image point from meas-00000
        coordinates_meas_1.push_back(m1.segment<2>(1)); // image point from meas-00001
    }

    // Initial estimation
    PointCloud point_cloud_0 = read_meas_file(file_path_meas_0);
    PointCloud point_cloud_1 = read_meas_file(file_path_meas_1);
    Eigen::Matrix3f F = eight_point_algorithm(cam, point_cloud_0, point_cloud_1, point_pairs);
    Eigen::Matrix3f E = compute_essential_matrix(cam, F);
    Eigen::Matrix3f R1, R2;
    Eigen::Vector3f t;
    std::tie(R1, R2, t) = compute_rotation_translation(E);

    Eigen::Isometry3f init_pose = estimate_transform(cam.cameraMatrix(), point_pairs, point_cloud_0, point_cloud_1, R1, R2, t);
    std::cout << "[Epipolar] Initial pose (frame 0 -> 1):\n" << init_pose.matrix() << "\n";

    
    // Triangulate initial 3D points
    Vector3fVector world_points;
    triangulate_points(cam.cameraMatrix(), init_pose, point_pairs, point_cloud_0, point_cloud_1, world_points);

    // Initialize PICP solver
    PICPSolver solver;
    solver.setKernelThreshold(10000);

    IsometryVector trajectory;
    trajectory.push_back(Eigen::Isometry3f::Identity()); // Frame 0
    trajectory.push_back(init_pose);                     // Frame 1

    Eigen::Isometry3f current_pose = init_pose;
    
    for (int i = 2; i < TOTAL_MEAS; ++i) {

        std::cout << "Current iteration: " << i << std::endl;

        std::ostringstream prev_path, curr_path;
        prev_path << "../data/meas-" << std::setw(5) << std::setfill('0') << i - 1 << ".dat";
        curr_path << "../data/meas-" << std::setw(5) << std::setfill('0') << i << ".dat";

        PointCloud meas_prev = read_meas_file(prev_path.str());
        PointCloud meas_curr = read_meas_file(curr_path.str());

        //auto [correspondences, index_corr] = perform_correspondences(prev_path.str(), curr_path.str());
        std::pair<CorresponcesPairVector, IntPairVector> result = perform_correspondences(file_path_meas_0, file_path_meas_1);
        CorresponcesPairVector correspondences = result.first;
        IntPairVector index_corr = result.second;

        Vector2fVector img_prev, img_curr;
        for (const auto& [m0, m1] : correspondences) {
            img_prev.push_back(m0.segment<2>(1));
            img_curr.push_back(m1.segment<2>(1));
        }

        // Trasforma i punti 3D nel sistema attuale
        Vector3fVector transformed_points;
        for (const auto& p : world_points)
            transformed_points.push_back(current_pose * p);
        
        // Stima nuova pose con PICP
        cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());
        solver.init(cam, transformed_points, img_curr);

        solver.oneRound(index_corr, false);
 
        std::cout << " round fatto" << std::endl;
        cam = solver.camera();
        current_pose = cam.worldInCameraPose();
        trajectory.push_back(current_pose);
 
        std::cout << "[PICP] Frame " << i << " pose:\n" << current_pose.matrix() << "\n";

        // Triangola nuovi punti per aggiornare la mappa
        triangulate_points(cam.cameraMatrix(), current_pose, correspondences, meas_prev, meas_curr, world_points);
    }
 

    // Extraicitn ground thruht and comparing trajectory
    std::string file_path_trajectory = "../data/trajectory.dat";
    Vector7fVector gt_trajectory_full = read_trajectory_file(file_path_trajectory);

    Vector3fVector gt_points;
    for (const auto& row : gt_trajectory_full) {
        gt_points.push_back(row.segment<3>(4)); // [4],[5],[6]
    }

    Vector3fVector estimated_points;
    for (const auto& pose : trajectory) {  
        estimated_points.push_back(pose.translation());
    }

    write_trajectory_on_file(gt_points, estimated_points);

    std::cout << "VO completed. Trajectory saved to 'trajectory_estimated.txt'\n";
    
}