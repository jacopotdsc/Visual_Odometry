#include "utils_file.h"
#include "camera.h"
#include "picp_solver.h"
#include "epipolar_geometry.h"
#include "utils_evaluation.h"

#define TOTAL_MEAS 120

int main(int argc, char* argv[]) {

    // --------------------- CAMERA ---------------------
    Camera cam = read_camera_file("../data/camera.dat");
    cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());

    // --------------------- CORRESPONDENCES ---------------------
    IntPairVector correspondences_image = perform_correspondences("../data/meas-00000.dat", "../data/meas-00001.dat");
    
    // --------------------- ESTIMATING POSE ---------------------
    PointCloud pc_prev = read_meas_file("../data/meas-00000.dat");
    PointCloud pc_curr = read_meas_file("../data/meas-00001.dat");

    Vector10fVector appearances_prev = pc_prev.extractAppearance();
    Vector10fVector appearances_curr = pc_curr.extractAppearance();
    Vector2fVector image_points_prev = pc_prev.extractImagePoints();
    Vector2fVector image_points_curr = pc_curr.extractImagePoints();

    const Eigen::Isometry3f pose_init = estimate_transform( cam,
                                                            correspondences_image, 
                                                            image_points_prev, 
                                                            image_points_curr);

    // --------------------- TRIANGULATION ---------------------
    Vector3fVector world_points;
    IntPairVector correspondences_world;
    triangulate_points( cam.cameraMatrix(), pose_init,
                        correspondences_image,
                        image_points_prev,
                        image_points_curr,
                        world_points,
                        correspondences_world);

    // --------------------- SOLVER ---------------------
    PICPSolver solver;
    solver.setKernelThreshold(10000);

    // --------------------- LOOP: Initialization ---------------------
    IsometryVector trajectory;
    trajectory.push_back(Eigen::Isometry3f::Identity()); // Frame 0
    trajectory.push_back(pose_init); // Frame 1
    Eigen::Isometry3f pose_curr = pose_init;
    
    image_points_prev = image_points_curr;
    appearances_prev  = appearances_curr;
    
    // --------------------- LOOP: pose estimation ---------------------
    for (int i = 2; i <= TOTAL_MEAS; i++){


        //std::cout << pose_curr.translation().transpose() << std::endl;

        // Reading next measurement
        std::ostringstream path_ref, path_curr;
        path_ref  << "../data/meas-" << std::setw(5) << std::setfill('0') << i-1  << ".dat";
        path_curr << "../data/meas-" << std::setw(5) << std::setfill('0') << i  << ".dat";

        pc_curr = read_meas_file(path_curr.str() );
        appearances_curr   = pc_curr.extractAppearance();

        // Finding correspondences between measurements
        correspondences_image = perform_correspondences(path_ref.str(), path_curr.str());

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
        trajectory.push_back(pose_curr);

        // Refreshing: Triangulating points
        triangulate_points( cam.cameraMatrix(),
                            cam.worldInCameraPose(),
                            correspondences_image,
                            image_points_prev,
                            image_points_curr,
                            world_points,
                            correspondences_world);

        // Refreshing: overloading data
        image_points_prev = image_points_curr;
        appearances_prev = appearances_curr;
    }

    // --------------------- SAVING DATA ---------------------
    Vector7fVector gt_trajectory_full = read_trajectory_file("../data/trajectory.dat");

    Vector3fVector gt_points;
    for (const auto& row : gt_trajectory_full) {
        gt_points.push_back(Eigen::Vector3f(row[4], row[5], row[6]));
    }

    IsometryVector estimated_poses;
    rel2Glob(trajectory, estimated_poses);

    Vector3fVector estimated_points;
    for(const auto& pose: estimated_poses){
        estimated_points.push_back(pose.translation());
    }

    write_trajectory_on_file(gt_points, estimated_points, "trajectory_gt.txt", "trajectory_complete.txt");
}

