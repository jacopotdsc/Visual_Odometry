#include "defs.h"
#include "camera.h"
#include "epipolar_geometry.h"
#include "picp_solver.h"
#include "utils_evaluation.h"
#include "utils_world.h"

#define TOTAL_MEAS 120

IntPairVector extract_correspondences_world(const IntPairVector& correspondences_imgs,const IntPairVector& correspondences_world){
    IntPairVector correspondences; correspondences.reserve(correspondences_imgs.size());

    for(size_t i=0;i<correspondences_imgs.size();i++){
        const int idx_ref=correspondences_imgs[i].first;
        for(size_t j=0;j<correspondences_world.size();j++){
            if(correspondences_world[j].first==idx_ref){
                correspondences.push_back(IntPair(correspondences_imgs[i].second,correspondences_world[j].second));
                break;
            }
        }
    }
    return correspondences;

}


int main (int argc, char** argv) {
    
    // --------------------- CAMERA ---------------------
    std::string file_path_camera = "../data/camera.dat";
    Camera cam = read_camera_file(file_path_camera);
    Eigen::Matrix3f K = cam.cameraMatrix();

    // --------------------- MEASUREMENTS ---------------------
    std::string file_path_meas_0 = "../data/meas-00000.dat";
    std::string file_path_meas_1 = "../data/meas-00001.dat";

    PointCloud pc0 = read_meas_file(file_path_meas_0);
    PointCloud pc1 = read_meas_file(file_path_meas_1);

    Vector2fVector image_points_0 = pc0.extractImagePoints();
    Vector2fVector image_points_1 = pc1.extractImagePoints();
    Vector2fVector current_image_points = image_points_1; 

    // --------------------- CORRESPONDENCES ---------------------
    auto result = perform_correspondences(file_path_meas_0, file_path_meas_1);
    CorresponcesPairVector point_pairs = result.first;
    IntPairVector index_pairs = result.second;

    std::cout << "[INFO] Correspondences: " << index_pairs.size() << "\n";

    // --------------------- ESTIMATING POSE ---------------------
    Eigen::Isometry3f init_pose = estimate_transform(K, index_pairs, image_points_0, image_points_1);
    std::cout << "[Epipolar] Initial pose (frame 0 -> 1):\n" << init_pose.matrix() << "\n";

    // --------------------- TRIANGULATION ---------------------
    Vector3fVector world_points;
    triangulate_points(K, init_pose, index_pairs, image_points_0, image_points_1, world_points);

    std::cout << "[INFO] World points triangolati: " << world_points.size() << "\n";

    // --------------------- SOLVER ---------------------
    PICPSolver solver;
    solver.setKernelThreshold(10000);

    IsometryVector trajectory;
    trajectory.push_back(Eigen::Isometry3f::Identity()); // Frame 0
    trajectory.push_back(init_pose);                     // Frame 1

    Eigen::Isometry3f current_pose = init_pose;
    
    // --------------------- LOOP ---------------------
    for (int i = 1; i < TOTAL_MEAS; ++i) { 

        std::cout << "Current iteration: " << i << std::endl;

        // Reading measurement
        std::ostringstream prev_path, curr_path;
        prev_path << "../data/meas-" << std::setw(5) << std::setfill('0') << i - 1 << ".dat";
        curr_path << "../data/meas-" << std::setw(5) << std::setfill('0') << i << ".dat";

        PointCloud meas_prev = read_meas_file(prev_path.str());
        PointCloud meas_curr = read_meas_file(curr_path.str());

        // Finding correspondences between measurements
        auto result = perform_correspondences(prev_path.str(), curr_path.str());
        CorresponcesPairVector correspondences = result.first;
        IntPairVector index_corr = result.second;

        Vector2fVector img_prev, img_curr;
        for (const auto& [id1, id2] : index_corr) {
            Point point1 = meas_prev.getPointWithId(id1);
            Point point2 = meas_curr.getPointWithId(id2);
        
            float x1 = std::get<0>(point1.image_point);
            float y1 = std::get<1>(point1.image_point);
            float x2 = std::get<0>(point2.image_point);
            float y2 = std::get<1>(point2.image_point);
        
            img_prev.emplace_back(x1, y1);
            img_curr.emplace_back(x2, y2);
        }
        // Mapping of 3D points
        IntPairVector map_ref_to_world;
        for (size_t i = 0; i < index_corr.size(); ++i) {
            map_ref_to_world.emplace_back(index_corr[i].first, i);  // world_points[i]
        }

        // Ora puoi ottenere (curr_idx, world_idx)
        IntPairVector map_curr_to_world = extract_correspondences_world(index_corr, map_ref_to_world);


        // Transform 3D point in the current system
        Vector3fVector transformed_points;
        for (const auto& p : world_points)
            transformed_points.push_back(current_pose * p);

        // Estimating pose with PICP
        cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());
        solver.init(cam, transformed_points, img_curr);
        for(int j=0; j<100; j++){
            //solver.oneRound(index_corr, false);
            solver.oneRound(map_curr_to_world, false);
        }

        cam = solver.camera();
        current_pose = cam.worldInCameraPose();
        trajectory.push_back(current_pose);

        std::cout << "[PICP] Frame " << i << " pose:\n" << current_pose.matrix() << "\n";

        // Triangulate new points
        world_points.clear();
        triangulate_points(K, current_pose, index_corr, img_prev, img_curr, world_points, map_curr_to_world);
        std::cout << "[INFO] World points triangulated: " << world_points.size() << "\n";
        int n_valid = 0;
        for (const auto& p : world_points) {
            if (!p.allFinite()) {
                std::cerr << "[ERROR] Triangolato punto NON valido (NaN o Inf): " << p.transpose() << "\n";
            } else {
                n_valid++;
            }
        }
    }
 
    
    // --------------------- SAVING DATA ---------------------
    std::string file_path_trajectory = "../data/trajectory.dat";
    Vector7fVector gt_trajectory_full = read_trajectory_file(file_path_trajectory);

    Vector3fVector gt_points;
    for (const auto& row : gt_trajectory_full) {
        gt_points.push_back(row.segment<3>(4));
    }

    Vector3fVector estimated_points;
    for (const auto& pose : trajectory) {  
        estimated_points.push_back(pose.translation());
    }

    write_trajectory_on_file(gt_points, estimated_points);
    
    std::cout << "VO completed. Trajectory saved to 'trajectory_estimated.txt'\n";
    
}