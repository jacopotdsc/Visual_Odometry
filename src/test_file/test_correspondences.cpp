#include "defs.h"
#include "utils_file.h"
#include "picp_solver.h"
#include "camera.h"
#include "picp_solver.h"
#include "utils_world.h"
#include "epipolar_geometry.h"
#include <iostream>
#include <unordered_map>

std::pair<int, int> counter_equal(CorresponcesPairVector point_pairs){
    int counter_equal = 0;
    int counter_different = 0;
    for (int i = 0; i < point_pairs.size(); i++) {
        const auto& tuple = point_pairs[i];
        const auto& v1 = tuple.first;
        const auto& v2 = tuple.second;
    
        bool equal = true;
        for (int j = 1; j < 11; ++j) { 
            if (std::abs(v1[j] - v2[j]) > 0) {
                equal = false;
                counter_different += 1;
                std::cout << "-------\ntuple " << i << " == "<< (equal ? "UGUALI" : "DIVERSI") << "\n\t[1] " << v1.transpose()
                          << "\n\t[2] " << v2.transpose()<< "\n";
                break;
            }
        }
        counter_equal += 1;

        //std::cout << "-------\ntuple " << i << " == "<< (equal ? "UGUALI" : "DIVERSI") << "\n\t[1] " << v1.transpose()
        //          << "\n\t[2] " << v2.transpose()<< "\n";
    }

    return std::make_pair(counter_equal, counter_different);
}

int main(int argc, char** argv) {
    // --------------------- CAMERA ---------------------
    std::string file_path_camera = "../data/camera.dat";
    Camera cam = read_camera_file(file_path_camera);

    // --------------------- MEASUREMENTS ---------------------
    std::string file_path_meas_0 = "../data/meas-00017.dat";
    std::string file_path_meas_1 = "../data/meas-00018.dat";

    PointCloud pc0 = read_meas_file(file_path_meas_0);
    PointCloud pc1 = read_meas_file(file_path_meas_1);

    Vector2fVector p1_img = pc0.extractImagePoints();  // frame 0
    Vector2fVector p2_img = pc1.extractImagePoints();  // frame 1

    // --------------------- CORRESPONDENZE ---------------------
    auto result = perform_correspondences(file_path_meas_0, file_path_meas_1);
    CorresponcesPairVector point_pairs = result.first;
    IntPairVector index_pairs = result.second;

    auto result_equal = counter_equal(point_pairs);
    int counter_equal = result_equal.first;
    int counter_different = result_equal.second;

    std::cout << "\n[INFO] equal vector: " << counter_equal << ", differnt vectors: " << counter_different << std::endl;
    std::cout << "[INFO] Corrispondenze trovate: " << index_pairs.size() << "\n\n";

    // --------------------- STIMA TRANSFORMAZIONE RELATIVA ---------------------
    Eigen::Isometry3f estimated_T = estimate_transform(cam._camera_matrix, index_pairs, p1_img, p2_img);
    std::cout << "=== Trasformazione Stimata (T_cam0_to_cam1) ===\n" << estimated_T.matrix() << "\n";
   
    Vector3fVector triangulated_points;
    int n_triangulated = triangulate_points(cam._camera_matrix, estimated_T, index_pairs, p1_img, p2_img, triangulated_points);

    std::cout << "Triangolati " << n_triangulated << " punti.\n";
    for (int i = 0; i < 5; ++i) {
        std::cout << "Punto " << i << ": " << triangulated_points[i].transpose() << "\n";
    }

    /*
    // --------------------- STIMA POSA INIZIALE CON EPIPOLARITÀ ---------------------
    Eigen::Matrix3f F = eight_point_algorithm(cam, pc0, pc1, point_pairs);
    Eigen::Matrix3f E = compute_essential_matrix(cam, F);

    Eigen::Matrix3f R1, R2;
    Eigen::Vector3f t;
    std::tie(R1, R2, t) = compute_rotation_translation(E);

    Eigen::Isometry3f init_pose = estimate_transform(cam.cameraMatrix(), point_pairs, pc0, pc1, R1, R2, t);
    std::cout << "[Epipolar] Initial pose (frame 0 -> 1):\n" << init_pose.matrix() << "\n";

    // --------------------- TRIANGOLAZIONE PER WORLD POINTS ---------------------
    Vector3fVector world_points;
    triangulate_points(cam.cameraMatrix(), init_pose, point_pairs, pc0, pc1, world_points);


    std::cout << "[INFO] World points triangolati: " << world_points.size() << "\n";
    std::cout << "[INFO] Corrispondenze valide: " << index_pairs.size() << "\n";

    // --------------------- Test con perform_correspondences ---------------------
    std::cout << "\n=== Test 1: perform_correspondences + oneRound ===\n";

    PICPSolver solver1;
    solver1.init(cam, world_points, current_image_points);
    solver1.oneRound(index_pairs, false);

    // --------------------- Test con computeFakeCorrespondences ---------------------
    std::cout << "\n=== Test 2: computeFakeCorrespondences + oneRound ===\n";

    IntPairVector fake_correspondences;
    computeFakeCorrespondences(fake_correspondences, current_image_points, current_image_points);

    PICPSolver solver2;
    solver2.init(cam, world_points, current_image_points);
    solver2.oneRound(fake_correspondences, false);
    */
    std::cout << "Programma finito\n";

}
