#include "defs.h"
#include "camera.h"
#include "epipolar_geometry.h"
#include "picp_solver.h"
#include "utils_evaluation.h"
#include "utils_world.h"

#define TOTAL_MEAS 120

int main (int argc, char** argv) {

    // --------------------- CAMERA ---------------------
    std::string file_path_camera = "../data/camera.dat";
    Camera cam = read_camera_file(file_path_camera);
    Eigen::Matrix3f K = cam.cameraMatrix();

    // --------------------- MEASUREMENTS ---------------------
    std::string file_path_meas_0 = "../data/meas-00006.dat";
    std::string file_path_meas_1 = "../data/meas-00007.dat";

    PointCloud pc0 = read_meas_file(file_path_meas_0);
    PointCloud pc1 = read_meas_file(file_path_meas_1);

    Vector2fVector image_points_0 = pc0.extractImagePoints();
    Vector2fVector image_points_1 = pc1.extractImagePoints();
    Vector2fVector current_image_points = image_points_1; // serve dopo per il solver

    // --------------------- CORRISPONDENZE ---------------------
    auto result = perform_correspondences(file_path_meas_0, file_path_meas_1);
    CorresponcesPairVector point_pairs = result.first;
    IntPairVector index_pairs = result.second;

    std::cout << "[INFO] Corrispondenze trovate: " << index_pairs.size() << "\n";

    // --------------------- STIMA POSA CON EPIPOLARITÀ ---------------------
    Eigen::Isometry3f init_pose = estimate_transform(K, index_pairs, image_points_0, image_points_1);
    std::cout << "[Epipolar] Initial pose (frame 0 -> 1):\n" << init_pose.matrix() << "\n";

    // --------------------- TRIANGOLAZIONE ---------------------
    Vector3fVector world_points;
    triangulate_points(K, init_pose, index_pairs, image_points_0, image_points_1, world_points);

    std::cout << "[INFO] World points triangolati: " << world_points.size() << "\n";

    // --------------------- Test con perform_correspondences ---------------------
    std::cout << "\n=== Test 1: perform_correspondences + oneRound ===\n";

    PICPSolver solver1;
    solver1.init(cam, world_points, current_image_points);
    solver1.oneRound(index_pairs, false);

    // --------------------- Test con corrispondenze finte ---------------------
    std::cout << "\n=== Test 2: computeFakeCorrespondences + oneRound ===\n";

    IntPairVector fake_correspondences;
    computeFakeCorrespondences(fake_correspondences, current_image_points, current_image_points);

    PICPSolver solver2;
    solver2.init(cam, world_points, current_image_points);
    solver2.oneRound(fake_correspondences, false);

    std::cout << "Programma finito\n";

}