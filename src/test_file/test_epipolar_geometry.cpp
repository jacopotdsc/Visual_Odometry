#include "utils_file.h"
#include "epipolar_geometry.h"
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    std::string file_path_meas = "../data/meas-00000.dat"; 
    std::string file_path_camera = "../data/camera.dat";

    Camera camera_params = read_camera_file(file_path_camera);
    PointCloud point_cloud = read_meas_file(file_path_meas);

    /********************************************************************/
    std::cout << "\nTEST: normalize_measurement" << std::endl;
    Eigen::Matrix3f T = normalize_measurement(camera_params, point_cloud);
    
    std::cout << "\n Normalizer matrix:\n" << T << std::endl;

    const auto& points = point_cloud.getPoints();
    for (size_t i = 0; i < 5 && i < points.size(); ++i) {
        const auto& point = points[i];
        
        const auto& image_point = point.image_point;
        std::cout << "Point " << point.local_id_and_appaerance[0] << " image point: ("
                  << std::get<0>(image_point) << ", "
                  << std::get<1>(image_point) << ") ";
        
        const auto& normalized_image_point = point.normalized_image_point;
        std::cout << "--> ("
                  << std::get<0>(normalized_image_point) << ", "
                  << std::get<1>(normalized_image_point) << ")\n";
    } 
    /********************************************************************/
    std::cout << "\nTEST: Initial Estimant adn triangulation" << std::endl;

     // --------------------- CAMERA ---------------------
     Camera cam = read_camera_file(file_path_camera);
 
     // --------------------- MEASUREMENTS ---------------------
     std::string file_path_meas_0 = "../data/meas-00000.dat";
     std::string file_path_meas_1 = "../data/meas-00001.dat";
 
     PointCloud pc0 = read_meas_file(file_path_meas_0);
     PointCloud pc1 = read_meas_file(file_path_meas_1);
 
     Vector2fVector p1_img = pc0.extractImagePoints();  // frame 0
     Vector2fVector p2_img = pc1.extractImagePoints();  // frame 1
 
     // --------------------- CORRESPONDENZE ---------------------
     auto result = perform_correspondences(file_path_meas_0, file_path_meas_1);
     CorresponcesPairVector point_pairs = result.first;
     IntPairVector index_pairs = result.second;
 
     std::cout << "Correspondences: : " << index_pairs.size() << "\n";
 
     // --------------------- STIMA TRANSFORMAZIONE RELATIVA ---------------------
     Eigen::Isometry3f estimated_T = estimate_transform(cam._camera_matrix, index_pairs, p1_img, p2_img);
     std::cout << "Estimated transformation \n" << estimated_T.matrix() << "\n";
    
     Vector3fVector triangulated_points;
     int n_triangulated = triangulate_points(cam._camera_matrix, estimated_T, index_pairs, p1_img, p2_img, triangulated_points);
 
     std::cout << "Triangulated " << n_triangulated << " points\n";
     for (int i = 0; i < 5; ++i) {
         std::cout << "Point " << i << ": " << triangulated_points[i].transpose() << "\n";
     }


    //Eigen::Matrix3f F = eight_point_algorithm(camera_params, point_cloud_1, point_cloud_2, correspondence_vector);
    //Eigen::Matrix3f E = compute_essential_matrix(camera_params, F);

    //std::cout << "\nFoundamental matrix: \n" << F << std::endl;
    //std::cout << "\nEssential matrix:\n" << E << std::endl;

    //Eigen::Matrix3f K = camera_params._camera_matrix;
    //Eigen::Matrix3f F_back = K.transpose().inverse()*E*K.inverse();
    //std::cout << "\nTesting from E to F:\n" << F_back << std::endl;

    //Eigen::Matrix3f error_matrix = F_back - F;
    //std::cout << "\nError matrix (F_back - F):\n" << error_matrix << std::endl;

    /********************************************************************/
    //std::cout << "\nTEST: Decomposing matrix E" << std::endl;

    //auto [R1, R2, t] = compute_rotation_translation(E);

    //std::cout << "Rotation Matrix R1:\n" << R1 << std::endl;
    //std::cout << "Rotation Matrix R2:\n" << R2 << std::endl;
    //std::cout << "Translation Vector t:\n" << t << std::endl;

    /********************************************************************/
    //std::cout << "\nTEST: Estimating transformation" << std::endl;

    //Eigen::Isometry3f trasformation = estimate_transform(camera_params._camera_matrix, 
    //                                                    correspondence_vector,
    //                                                    point_cloud_1, point_cloud_2, R1, R2, t);

    //std::cout << "Estimated transformation: \n" << trasformation.matrix() << std::endl;

}