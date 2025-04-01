#include <iostream>
#include "utils_file.h"
#include "PointCloud.h"
#include "camera.h"

int main() {
    std::cout << "TEST: read_meas_file" << std::endl;
    std::string file_path_meas = "../data/meas-00000.dat"; 
    //std::vector<PointDataMeasurement> points = read_meas_file(file_path_meas);
    PointCloud point_cloud = read_meas_file(file_path_meas);
    const auto point_vec = point_cloud.getPoints();

    if (point_vec.empty()) {
        std::cerr << "Error: No points found in the point cloud." << std::endl;
        return -1; 
    }

    std::cout << "Read " << point_vec.size() << " from point cloud." << std::endl;
    for (size_t i = 0; i < size_t(5); i++) {
        auto [x, y] = point_vec[i].image_point;

        std::cout << "Point " << point_vec[i].local_id_and_appaerance[0] 
                    << ", ( Actual ID: " << point_vec[i].actual_point_id << ") : (" 
                    << x << ", "
                    << y << ") "
                    << ", Appearance: [";
        for (int j = 1; j < 11; j++) {
            std::cout << point_vec[i].local_id_and_appaerance[j];
            if (j < 10) std::cout << ", ";
        }
        std::cout << "]\n";
    }


    /********************************************************************/
    
    std::cout << "\nTEST: read_camera_file" << std::endl;
    std::string file_path_camera = "../data/camera.dat";
    Camera cam_params = read_camera_file(file_path_camera);
    std::cout << "Intrinsic Matrix (K):\n" << cam_params._camera_matrix << "\n";
    //std::cout << "Camera-to-Robot Transformation Matrix (T_cam_robot):\n" << cam_params.T_cam_robot << "\n";
    std::cout << "z_near: " << cam_params._z_near << "\n";
    std::cout << "z_far: " << cam_params._z_far << "\n";
    std::cout << "width: " << cam_params._cols << "\n";
    std::cout << "height: " << cam_params._rows << "\n";

    /********************************************************************/

    //std::cout << "\nTEST: PointCloud" << std::endl;
    //PointCloud point_cloud(points);

    //std::cout << "PointCloud contains " << point_cloud.size() << " points.\n";

}