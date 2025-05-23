#include <iostream>
#include "utils_file.h"
#include "PointCloud.h"
#include "camera.h"

int main() {
    std::cout << "TEST: read_meas_file" << std::endl;
    std::string file_path_meas = "../data/meas-00000.dat"; 
    
    PointCloud point_cloud = read_meas_file(file_path_meas);
    const auto point_vec = point_cloud.getPoints();

    if (point_vec.empty()) {
        std::cerr << "Error: No points found in the point cloud." << std::endl;
        return -1; 
    }

    std::cout << "Read " << point_vec.size() << " from point cloud." << std::endl;
    for (size_t i = 0; i < size_t(5); i++) {
        float x = point_vec[i].image_point[0];
        float y = point_vec[i].image_point[1];

        std::cout << "Point " << point_vec[i].local_point_id
                    << ", ( Actual ID: " << point_vec[i].actual_point_id << ") : (" 
                    << x << ", "
                    << y << ") "
                    << ", Appearance: [";
        for (int j = 0; j < 10; j++) {
            std::cout << point_vec[i].appaerance[j];
            if (j < 9) std::cout << ", ";
        }
        std::cout << "]\n";
    }


    /********************************************************************/
    
    std::cout << "\nTEST: read_camera_file" << std::endl;
    std::string file_path_camera = "../data/camera.dat";
    Camera cam_params = read_camera_file(file_path_camera);
    std::cout << "Intrinsic Matrix (K):\n" << cam_params.cameraMatrix() << "\n";
    //std::cout << "Camera-to-Robot Transformation Matrix (T_cam_robot):\n" << cam_params.T_cam_robot << "\n";
    std::cout << "z_near: " << cam_params.z_near() << "\n";
    std::cout << "z_far: " << cam_params.z_far() << "\n";
    std::cout << "width: " << cam_params.cols() << "\n";
    std::cout << "height: " << cam_params.rows() << "\n";

    /********************************************************************/

    std::cout << "\nTEST: read_trajectory_file" << std::endl;
    std::string file_path_trajectory = "../data/trajectory.dat";
    Vector7fVector trajectory_vector = read_trajectory_file(file_path_trajectory);

    for(int i=15; i<20; i++){
        std::cout << "Trajectory vector: " << trajectory_vector[i].transpose() << " \n";
    }
}