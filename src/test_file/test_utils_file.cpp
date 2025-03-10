#include <iostream>
#include "utils_file.h"

int main() {
    std::string file_path_meas = "../data/meas-00000.dat"; 
    std::vector<PointDataMeasurement> points = read_meas_file(file_path_meas);
    
    if (points.empty()) {
        std::cerr << "No points have been read" << std::endl;
        return -1;
    }
    
    std::cout << "TEST: read_meas_file" << std::endl;
    std::cout << "Read " << points.size() << " from point cloud." << std::endl;
    for (size_t i = 0; i < std::min(points.size(), size_t(5)); i++) {
        std::cout << "Point " << i << ": (" 
                  << points[i].image_point(0) << ", "
                  << points[i].image_point(1) << ") "
                  << "Appearance: [";
        for (int j = 0; j < 10; j++) {
            std::cout << points[i].appearance(j);
            if (j < 9) std::cout << ", ";
        }
        std::cout << "]\n";
    }


    /********************************************************************/
    
    std::cout << "TEST: read_camera_file" << std::endl;
    std::string file_path_camera = "../data/camera.dat";
    CameraParameters cam_params = read_camera_file(file_path_camera);
    std::cout << "Intrinsic Matrix (K):\n" << cam_params.K << "\n";
    std::cout << "Camera-to-Robot Transformation Matrix (T_cam_robot):\n" << cam_params.T_cam_robot << "\n";
}