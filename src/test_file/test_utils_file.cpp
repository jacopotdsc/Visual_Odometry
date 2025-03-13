#include <iostream>
#include "utils_file.h"
#include "PointCloud.h"

int main() {
    std::cout << "TEST: read_meas_file" << std::endl;
    std::string file_path_meas = "../data/meas-00000.dat"; 
    //std::vector<PointDataMeasurement> points = read_meas_file(file_path_meas);
    std::vector<Vectorf<11>> points = read_meas_file(file_path_meas);
    
    std::cout << "Read " << points.size() << " from point cloud." << std::endl;
    for (size_t i = 0; i < size_t(5); i++) {
        std::cout << "Point " << points[i][0] //<< " (local ID: " << points[i].point_id_current_measurement
                    //<< ", Actual ID: " << points[i].actual_point_id << ") : (" 
                  //<< points[i].image_point(0) << ", "
                  //<< points[i].image_point(1) << ") "
                  << ", Appearance: [";
        for (int j = 1; j < 12; j++) {
            std::cout << points[i](j);
            if (j < 10) std::cout << ", ";
        }
        std::cout << "]\n";
    }


    /********************************************************************/
    
    std::cout << "\nTEST: read_camera_file" << std::endl;
    std::string file_path_camera = "../data/camera.dat";
    CameraParameters cam_params = read_camera_file(file_path_camera);
    std::cout << "Intrinsic Matrix (K):\n" << cam_params.K << "\n";
    std::cout << "Camera-to-Robot Transformation Matrix (T_cam_robot):\n" << cam_params.T_cam_robot << "\n";

    /********************************************************************/

    std::cout << "\nTEST: PointCloud" << std::endl;
    //PointCloud point_cloud(points);

    //std::cout << "PointCloud contains " << point_cloud.size() << " points.\n";

}