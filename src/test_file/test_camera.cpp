#include "utils_file.h"
#include "camera.h"
#include "epipolar_geometry.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>

Vector3fVector generateRandomWorldPoints(int num_points) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-10.0, 10.0);  // Intervallo [-10, 10] per i punti nel mondo

    Vector3fVector world_points;
    for (int i = 0; i < num_points; ++i) {
        Vector3f point(dis(gen), dis(gen), dis(gen));  
        world_points.push_back(point);
    }
    return world_points;
}

int main(int argc, char** argv) {
    std::string file_path_meas = "../data/meas-00000.dat"; 
    std::string file_path_camera = "../data/camera.dat";

    Camera camera_params = read_camera_file(file_path_camera);

    int num_points = 100;
    Vector3fVector world_points = generateRandomWorldPoints(num_points);

    Vector2fVector image_points;
    int num_points_inside = camera_params.projectPoints(image_points, world_points, true);

    std::cout << "Number of points inside the image: " << num_points_inside << std::endl;

    for (size_t i = 0; i < image_points.size(); i++) {
        std::cout << "Image Point " << i << ": " << image_points[i].transpose() << std::endl;
    }

    return 0;
}
