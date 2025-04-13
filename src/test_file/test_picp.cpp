#include "utils_file.h"
#include "utils_world.h"
#include "camera.h"
#include "picp_solver.h"
#include "defs.h"
#include "epipolar_geometry.h"
#include <opencv2/opencv.hpp>
#include <random>
#include <Eigen/Dense>

int main(int argc, char** argv) {

    std::string file_path_camera = "../data/camera.dat";
    Camera cam = read_camera_file(file_path_camera);

    Vector3fVector world_points;
    Eigen::Vector3f lower_left_bottom(-10,-10,-10);
    Eigen::Vector3f upper_right_top(10,10,10);
    int num_segments=20;
    int density = 10;
    makeWorld(world_points,
            lower_left_bottom, 
            upper_right_top,
            num_segments,
            density);

    std::cerr << "Generated Model with " << world_points.size() << " points" << std::endl;

    Vector2fVector reference_image_points;
    cam.projectPoints(reference_image_points, world_points, true); 

    std::cerr << "Point into the image " << reference_image_points.size() << " points" << std::endl;
     
    // construct a solver
    PICPSolver solver;
    solver.setKernelThreshold(10000);
    float dt = 0.1;

    for(int i=0; i < 5; i++){
        
        std::cout << "----------------" << std::endl; 

        Eigen::Isometry3f motion=Eigen::Isometry3f::Identity();
        motion.translation() = Eigen::Vector3f(-dt, 0, 0);
        
        cam.setWorldInCameraPose(motion*cam.worldInCameraPose());

        Vector2fVector current_image_points;
        const bool keep_indices=true;
        
        cam.projectPoints(current_image_points, world_points, keep_indices);
    
        IntPairVector correspondences;
        computeFakeCorrespondences(correspondences, reference_image_points, current_image_points);
        
        
        solver.init(cam,world_points,reference_image_points);
        solver.oneRound(correspondences,false);
        cam=solver.camera();

        std::cout << "Iteration " << i << std::endl;
        std::cout << "Rotation:\n" << cam.worldInCameraPose().rotation() << std::endl;
        std::cout << "Translation:\n" << cam.worldInCameraPose().translation().transpose() << std::endl;
    }


}   