#include "utils_file.h"

PointCloud read_meas_file(const std::string& file_path) {
    PointCloud points;
    std::ifstream input_stream(file_path);
    std::string line;
    if (!input_stream.is_open()){
        std::cout << "[read_meas_file] Could not open " << file_path << ", exit" << std::endl;
        exit(-1);
    }

    // Skip first three lines
    for (int i = 0; i < 3; i++)
        std::getline(input_stream, line);

    while (std::getline(input_stream, line)) {
        if (line.empty())
            continue;

        std::stringstream ss(line);

        std::string word_point;
        float value;
        Point point;

        ss >> word_point;   // point 

        ss >> value;    // POINT_ID_CURRENT_MEASUREMENT
        point.local_point_id = static_cast<int>(value);

        ss >> value;    // ACTUAL_POINT_ID
        point.actual_point_id = static_cast<int>(value);

        ss >> value;    // IMAGE_POINT x
        point.image_point(0) = value;

        ss >> value;    // IMAGE_POINT y
        point.image_point(1) = value;

        for (int i = 0; i < 10; i++) {
            ss >> value;    // APPEARANCE
            point.appaerance(i) = value;
        }

        point.normalized_image_point = point.image_point;
        points.addPoint(point);
    }

    input_stream.close();
    return points;
}

Camera read_camera_file(const std::string& file_path) {
    std::ifstream input_stream(file_path);
    std::string word;

    if (!input_stream.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return Camera();
    }

    Eigen::Matrix3f camera_matrix;
    input_stream >> word; // "camera"
    input_stream >> word; // "matrix:"
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            input_stream >> camera_matrix(i, j);
        }
    }

    // Read camera_transformation
    Eigen::Isometry3f T_cam_robot;
    input_stream >> word; // "cam_transform:"
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            input_stream >> T_cam_robot(i, j);
            //input_stream >> cam_params._world_in_camera_pose(i, j);
            //input_stream >> word;
        }
    }


    float z_near = 0.f, z_far = 0.f;
    int cols = 0, rows = 0;

    input_stream >> word; 
    input_stream >> z_near; //cam_params._z_near;

    input_stream >> word;
    input_stream >> z_far; //cam_params._z_far;

    input_stream >> word; 
    input_stream >> cols; //cam_params._cols; //width;

    input_stream >> word;
    input_stream >> rows; //cam_params._rows; //height;

    
    Camera cam_params = Camera(rows, cols, z_near, z_far, camera_matrix);

    //std::cout << "[read_camera_file] Set Isometry3f::Identity as WorldInCameraPose" << std::endl;
    //cam_params.setWorldInCameraPose(Eigen::Isometry3f::Identity());
    cam_params.setWorldInCameraPose(T_cam_robot);

    input_stream.close();
    return cam_params;
}

Vector7fVector read_trajectory_file(const std::string& file_path) {
    Vector7fVector trajectory_vector; ;
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    float value;

    if (!input_stream.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return trajectory_vector; 
    }

    while (std::getline(input_stream, line)) {

        if(line.empty())
            continue;
        std::stringstream ss(line);
        Vector7f trajectory_point;

        for (int i = 0; i < 7; i++) {
            ss >> value;
            trajectory_point(i) = value;
        }

        trajectory_vector.push_back(trajectory_point);
        
    }
    input_stream.close();
    return trajectory_vector; 
}

void write_trajectory_on_file(Vector3fVector gt_points, Vector3fVector estimated_points, std::string gt_file_name, std::string est_file_name){
    std::ofstream gt_file(gt_file_name);
    gt_file << "x,y,z\n";
    for (const auto& p : gt_points)
        gt_file << p.x() << "," << p.y() << "," << p.z() << "\n";
    gt_file.close();

    std::ofstream est_file(est_file_name);
    est_file << "x,y\n";
    for (const auto& p : estimated_points)
        est_file << p.x() << "," << p.y() << "," << p.z()  << "\n";
    est_file.close();

    std::cout << "Curve trajectories saved to " << gt_file_name << " and " << est_file_name << std::endl;
}

