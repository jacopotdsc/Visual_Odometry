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

Vector14fVector read_world_file(const std::string& file_path) {
    Vector14fVector world_vector; ;
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    float value;

    if (!input_stream.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return world_vector; 
    }

    while (std::getline(input_stream, line)) {

        if(line.empty())
            continue;

        std::stringstream ss(line);
        Vector14f world_point;

        for (int i = 0; i < 14; i++) {
            ss >> value;
            world_point(i) = value;
        }

        world_vector.push_back(world_point);
        
    }
    input_stream.close();
    return world_vector; 
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

void write_trajectory_on_file(Vector3fVector gt_points, Vector3fVector estimated_points, Vector3fVector ratio_glob, std::string gt_file_name, std::string est_file_name){
    std::ofstream gt_file(gt_file_name);
    gt_file << "x,y,z\n";
    for (const auto& p : gt_points)
        gt_file << p.x() << "," << p.y() << "," << p.z() << "\n";
    gt_file.close();

    std::ofstream est_file(est_file_name);
    est_file << "x,y\n";

    Eigen::Vector3f ratio_sum(0.0f, 0.0f, 0.0f);
    size_t n = ratio_glob.size();
    for (const auto& r : ratio_glob) {
        ratio_sum += r;
    }
    Eigen::Vector3f ratio_avg = (n > 0) ? (ratio_sum / float(n)) : Eigen::Vector3f(1.0f, 1.0f, 1.0f);
    
    for (size_t i = 0; i < estimated_points.size(); ++i) {

        auto scaled_point = estimated_points[i].cwiseProduct(ratio_avg);  
        est_file << scaled_point.x() << "," << scaled_point.y() << "," << scaled_point.z() << "\n";
    }
    est_file.close();

    std::cout << "Curve trajectories saved to " << gt_file_name << " and " << est_file_name << std::endl;
}

void write_world_on_file(   CustomVector<Vector3fVector> vector_world_glob, 
                            CustomVector<Vector10fVector> vector_world_appearances, 
                            const std::string& filename){

    std::ofstream file(filename ); 

    if (!file.is_open()) {
        std::cerr << "[ERROR] Can't open " << filename << ".txt\n";
        return;
    }

    assert( vector_world_glob.size() == vector_world_appearances );

    // Iteration all over vector of each measurement
    for( size_t idx_meas=0; idx_meas < vector_world_appearances.size(); idx_meas++){
        
        Vector3fVector vector_world_curr = vector_world_glob[idx_meas];
        Vector10fVector vector_appearances = vector_world_appearances[idx_meas];

        assert( vector_world_curr.size() == vector_appearances.size() );

        // ITeration over the measurement
        for( size_t idx_point=0; idx_point < vector_world_curr.size(); idx_point++){
            
            Eigen::Vector3f world_point_curr = vector_world_curr[idx_point];
            Vector10f appearance_point_curr = vector_appearances[idx_point];

            // Writing coordinate
            for( const auto& coord: world_point_curr){
                file << coord  << " "; 
            }

            // Writing appearances
            for( const auto& app : appearance_point_curr){
                file << app << " ";
            }
            file << "\n";

        }
        
    }

    file.close();
    std::cout << "Writting world points in " << filename << std::endl;
}

void write_world_on_file(   CustomVector<Vector3fVector> vector_world_glob, 
                            CustomVector<Vector10fVector> vector_world_appearances, 
                            Vector3fVector ratio_glob,
                            const std::string& filename){

    std::ofstream file(filename ); 

    if (!file.is_open()) {
    std::cerr << "[ERROR] Can't open " << filename << ".txt\n";
    return;
    }

    assert( vector_world_glob.size() == vector_world_appearances );


    Eigen::Vector3f ratio_sum(0.0f, 0.0f, 0.0f);
    size_t n = ratio_glob.size();
    for (const auto& r : ratio_glob) {
        ratio_sum += r;
    }
    Eigen::Vector3f ratio_avg = (n > 0) ? (ratio_sum / float(n)) : Eigen::Vector3f(1.0f, 1.0f, 1.0f);

    // Iteration all over vector of each measurement
    for( size_t idx_meas=0; idx_meas < vector_world_appearances.size(); idx_meas++){

        Vector3fVector vector_world_curr = vector_world_glob[idx_meas];
        Vector10fVector vector_appearances = vector_world_appearances[idx_meas];

        assert( vector_world_curr.size() == vector_appearances.size() );

        // ITeration over the measurement
        for( size_t idx_point=0; idx_point < vector_world_curr.size(); idx_point++){

        Eigen::Vector3f world_point_curr = vector_world_curr[idx_point].cwiseProduct(ratio_avg); 
        Vector10f appearance_point_curr = vector_appearances[idx_point];

        // Writing coordinate
        for( const auto& coord: world_point_curr){
            file << coord  << " "; 
        }

        // Writing appearances
        for( const auto& app : appearance_point_curr){
            file << app << " ";
        }
        file << "\n";

    }

}

file.close();
std::cout << "Writting world points in " << filename << std::endl;
}

std::string join_appearance(const std::vector<std::string>& tokens, size_t start_index) {
    std::ostringstream oss;
    for (size_t i = start_index; i < tokens.size(); ++i) {
        if (i > start_index) oss << " ";
        oss << tokens[i];
    }
    return oss.str();
}

std::vector<std::string> split_line(const std::string& line) {
    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string tok;
    while (iss >> tok) {
        tokens.push_back(tok);
    }
    return tokens;
}

void match_appearance_and_write(const std::string& file1_path, const std::string& file2_path, const std::string& filename) {
    std::ifstream file1(file1_path);
    std::ifstream file2(file2_path);
    std::ofstream result(filename);

    if (!file1.is_open() || !file2.is_open()) {
        std::cerr << "Errore apertura file.\n";
        return;
    }

    // Mappa: appearance → landmark_id
    std::unordered_map<std::string, std::string> appearance_to_landmark;

    // Leggi il file 1
    std::string line;
    while (std::getline(file1, line)) {
        auto tokens = split_line(line);
        if (tokens.size() < 5) continue; // invalid line
        std::string landmark_id = tokens[0];
        std::string appearance = join_appearance(tokens, 4);
        appearance_to_landmark[appearance] = landmark_id;
    }

    // Set per evitare duplicati
    std::unordered_set<std::string> used_landmarks;

    // Leggi il file 2
    while (std::getline(file2, line)) {
        auto tokens = split_line(line);
        if (tokens.size() < 4) continue;
        std::string position = tokens[0] + " " + tokens[1] + " " + tokens[2];
        std::string appearance = join_appearance(tokens, 3);

        auto it = appearance_to_landmark.find(appearance);
        if (it != appearance_to_landmark.end()) {
            const std::string& landmark_id = it->second;
            if (used_landmarks.find(landmark_id) == used_landmarks.end()) {
                result << landmark_id << " " << position << " " << appearance << "\n";
                used_landmarks.insert(landmark_id);
            }
        }
    }

    file1.close();
    file2.close();
    result.close();


    std::cout << "Map written in " << filename << std::endl;
}
