#include "utils_file.h"

/***********************************************************/
// A couples of auxiliar functions usefull for other ones

/**
 * @param file_path_1, file_path_2 Path of meas-xxxxx.dat file 
 * @return A string to have a file.txt name in function of meas-xxxxx.dat files
 */
std::string getOutputFileName(const std::string& file_path_1, const std::string& file_path_2) {
    // Estrai i numeri dai percorsi dei file
    std::regex rgx("meas-(\\d+).dat");
    std::smatch match_1, match_2;

    if (std::regex_search(file_path_1, match_1, rgx) && std::regex_search(file_path_2, match_2, rgx)) {
        // Estrai i numeri dai match
        std::string num1 = match_1[1];
        std::string num2 = match_2[1];

        // Crea il nome del file di output
        return "meas-" + num1 + "-" + num2 + ".txt";
    } else {
        // Se non trovi il formato corretto nel percorso dei file
        return "invalid_file_name";
    }
}

/**********************************************************/
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

IntPairVector perform_correspondences(std::string file_meas_prev, std::string file_meas_next){
    
    // Initializing name of files
    std::string file_to_write  = getOutputFileName(file_meas_prev, file_meas_next);

    // Reading measurement files
    PointCloud pc_prev = read_meas_file(file_meas_prev);
    PointCloud pc_next = read_meas_file(file_meas_next); 
    Vector11fVector meas_prev = pc_prev.extractLocalIdAndAppearance();
    Vector11fVector meas_next = pc_next.extractLocalIdAndAppearance();
    CorresponcesPairVector correspondences;
    IntPairVector int_correspondences;

    // opening file to write
    //std::ofstream output(file_to_write, std::ios::out);
    //if (!output.is_open()) {
    //    std::cerr << "Errore apertura file: " << std::endl;
    //}

    // Initializing kdtree and seek for correspondences
    TreeNodeType kd_tree(meas_prev.begin(), meas_prev.end(), 10);

    for (const auto& query_point : meas_next) {
        Vector11f* nearest_neighbor = kd_tree.fullSearchCustom(query_point, 0.1f);

        if (nearest_neighbor != nullptr) {
            for (int i = 0; i < 11; ++i) {
                //output << (*nearest_neighbor)[i] << " "; 
            }
            //output << std::endl;  
    
            for (int i = 0; i < 11; ++i) {
                //output << query_point[i] << " ";  
            }
            //output << std::endl << std::endl;  
            
            Vector11f prev_point(*nearest_neighbor);
            Vector11f next_point(query_point);
            //correspondences.push_back(std::make_pair(prev_point, next_point));
            int_correspondences.push_back(std::make_pair(prev_point[0], next_point[0]));
    
        } else {
            //std::cout << "Nothing found for this query." << std::endl;
        }
    }

    return int_correspondences;
    //return std::make_pair(correspondences, int_correspondences);
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

IntPairVector perform_correspondences_world(const IntPairVector& correspondences_image,const IntPairVector& correspondences_world){
    IntPairVector correspondences; 

    for(size_t i=0; i<correspondences_image.size(); i++){
        const int idx_ref=correspondences_image[i].first;   // ID reference point
         
        for(size_t j=0; j<correspondences_world.size(); j++){
            const int idx_ref_world = correspondences_world[j].first; // ID reference point traingulated point

            if( idx_ref_world == idx_ref){
                correspondences.push_back(
                    IntPair(correspondences_image[i].second,    // Id image current point
                            correspondences_world[j].second)    // ID triangulated point
                    );
                break;
            }
        }
    }
    return correspondences;
}