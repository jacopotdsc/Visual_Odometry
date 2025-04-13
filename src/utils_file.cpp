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

// Utility for read from files
PointCloud read_meas_file(const std::string& file_path) {
    PointCloud point_cloud; // ContainerType points;
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    float value;

    if (!input_stream.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return point_cloud; 
    }

    // Skip first 3 lines ( Metadata line ) 
    for (int i = 0; i < 3; i++)
        std::getline(input_stream, line);
    
    while (std::getline(input_stream, line)) {

        if(line.empty())
            continue;

        std::stringstream ss(line);
        Point point_structure;
        Vector11f pointData;
        ss >> word; // point

        ss >> value; // POINT_ID_CURRENT_MESUREMENT
        pointData(0) = value;

        ss >> value; // ACTUAL_POINT_ID
        point_structure.actual_point_id = static_cast<int>(value);

        // Take IMAGE_POINT
        float x, y;
        ss >> x >> y; //ss >> value;
        point_structure.image_point = std::make_tuple(x, y);


        // Take APPEARANCE
        for (int i = 1; i < 11; i++) {
            ss >> value;
            //pointData.appearance(i) = value;
            pointData(i) = value;
        }

        point_structure.local_id_and_appaerance = pointData;
        point_cloud.addPoint(point_structure);
        
        //points.push_back(pointData);
    }
    input_stream.close();
    return point_cloud; // points;
}

Camera read_camera_file(const std::string& file_path) {
    Camera cam_params = Camera();
    std::ifstream input_stream(file_path);
    std::string word;

    if (!input_stream.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return cam_params;
    }

    input_stream >> word; // "camera"
    input_stream >> word; // "matrix:"
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            input_stream >> cam_params._camera_matrix(i, j);
        }
    }

    // Read camera_transformation
    input_stream >> word; // "cam_transform:"
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            //input_stream >> cam_params.T_cam_robot(i, j);
            input_stream >> cam_params._world_in_camera_pose(i, j);
        }
    }

    input_stream >> word; 
    input_stream >> cam_params._z_near;

    input_stream >> word;
    input_stream >> cam_params._z_far;

    input_stream >> word; 
    input_stream >> cam_params._cols; //width;

    input_stream >> word;
    input_stream >> cam_params._rows; //height;

    input_stream.close();
    return cam_params;
}

std::pair<CorresponcesPairVector, IntPairVector> perform_correspondences(std::string file_meas_prev, std::string file_meas_next ){
    
    // Initializing name of files
    std::string file_to_write  = getOutputFileName(file_meas_prev, file_meas_next);

    // Reading measurement files
    Vector11fVector meas_prev = read_meas_file(file_meas_prev).extractLocalIdAndAppearance();
    Vector11fVector meas_next = read_meas_file(file_meas_next).extractLocalIdAndAppearance();
    CorresponcesPairVector correspondences;
    IntPairVector int_correspondences;

    // opening file to write
    std::ofstream output(file_to_write, std::ios::out);
    if (!output.is_open()) {
        std::cerr << "Errore apertura file: " << std::endl;
    }

    // Initializing kdtree and seek for correspondences
    TreeNodeType kd_tree(meas_prev.begin(), meas_prev.end(), 10);

    for (const auto& query_point : meas_next) {
        Vector11f* nearest_neighbor = kd_tree.fullSearchCustom_v2(query_point, 1.0f);
        
        if (nearest_neighbor != nullptr) {
            for (int i = 0; i < 11; ++i) {
                output << (*nearest_neighbor)[i] << " "; 
            }
            output << std::endl;  
    
            for (int i = 0; i < 11; ++i) {
                output << query_point[i] << " ";  
            }
            output << std::endl << std::endl;  
            
            Vector11f prev_point(*nearest_neighbor);
            Vector11f next_point(query_point);
            correspondences.push_back(std::make_pair(prev_point, next_point));
            int_correspondences.push_back(std::make_pair(prev_point[0], next_point[0]));
    
        } else {
            //cout << "Nothing found for this query." << endl;
        }
    }

    return std::make_pair(correspondences, int_correspondences);
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