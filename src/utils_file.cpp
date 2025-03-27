#include "defs.h"
#include "utils_file.h"

// Auxiliar functions
bool all_same_values(const std::vector<float>& values) {
    return std::all_of(values.begin() + 1, values.end(), [&](float v) { return v == values[1]; });
}
  
bool are_identical(const std::vector<float>& v1, const std::vector<float>& v2) {
    return std::equal(v1.begin()+1, v1.end(), v2.begin()+1);
}

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

// Utility for read from files
PointCloud read_meas_file(const std::string& file_path) {
    PointCloud point_cloud; // ContainerType points;
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    float value;

    if (!input_stream.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return point_cloud; //return points;
    }

    // Skip first 3 lines ( Metadata line ) 
    for (int i = 0; i < 3; i++)
        std::getline(input_stream, line);
    
    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        Point point_structure;
        Vectorf<11> pointData;
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

CameraParameters read_camera_file(const std::string& file_path) {
    CameraParameters cam_params;
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
            input_stream >> cam_params.K(i, j);
        }
    }

    // Read camera_transformation
    input_stream >> word; // "cam_transform:"
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            input_stream >> cam_params.T_cam_robot(i, j);
        }
    }

    input_stream >> word; 
    input_stream >> cam_params.z_near;

    input_stream >> word;
    input_stream >> cam_params.z_far;

    input_stream >> word; 
    input_stream >> cam_params.width;

    input_stream >> word;
    input_stream >> cam_params.height;

    input_stream.close();
    return cam_params;
}

// Utility to extract correspondences
CorresponcesPairVector compute_correspondences(const std::string& input_file, const std::string& output_file) {
    std::ifstream input(input_file);
    std::ofstream output(output_file);

    CorresponcesPairVector pairVector;
  
    if (!input.is_open()) {
        std::cerr << "Error opening file: " << input_file << std::endl;
    }
  
    if (!output.is_open()) {
        std::cerr << "Error opening output file: " << output_file << std::endl;
    }
  
    while (true) {
      std::string line1, line2, line_dummy;
      if (!getline(input, line1)) { break; }
      if (!getline(input, line2)) { break; }
      if (!getline(input, line_dummy)) {}
  
      std::stringstream ss1(line1), ss2(line2);
      std::vector<float> values1, values2;
      float value;
  
      while (ss1 >> value) { values1.push_back(value);}
      while (ss2 >> value) { values2.push_back(value); }
  
      if (all_same_values(values1) || all_same_values(values2)) { continue; }
  
      if (are_identical(values1, values2)) {
          for (const float& v : values1) {
              output << v << " ";
          }
          output << std::endl;
  
          for (const float& v : values2) {
              output << v << " ";
          }
          output << std::endl << std::endl;

          pairVector.push_back(std::make_pair(values1, values2));
  
      }
    }

    input.close();
    output.close();
    std::cout << "File processed and saved as: " << output_file << ", return vector with correspondences" << std::endl;
    return pairVector;
}

CorresponcesPairVector perform_correspondences(std::string file_meas_prev, std::string file_meas_next ){
    
    // Initializing name of files
    std::string file_to_write  = getOutputFileName(file_meas_prev, file_meas_next);
    std::string file_cleaned = file_to_write.substr( 0, file_to_write.rfind(".txt")) + "_cleaned.txt";  

    // Reading measurement files
    std::vector<Vectorf<11>> meas_prev = read_meas_file(file_meas_prev).extractLocalIdAndAppearance();
    std::vector<Vectorf<11>> meas_next = read_meas_file(file_meas_next).extractLocalIdAndAppearance();

    // opening file to write
    std::ofstream output(file_to_write, std::ios::out);
    if (!output.is_open()) {
        std::cerr << "Errore apertura file: " << std::endl;
    }

    // Initializing kdtree and seek for correspondences
    TreeNodeType kd_tree(meas_prev.begin(), meas_prev.end(), 10);

    for (const auto& query_point : meas_next) {
        Vectorf<11>* nearest_neighbor = kd_tree.fullSearchCustom_v2(query_point, 1.0f);
        
        if (nearest_neighbor != nullptr) {
            for (int i = 0; i < 11; ++i) {
                output << (*nearest_neighbor)[i] << " "; 
            }
            output << std::endl;  
    
            for (int i = 0; i < 11; ++i) {
                output << query_point[i] << " ";  
            }
            output << std::endl << std::endl;  
        } else {
            //cout << "Nothing found for this query." << endl;
        }
    }

    // Clearing the output and return vector of correspondences
    return compute_correspondences(file_to_write, file_cleaned);
}


