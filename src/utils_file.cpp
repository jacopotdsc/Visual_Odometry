#include "utils_file.h"

ContainerType read_meas_file(const std::string& file_path) {
    //std::vector<PointDataMeasurement> points;
    ContainerType points;
    std::ifstream input_stream(file_path);
    std::string word;
    std::string line;
    float value;

    if (!input_stream.is_open()) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return points;
    }

    // SSkipp first 3 lines ( Metadata line ) 
    for (int i = 0; i < 3; i++)
        std::getline(input_stream, line);
    
    // Leggere i punti
    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        //PointDataMeasurement pointData;
        Vectorf<11> pointData;
        ss >> word; // point

        ss >> value; // POINT_ID_CURRENT_MESUREMENT
        pointData(0) = value;

        ss >> word; // ACTUAL_POINT_ID

        // Take IMAGE_POINT
        for (int i = 0; i < 2; i++) {
            ss >> value;
            //pointData.image_point(i) = value;
        }

        // Take APPEARANCE
        for (int i = 1; i < 11; i++) {
            ss >> value;
            //pointData.appearance(i) = value;
            pointData(i) = value;
        }
        
        points.push_back(pointData);
    }
    input_stream.close();
    return points;
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

    input_stream.close();
    return cam_params;
}