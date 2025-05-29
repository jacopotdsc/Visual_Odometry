#include "utils_map.h"

void rel2Glob(CustomVector<Vector3fVector>& vector_world_rel, IsometryVector& est_pose_rel, CustomVector<Vector3fVector>& vector_world_glob){

    assert(vector_world_points.size() == est_pose_rel.size()-1);
    
    /*
    world_frame:             camera_frame:
     
         z                       
         ^                        
         |                        
         |______> y     x <_______
        /                        / |
       x                        z   y

    w_R_c: Ry(-90°)*Rz(90°) Euler angles
    */

    Vector3fVector current_world_glob;
    Eigen::Isometry3f rf_camera_rotation = Eigen::Isometry3f::Identity();
    rf_camera_rotation.linear() = Ry( 90 * 3.14159/180 ) * Rz( -90 * 3.14159/180 );
    rf_camera_rotation.translation() = Eigen::Vector3f(0.2, 0.0, 0.0);

    Eigen::Isometry3f pose_global = Eigen::Isometry3f::Identity();
    for( size_t i=0; i < vector_world_rel.size(); i++){

        current_world_glob.clear();
        Vector3fVector current_world_rel = vector_world_rel[i];
        Eigen::Isometry3f current_pose_rel = est_pose_rel[i];

        pose_global = pose_global * current_pose_rel.inverse();
        auto computed_global_pose = rf_camera_rotation * pose_global;
        
        for(const auto& wp : current_world_rel){
            current_world_glob.push_back( computed_global_pose * wp );
        }
        vector_world_glob.push_back( current_world_glob );
    }
}

Vector4fVector load_positions(const std::string& filename) {
    std::ifstream file(filename);
    Vector4fVector positions;

    if (!file.is_open()) {
        std::cerr << "Errore apertura file: " << filename << "\n";
        return positions;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string id_str;
        float id, x, y, z;
        iss >> id_str >> x >> y >> z;

        try {
            id = std::stof(id_str);
        } catch (...) {
            continue;
        }

        if (iss.fail()) continue;
        positions.push_back({id, x, y, z});
    }

    return positions;
}

Eigen::Vector2f evaluate_map_rmse(const std::string& gt_file, const std::string& est_file) {
    Vector4fVector gt_positions = load_positions(gt_file);
    Vector4fVector est_positions = load_positions(est_file);

    if (gt_positions.empty() || est_positions.empty()) {
        std::cerr << "[ERROR] Empty file\n";
        return Eigen::Vector2f(-1.0, -1.0);
    }

    float sum_squared_error = 0.0f;
    int matched_points = 0;

    for (const auto& est : est_positions) {
        float est_id = est[0];
        for (const auto& gt : gt_positions) {
            if (gt[0] == est_id) {
                float dx = gt[1] - est[1];
                float dy = gt[2] - est[2];
                float dz = 0.0f; //gt[3] - est[3];
                sum_squared_error += dx * dx + dy * dy + dz * dz;
                ++matched_points;
                break;
            }
        }
    }

    if (matched_points == 0) {
        std::cerr << "[ERROR] No landarmk matching\n";
        return Eigen::Vector2f(-1.0, -1.0);
    }

    float mse = sum_squared_error / matched_points;
    return Eigen::Vector2f(std::sqrt(mse), matched_points);
}

Eigen::Vector3f evaluate_map_cumulative_error(const std::string& gt_file, const std::string& est_file, int matched_points) {
    Vector4fVector gt_positions = load_positions(gt_file);
    Vector4fVector est_positions = load_positions(est_file);

    if (gt_positions.empty() || est_positions.empty()) {
        std::cerr << "[ERROR] Empty file\n";
        return Eigen::Vector3f(-1.0f, -1.0f, -1.0f);
    }

    float sum_dx = 0.0f, sum_dy = 0.0f, sum_dz = 0.0f;

    for (const auto& est : est_positions) {
        float est_id = est[0];
        for (const auto& gt : gt_positions) {
            if (gt[0] == est_id) {
                float dx = std::abs(gt[1] - est[1]);
                float dy = std::abs(gt[2] - est[2]);
                float dz = std::abs(gt[3] - est[3]);
                sum_dx += dx;
                sum_dy += dy;
                sum_dz += dz;
                break;
            }
        }
    }

    if (matched_points == 0) {
        std::cerr << "[ERROR] No landmark matching\n";
        return Eigen::Vector3f(-1.0f, -1.0f, -1.0f);
    }

    return Eigen::Vector3f(sum_dx, sum_dy, sum_dz);
}

