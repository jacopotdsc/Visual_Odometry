#include "utils_evaluation.h"

void rel2Glob(IsometryVector& relative_poses, IsometryVector& global_poses){

    global_poses.clear();
    global_poses.reserve(relative_poses.size());

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

    Eigen::Isometry3f rf_camera_rotation = Eigen::Isometry3f::Identity();
    rf_camera_rotation.linear() = Ry( 90 * 3.14159/180 ) * Rz( -90 * 3.14159/180 );

    Eigen::Isometry3f pose_global = Eigen::Isometry3f::Identity();
    for (const auto& pose : relative_poses) { 
        
        pose_global = pose_global * pose.inverse();
        auto computed_global_pose = rf_camera_rotation * pose_global;
        global_poses.push_back(computed_global_pose);
    }
}

void glob2Rel(IsometryVector& global_poses, IsometryVector& relative_poses){

    relative_poses.clear();
    relative_poses.reserve(relative_poses.size());

    Eigen::Isometry3f pose_relative = Eigen::Isometry3f::Identity();
    relative_poses.push_back ( pose_relative );

    for( size_t i=1; i < global_poses.size(); i++){
        pose_relative = pose_relative.inverse() * global_poses[i];
        relative_poses.push_back( pose_relative );
    }

}

float evaluate_relative_translation(IsometryVector& gt_relative_poses, IsometryVector& est_relative_poses){
    
    assert( gt_relative_poses.size() == estimated_relative_poses.size());
    float translation_evaluation = 0;

    for( size_t i=0; i < gt_relative_poses.size(); i++){

        Eigen::Isometry3f rel_T = est_relative_poses[i];
        Eigen::Isometry3f rel_GT = gt_relative_poses[i];

        // norm(rel_T(1:3, 4))/norm(rel_GT(1:3, 4))
        if(rel_GT.translation().norm()  == 0.0)
            continue;

        translation_evaluation += rel_T.translation().norm() / rel_GT.translation().norm(); 
    }

    return translation_evaluation;
}

float evaluate_relative_rotation(IsometryVector& gt_relative_poses, IsometryVector& est_relative_poses){
    
    assert( gt_relative_poses.size() == estimated_relative_poses.size());
    float rotation_evaluation = 0;

    for( size_t i=0; i < gt_relative_poses.size(); i++){

        Eigen::Isometry3f rel_T = est_relative_poses[i];
        Eigen::Isometry3f GT_0 = i == 0? gt_relative_poses[i] :  gt_relative_poses[i-1];
        Eigen::Isometry3f GT_1 = gt_relative_poses[i];

        // trace(eye(3)-error_T(1:3, 1:3)), with rel_GT = inv(GT_0)*GT_1
        Eigen::Isometry3f rel_GT = (GT_0.inverse())*GT_1;
        Eigen::Isometry3f error_T = ( rel_T * rel_GT).inverse();

        rotation_evaluation += ( Eigen::Matrix3f::Identity() - error_T.linear() ).trace();
    }

    return rotation_evaluation;
}

Eigen::Vector3f evaluate_global_translation_error(const IsometryVector& gt_global_poses, const IsometryVector& est_global_poses) {
    assert(gt_global_poses.size() == est_global_poses.size());

    float error_x = 0;
    float error_y = 0;
    float error_z = 0;

    Eigen::Vector3f translation_error(0.0f, 0.0f, 0.0f);  

    for (size_t i = 0; i < gt_global_poses.size(); ++i) {

        Eigen::Vector3f gt_translation = gt_global_poses[i].translation();
        Eigen::Vector3f est_translation = est_global_poses[i].translation();

        error_x += gt_translation[0] - est_translation[0];
        error_y += gt_translation[1] - est_translation[1];
        error_z += gt_translation[2] - est_translation[2];
    }

    return Eigen::Vector3f(error_x, error_y, error_z);
}

Eigen::Vector3f evaluate_global_translation_variance(const IsometryVector& gt_global_poses, const IsometryVector& est_global_poses, const Eigen::Vector3f& translation_error_vector) {
    assert(gt_global_poses.size() == est_global_poses.size());

    float var_x = 0.0f;
    float var_y = 0.0f;
    float var_z = 0.0f;

    Eigen::Vector3f mean_translation_error = translation_error_vector / gt_global_poses.size();

    for (size_t i = 0; i < gt_global_poses.size(); ++i) {
        Eigen::Vector3f gt_translation = gt_global_poses[i].translation();
        Eigen::Vector3f est_translation = est_global_poses[i].translation();

        var_x += std::pow( (gt_translation[0] - est_translation[0]) - mean_translation_error[0], 2);
        var_y += std::pow( (gt_translation[1] - est_translation[1]) - mean_translation_error[1], 2);
        var_z += std::pow( (gt_translation[2] - est_translation[2]) - mean_translation_error[2], 2);
    }

    return Eigen::Vector3f(var_x, var_y, var_z) / ( gt_global_poses.size() - 1); 
}

void print_evaluations(float translation_evaluation, float rotation_evaluation, 
                       const Eigen::Vector3f& translation_component_wise_error, 
                       const Eigen::Vector3f& translation_variance, 
                       const IsometryVector& est_pose_glob, 
                       const IsometryVector& gt_pose_glob,
                       bool write_to_file, const std::string& filename) {
    // If the flag is true, open the file
    std::ofstream output_file;
    if (write_to_file) {
        output_file.open(filename);
        if (!output_file) {
            std::cerr << "Error opening file for writing." << std::endl;
            return;
        }
    }

    // Print to console and file
    std::cout << "\n------- Evaluation README:" << std::endl;
    std::cout << "Translation Evaluation: " << translation_evaluation << std::endl;
    std::cout << "Rotation Evaluation: " << rotation_evaluation << std::endl;

    if (write_to_file) {
        output_file << "\n------- Evaluation README:" << std::endl;
        output_file << "Translation Evaluation: " << translation_evaluation << std::endl;
        output_file << "Rotation Evaluation: " << rotation_evaluation << std::endl;
    }

    // Mean section
    std::cout << "\n------- Evaluation README Mean:" << std::endl;
    std::cout << "Translation Evaluation Mean: " << translation_evaluation / est_pose_glob.size() << std::endl;
    std::cout << "Rotation Evaluation Mean: " << rotation_evaluation / est_pose_glob.size() << std::endl;

    if (write_to_file) {
        output_file << "\n------- Evaluation README Mean:" << std::endl;
        output_file << "Translation Evaluation Mean: " << translation_evaluation / est_pose_glob.size() << std::endl;
        output_file << "Rotation Evaluation Mean: " << rotation_evaluation / est_pose_glob.size() << std::endl;
    }

    // Other Evaluation section
    std::cout << "\n------- Other Evaluation:" << std::endl;
    std::cout << "Translation Error: " << translation_component_wise_error.transpose() << std::endl;
    std::cout << "Mean Translation Error: " << (translation_component_wise_error / gt_pose_glob.size()).transpose() << std::endl;
    std::cout << "Translation Variance: " << translation_variance.transpose() << std::endl;

    if (write_to_file) {
        output_file << "\n------- Other Evaluation:" << std::endl;
        output_file << "Translation Error: " << translation_component_wise_error.transpose() << std::endl;
        output_file << "Mean Translation Error: " << (translation_component_wise_error / gt_pose_glob.size()).transpose() << std::endl;
        output_file << "Translation Variance: " << translation_variance.transpose() << std::endl;
    }

    // Close the file if it was opened
    if (write_to_file) {
        std::cout << "\nEvaluation saved in: " << filename << std::endl;
        output_file.close();
    }
}

