#include "defs.h"
#include "utils_world.h"
#include "utils_file.h"
#include "epipolar_geometry.h"
#include "picp_solver.h"

void print_comparison(const Eigen::Isometry3f& X_est, const Eigen::Isometry3f& X_gt,const std::string title={}){
    if(!title.empty())
        std::cout << title << std::endl;
    std::cout <<"R estimated:\n";
    std::cout << X_est.linear() << std::endl;
    std::cout << "R gt:\n";
    std::cout << X_gt.linear() << std::endl;
    const Eigen::Vector3f t_est=X_est.translation();
    const Eigen::Vector3f t_gt=X_gt.translation();
    //std::cout << "t ratio: ";
    //for (int i=0;i<3;i++)
    //    std::cout << t_est(i)/t_gt(i) << ", ";
    //std::cout << std::endl;

    std::cout << "Translation error (norm): " << (t_est - t_gt).norm() << std::endl;
    std::cout << "Translation estimated: " << t_est.transpose() << std::endl;
    std::cout << "Translation ground truth: " << t_gt.transpose() << std::endl;
}

int main(int argc, char** argv) {

    // --------------------- CAMERA ---------------------
    std::string file_path_camera = "../data/camera.dat";
    Camera cam = read_camera_file(file_path_camera);
    //std::cout << "[CAMPS] Camera matrix:\n" << cam.cameraMatrix() << std::endl;

    // --------------------- MEASUREMENTS ---------------------
    std::ostringstream oss_curr, oss_new;
    int id_file_curr = 0;
    int id_file_new = id_file_curr + 1;
    oss_curr << "../data/meas-" << std::setw(5) << std::setfill('0') << id_file_curr << ".dat";
    oss_new  << "../data/meas-" << std::setw(5) << std::setfill('0') << id_file_new  << ".dat";

    std::string file_path_meas_curr = oss_curr.str();
    std::string file_path_meas_new  = oss_new.str();

    PointCloud pc_curr = read_meas_file(file_path_meas_curr);
    PointCloud pc_new  = read_meas_file(file_path_meas_new);

    Vector2fVector reference_image_points = pc_curr.extractImagePoints();
    Vector2fVector current_measurements   = pc_new.extractImagePoints();

    std::cout << "[MEAS] current: " << file_path_meas_curr << std::endl;
    std::cout << "[MEAS] new:     " << file_path_meas_new  << std::endl;
    std::cout << "[PROJT] Points in reference_image_points: " << reference_image_points.size() << std::endl;
    std::cout << "[PROJT] Points in current_measurements:    " << current_measurements.size() << std::endl;

    // --------------------- CORRISPONDENZE ---------------------
    auto result_corr = perform_correspondences(file_path_meas_curr, file_path_meas_new);
    CorresponcesPairVector corr_pair = result_corr.first;
    IntPairVector correspondences = result_corr.second;

    std::cout << "[CORRS] Correspondences: " << correspondences.size() << std::endl;

    // --------------------- STIMA TRASFORMAZIONE EPIPOLARE ---------------------
    Eigen::Isometry3f X_est = estimate_transform(cam.cameraMatrix(), correspondences, reference_image_points, current_measurements);
    //print_comparison(X_est,X_gt1,"**********EPIPOLAR RESULTS**********");
    //std::cout << "[EP] translation: " << X_est.translation().transpose() << std::endl;

    // --------------------- TRIANGOLAZIONE ---------------------
    Vector3fVector world_points_est;
    IntPairVector correspondences_new;
    triangulate_points(cam.cameraMatrix(), X_est, correspondences, reference_image_points, current_measurements, world_points_est, correspondences_new);

    // --------------------- PICP ---------------------
    PICPSolver solver;
    solver.setKernelThreshold(10000);

    Vector3fVector points_in_cameraframe1;
    for (const auto& p : world_points_est)
        points_in_cameraframe1.push_back(X_est * p);

    cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());
    solver.init(cam, points_in_cameraframe1, current_measurements);

    for (int i = 0; i < 100; ++i)
        solver.oneRound(correspondences_new, false);

    cam = solver.camera();
    Eigen::Isometry3f cam_in_world = cam.worldInCameraPose().inverse();
    //print_comparison(cam.worldInCameraPose(),*(X_gt1.inverse()),"**********PICP RESULTS**********");
    
    //std::cout << "[IP] Translation (camera in world): " << cam_in_world.translation().transpose() << std::endl;

    // --------------------- READ GROUND TRUTH ---------------------
    std::string gt_file_path = "../data/trajectory.dat";
    Vector7fVector trajectory_data = read_trajectory_file(gt_file_path);

    Eigen::Vector3f gt_translation_curr = Eigen::Vector3f::Zero();
    Eigen::Vector3f gt_translation_new  = Eigen::Vector3f::Zero();

    for (const auto& data : trajectory_data) {
        int pose_id = static_cast<int>(data[0]);
        if (pose_id == id_file_curr) {
            gt_translation_curr << data[4], data[5], data[6];
        }
        if (pose_id == id_file_new) {
            gt_translation_new << data[4], data[5], data[6];
        }
    }

    // --------------------- ERROR ---------------------
    Eigen::Vector3f estimated_translation = cam_in_world.translation();
    Eigen::Vector3f translation_error = gt_translation_new - estimated_translation;

    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "[GT-" << id_file_curr << "]: " << gt_translation_curr.transpose() << std::endl;
    std::cout << "[GT-" << id_file_new  << "]: " << gt_translation_new.transpose()  << std::endl;
    std::cout << "[EST]: " << estimated_translation.transpose() << std::endl;
    std::cout << "Translation error: " << translation_error.transpose() << std::endl;

    return 0;
}