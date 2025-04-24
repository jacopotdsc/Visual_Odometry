#include "defs.h"
#include "utils_world.h"
#include "utils_file.h"
#include "epipolar_geometry.h"
#include "picp_solver.h"


void computeFakeCorrespondences_v2(IntPairVector& correspondences,
                                    const Vector2fVector reference_image_points,
                                    const Vector2fVector current_measurements){
    correspondences.clear();                                  
    correspondences.resize(current_measurements.size());
    int num_correspondences=0;
    assert(reference_image_points.size()==current_measurements.size());
    

    for (size_t i=0; i<reference_image_points.size(); i++){
        const Eigen::Vector2f& reference_point=reference_image_points[i];
        const Eigen::Vector2f& current_point=current_measurements[i];
        //IntPair& correspondence=correspondences[num_correspondences];
        if (reference_point.x()<0 || current_point.x()<0) //if one of them is the invalid point
            continue;
        correspondences.emplace_back(i, i);
        num_correspondences++;
    }
    correspondences.resize(num_correspondences);
}

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

void generate_isometry3f_v2(Eigen::Isometry3f& X){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-1.0f, 1.0f);

    Eigen::Vector3f a = Eigen::Vector3f::NullaryExpr(3,1,[&](){return dis(gen);});
    a.normalize();
    Eigen::AngleAxisf random_angle_axis(dis(gen),a);
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f(random_angle_axis);

    X.linear()=rotation_matrix;
    X.translation()=Eigen::Vector3f::NullaryExpr(3,1,[&](){return dis(gen);});
}

Vector3fVector generate_points3d(const int& num_points){
    //generating data in the same range of the ones provided
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(-10.f, 10.0f);
    Vector3fVector points; points.resize(num_points);
    for(int i=0;i<num_points;i++){
        Eigen::Vector3f p;
        p << dis(gen),dis(gen),dis(gen)*0.1f+1.0f;
        points[i]=p;
    }
    return points;
}

int main(int argc, char** argv) {


    //Camera cam = read_camera_file("../data/camera.dat");
    Eigen::Matrix3f k;
    k << 150.f,0.f,320.f,
        0.f,150.f,240.f,
        0.f,0.f,1.f;
    Camera cam(480,640,0,10,k);

    Eigen::Isometry3f X_gt1 = Eigen::Isometry3f::Identity();
    X_gt1.translation() = Eigen::Vector3f(0.0, 0.0, 0.0);
    //std::cout << "[GT] translation: " << X_gt1.translation().transpose() << std::endl;

    Vector3fVector world_points_gt=generate_points3d(1000);
  
    Vector2fVector reference_image_points;
    Vector2fVector current_measurements;
    cam.projectPoints(reference_image_points,world_points_gt,true);
    cam.setWorldInCameraPose(X_gt1);
    cam.projectPoints(current_measurements,world_points_gt,true);


    IntPairVector correspondences;
    computeFakeCorrespondences(correspondences, reference_image_points, current_measurements);

    const Eigen::Isometry3f X_est = estimate_transform(cam.cameraMatrix(), correspondences,reference_image_points,current_measurements);
    print_comparison(X_est,X_gt1,"**********EPIPOLAR RESULTS**********");
    //std::cout << "[EP] translation: " << X_est.translation().transpose() << std::endl;
    //std::cout << "[EP] rotation: \n" << X_est.linear().transpose() << std::endl;

    Vector3fVector world_points_est;
    IntPairVector correspondences_new;
    triangulate_points(cam.cameraMatrix(),X_est,correspondences,reference_image_points,
                        current_measurements,world_points_est,correspondences_new);

    Eigen::Isometry3f X_gt2;
    generate_isometry3f(X_gt2); //  pose of the world in camera 2

    cam.setWorldInCameraPose(X_gt2);
    cam.projectPoints(current_measurements,world_points_gt,true);
    
    PICPSolver solver;
    solver.setKernelThreshold(10000);

    Vector3fVector points_in_cameraframe1;
    for(const auto& p : world_points_est)
        points_in_cameraframe1.push_back(X_est*p);

    cam.setWorldInCameraPose(Eigen::Isometry3f::Identity());
    solver.init(cam,points_in_cameraframe1,current_measurements);
    for(int i=0;i<100;i++)
        solver.oneRound(correspondences_new,false);
    
    cam=solver.camera();// this should estimate the pose of the first camera in the frame of the second
    print_comparison(cam.worldInCameraPose(),X_gt2*(X_gt1.inverse()),"**********PICP RESULTS**********");
    //std::cout << "[IP] Translation: " << cam.worldInCameraPose().inverse().translation().transpose() << std::endl;
    //std::cout << "[IP] rotation: \n" << cam.worldInCameraPose().inverse().linear() << std::endl;

    //std::cout << (Eigen::Matrix3f::Identity()-(cam.worldInCameraPose().inverse()*(X_gt2*(X_gt1.inverse()))).linear()).trace() << std::endl;
}