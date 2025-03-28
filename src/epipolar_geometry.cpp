#include "epipolar_geometry.h"

Eigen::Matrix3f normalize_measurement(const CameraParameters& camera, PointCloud& point_cloud) {
    
    float width = camera.width;
    float height = camera.height;

    Eigen::Matrix3f T;
    T << 2.0f / width, 0, -1.0f,  
         0, 2.0f / height, -1.0f, 
         0, 0, 1.0f;  

    for (auto& point : point_cloud.getPoints()) {
        const auto& image_point = point.image_point;

        float x = std::get<0>(image_point);
        float y = std::get<1>(image_point);

        Eigen::Vector3f image_point_homogeneous(x, y, 1.0f);
        Eigen::Vector3f normalized_point = T * image_point_homogeneous;

        point.normalized_image_point = std::make_tuple(normalized_point[0], normalized_point[1]);
    }

    return T;
}

Eigen::Matrix3f eight_point_algorithm(CameraParameters& camera_params, PointCloud& point_cloud_1, PointCloud& point_cloud_2, CorresponcesPairVector correspondence_vector){
    
    if(correspondence_vector.size()<8){
        std::cout << "Cannot apply 8-point algorithm: less then 8 points\n";
        exit(-1);
    }

    Eigen::Matrix3f T1 = normalize_measurement(camera_params, point_cloud_1);
    Eigen::Matrix3f T2 = normalize_measurement(camera_params, point_cloud_2);
    
    int M = correspondence_vector.size();
    Eigen::MatrixXf A(M,9);

    for( int i=0; i<M; i++){
        auto current_pair = correspondence_vector[i];
        int first_local_idx  = static_cast<int>(current_pair.first[0]);
        int second_local_idx = static_cast<int>(current_pair.second[0]);

        Point first_point = point_cloud_1.getPointWithId( first_local_idx );
        Point second_point = point_cloud_2.getPointWithId( second_local_idx );

        float xm = std::get<0>( first_point.normalized_image_point );
        float ym = std::get<1>( first_point.normalized_image_point );
        float xm_f = std::get<0>( second_point.normalized_image_point );
        float ym_f = std::get<1>( second_point.normalized_image_point );

        A.row(i) << 
            xm*xm_f, xm*ym_f, xm,
            ym*xm_f, ym*ym_f, ym,
            xm_f,    ym_f,    1;
    }

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A,Eigen::ComputeThinV);
    auto v=svd.matrixV().col(8);
    
    Eigen::Matrix3f Fa;
    Fa << v(0),v(1),v(2),
            v(3),v(4),v(5),
            v(6),v(7),v(8);

    const Eigen::JacobiSVD<Eigen::Matrix3f> svd_Fa(Fa,Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::DiagonalMatrix<float,3> Da;
    Da.diagonal() << svd_Fa.singularValues().head<2>(),0.f;

    // impose fundamental to have rk 2
    Eigen::Matrix3f F;
    F = svd_Fa.matrixU()*Da*svd_Fa.matrixV().transpose();

    // undo the effect of coordinates normalization
    return T1.transpose()*F*T2;

}

Eigen::Matrix3f compute_essential_matrix(CameraParameters& camera_params, Eigen::Matrix3f F){
    
    Eigen::Matrix3f K = camera_params.K;
    return K.transpose()*F*K;
}