#include "epipolar_geometry.h"

int triangulate_points(const Eigen::Matrix3f& k, const Eigen::Isometry3f& X,
                       const IntPairVector& correspondences,
                       const Vector2fVector& p1_img, const Vector2fVector& p2_img,
                       Vector3fVector& triangulated,
                       IntPairVector& correspondences_new) {
    
    // Projection matrix for first camera: P1 = K [I | 0]
    Eigen::Matrix<float, 3, 4> P1;
    P1.block<3,3>(0,0) = k;
    P1.col(3).setZero();

    // Projection matrix for second camera: P2 = K [R | t]
    Eigen::Matrix<float, 3, 4> P2;
    P2.block<3,3>(0,0) = k * X.linear();
    P2.col(3) = k * X.translation();

    // Convert to OpenCV matrices
    cv::Mat proj1(3, 4, CV_64F); 
    cv::Mat proj2(3, 4, CV_64F);
    for (int i = 0; i < 3; ++i){
        for (int j = 0; j < 4; ++j) {
            proj1.at<double>(i,j) = P1(i,j);
            proj2.at<double>(i,j) = P2(i,j);
        }
    }
    triangulated.clear();
    correspondences_new.clear();
    triangulated.reserve(correspondences.size());
    correspondences_new.reserve(correspondences.size());

    int n_success = 0;

    for (const IntPair& corr : correspondences) {
        int idx1 = corr.first;
        int idx2 = corr.second;

        const Eigen::Vector2f& point_ref = p1_img[idx1];
        const Eigen::Vector2f& point_curr = p2_img[idx2];

        // Preparing one point per image in cv version
        cv::Mat point_ref_cv(2, 1, CV_64F);
        cv::Mat point_curr_cv(2, 1, CV_64F);
        point_ref_cv.at<double>(0,0) = point_ref.x();
        point_ref_cv.at<double>(1,0) = point_ref.y();
        point_curr_cv.at<double>(0,0) = point_curr.x();
        point_curr_cv.at<double>(1,0) = point_curr.y();

        // Triangulate the point
        cv::Mat point4D(4, 1, CV_64F);
        cv::triangulatePoints(proj1, proj2, point_ref_cv, point_curr_cv, point4D);

        // Convert from homogeneous coordinates: rom cv_pt = [x,y,z,w] to 3d_pt = [x, y, z]/w, w omogeneous scale
        double w = point4D.at<double>(3,0);
        if (std::abs(w) > 1e-5) {
            Eigen::Vector3f p;
            p << point4D.at<double>(0,0) / w,
                 point4D.at<double>(1,0) / w,
                 point4D.at<double>(2,0) / w;
            triangulated.push_back(p);
            correspondences_new.emplace_back(idx2, n_success);
            n_success++;
        }
    }

    return n_success;
}


Vector2fVector normalize_measurement(const Camera& camera, const Vector2fVector& image_points,Eigen::Matrix3f& T){
    
    Vector2fVector ret(image_points.size());

    int width = camera.cols();
    int height = camera.rows();

    T << 2.0f / width, 0, -1.0f,  
         0, 2.0f / height, -1.0f, 
         0, 0, 1.0f;  

    for (size_t i = 0; i < image_points.size(); ++i) {
        Eigen::Vector3f image_point_homogeneous(image_points[i].x(), image_points[i].y(), 1.0f);
        Eigen::Vector3f normalized = T * image_point_homogeneous;
        ret[i] = normalized.head<2>();
    }

    return ret;
}

const Eigen::Matrix3f estimate_fundamental( const Camera& camera, const IntPairVector& correspondences, 
                                            const Vector2fVector& p1_img, const Vector2fVector& p2_img){
    if(correspondences.size()<8){
        std::cout << "[ERROR] Cannot apply 8-point algorithm, not enough point. Exit(-1). \n";
        exit(-1);
    }
    Eigen::Matrix3f T;

    //coordinate points normalization in [-1,1]
    Vector2fVector p1_img_norm = normalize_measurement(camera, p1_img,T);
    Vector2fVector p2_img_norm = normalize_measurement(camera, p2_img,T);

    const int N=correspondences.size();
    Eigen::MatrixXf A(N,9);

    for (int i=0; i<N; i++){

        const int idx_first=correspondences[i].first;
        const int idx_second=correspondences[i].second;

        Eigen::Vector3f d1;
        Eigen::Vector3f d2;

        d1 << p1_img_norm[idx_first],1;
        d2 << p2_img_norm[idx_second],1;

        const Eigen::Matrix3f m=d1*d2.transpose();
        A.row(i)<<m(0,0),m(0,1),m(0,2),m(1,0),m(1,1),m(1,2),m(2,0),m(2,1),m(2,2);
    }
    const Eigen::JacobiSVD<Eigen::MatrixXf> svd(A,Eigen::ComputeThinV);
    Vector9f v=svd.matrixV().col(8);
    Eigen::Matrix3f Fa;

    Fa <<   v(0),v(1),v(2),
            v(3),v(4),v(5),
            v(6),v(7),v(8);

    const Eigen::JacobiSVD<Eigen::Matrix3f> svd_Fa(Fa,Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::DiagonalMatrix<float,3> Da;
    Da.diagonal() << svd_Fa.singularValues().head<2>(),0.f;

    // Imposing rank 2
    Eigen::Matrix3f F;
    F=svd_Fa.matrixU()*Da*svd_Fa.matrixV().transpose();

    // Cancelling normalization
    return T.transpose()*F*T;
}

const IsometryPair decompose_matrix(const Eigen::Matrix3f& E){
    const Eigen::Matrix3f w((Eigen::Matrix3f() << 0.f, -1.f, 0.f,
                                                1.f, 0.f, 0.f,
                                                0.f, 0.f, 1.f).finished());
    //1st solution                                
    const Eigen::JacobiSVD<Eigen::Matrix3f> svd(E,Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f v=svd.matrixV();
    Eigen::Matrix3f u=svd.matrixU();
    Eigen::Matrix3f R1=v*w*u.transpose();
    if(R1.determinant()<0){
        const Eigen::JacobiSVD<Eigen::Matrix3f> svd(-E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        v=svd.matrixV();
        u=svd.matrixU();
        R1=v*w*u.transpose();
    }
    Eigen::Isometry3f X1=Eigen::Isometry3f::Identity();
    X1.linear()=R1;
    Eigen::Matrix3f t_skew=R1*E;
    X1.translation()=Eigen::Vector3f(t_skew(2,1),t_skew(0,2),t_skew(1,0));

    //2nd solution
    Eigen::Matrix3f R2=v*w.transpose()*u.transpose();
    Eigen::Isometry3f X2=Eigen::Isometry3f::Identity();
    X2.linear()=R2;
    t_skew=R2*E;
    X2.translation()=Eigen::Vector3f(t_skew(2,1),t_skew(0,2),t_skew(1,0));
    IsometryPair X12(X1,X2);
    return X12;
}

const Eigen::Isometry3f estimate_transform(const Camera& camera, const IntPairVector& correspondences, 
    const Vector2fVector& p1_img, const Vector2fVector& p2_img){

    Eigen::Matrix3f k = camera.cameraMatrix();
    Eigen::Matrix3f F=estimate_fundamental(camera, correspondences,p1_img,p2_img);
    Eigen::Matrix3f E=k.transpose()*F*k;    
    const IsometryPair X12=decompose_matrix(E);

    return std::get<0>(X12);
}