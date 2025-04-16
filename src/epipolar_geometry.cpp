#include "epipolar_geometry.h"

////////////////////////////////////////////////////////
bool triangulate_point(const Eigen::Vector3f& d1,const Eigen::Vector3f& d2,const Eigen::Vector3f& p2,Eigen::Vector3f& p){
    Eigen::Matrix<float,3,2> D;
    D.col(0)=-d1;
    D.col(1)=d2;
    const Eigen::Vector2f ss=-(D.transpose() * D).ldlt().solve(D.transpose() * p2);
    
        
    if(ss(0)<0 || ss(1)<0)
        return false;

    const Eigen::Vector3f p1_triangulated=ss(0)*d1;
    const Eigen::Vector3f p2_triangulated=p2+ss(1)*d2;
    //std::cout << "error: " << (p1_triangulated-p2_triangulated).norm() << std::endl;
    p=0.5f*(p1_triangulated+p2_triangulated);
    return true;
}

int triangulate_points(const Eigen::Matrix3f& k, const Eigen::Isometry3f& X, const IntPairVector& correspondences,
                        const Vector2fVector& p1_img, const Vector2fVector& p2_img, Vector3fVector& triangulated){
    const Eigen::Isometry3f iX = X.inverse();
    const Eigen::Matrix3f iK = k.inverse();
    const Eigen::Matrix3f iRiK = iX.linear()*iK;
    const Eigen::Vector3f t= iX.translation();
    int n_success=0;
    triangulated.resize(correspondences.size());
    for (const IntPair& correspondence: correspondences){
        const int idx_first=correspondence.first;
        const int idx_second=correspondence.second;
        Eigen::Vector3f d1;
        d1 << p1_img[idx_first],1;
        d1 = iK*d1;
        Eigen::Vector3f d2;
        d2 << p2_img[idx_second],1;
        d2 = iRiK*d2;
        Eigen::Vector3f p;
        if(triangulate_point(d1,d2,t,p)){
            triangulated[n_success]=p;
            n_success++;
        }
    }
    triangulated.resize(n_success);
    return n_success;
}

int triangulate_points(const Eigen::Matrix3f& k, const Eigen::Isometry3f& X, const IntPairVector& correspondences,
                        const Vector2fVector& p1_img, const Vector2fVector& p2_img, Vector3fVector& triangulated, IntPairVector& correspondences_new){
    const Eigen::Isometry3f iX = X.inverse();
    const Eigen::Matrix3f iK = k.inverse();
    const Eigen::Matrix3f iRiK = iX.linear()*iK;
    const Eigen::Vector3f t= iX.translation();
    int n_success=0;
    triangulated.resize(correspondences.size());
    correspondences_new.resize(correspondences.size());
    for (const IntPair& correspondence: correspondences){
        const int idx_first=correspondence.first;
        const int idx_second=correspondence.second;
        Eigen::Vector3f d1;
        d1 << p1_img[idx_first],1;
        d1 = iK*d1;
        Eigen::Vector3f d2;
        d2 << p2_img[idx_second],1;
        d2 = iRiK*d2;
        Eigen::Vector3f p;
        if(triangulate_point(d1,d2,t,p)){
            correspondences_new[n_success]=IntPair(idx_second,n_success);
            triangulated[n_success]=p;
            n_success++;
        }
    } 
    triangulated.resize(n_success);
    correspondences_new.resize(n_success);
    return n_success;
}

Vector2fVector normalize(const Vector2fVector& p,Eigen::Matrix3f& T){
    Vector2fVector ret(p.size());
    float max_x=0.f,max_y=0.f;
    for(const auto& v : p){
        if(v.x()>max_x)
            max_x=v.x();
        if(v.y()>max_y)
            max_y=v.y();
    }

    for(size_t i=0;i<p.size();i++)
        ret[i] << p[i].x()/(max_x/2.f)-1.f,p[i].y()/(max_y/2.f)-1.f;

    T << 1.f/(max_x/2.f),0.f,-1.f,
        0.f,1.f/(max_y/2.f),-1.f,
        0.f,0.f,1.f;
    return ret;
}

const Eigen::Matrix3f estimate_fundamental(const IntPairVector& correspondences, 
    const Vector2fVector& p1_img, const Vector2fVector& p2_img){
    if(correspondences.size()<8){
        std::cout << "Less than 8 points available to compute the fundamental matrix, aborting . . .\n";
        exit(-1);
    }
    Eigen::Matrix3f T1,T2; //conditioning matrices
    //coordinate points normalization in [-1,1]
    Vector2fVector p1_img_norm=normalize(p1_img,T1);
    Vector2fVector p2_img_norm=normalize(p2_img,T2);

    const int N=correspondences.size();
    Eigen::MatrixXf A(N,9);

    for (int i=0;i<N;i++){
        const int idx_first=correspondences[i].first;
        const int idx_second=correspondences[i].second;
        Eigen::Vector3f d1;
        d1 << p1_img_norm[idx_first],1;
        Eigen::Vector3f d2;
        d2 << p2_img_norm[idx_second],1;
        const Eigen::Matrix3f m=d1*d2.transpose();
        A.row(i)<<m(0,0),m(0,1),m(0,2),m(1,0),m(1,1),m(1,2),m(2,0),m(2,1),m(2,2);
    }

    const Eigen::JacobiSVD<Eigen::MatrixXf> svd(A,Eigen::ComputeThinV);
    Vector9f v=svd.matrixV().col(8);
    Eigen::Matrix3f Fa;
    Fa << v(0),v(1),v(2),
    v(3),v(4),v(5),
    v(6),v(7),v(8);
    const Eigen::JacobiSVD<Eigen::Matrix3f> svd_Fa(Fa,Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::DiagonalMatrix<float,3> Da;
    Da.diagonal() << svd_Fa.singularValues().head<2>(),0.f;

    // impose fundamental to have rk 2
    Eigen::Matrix3f F;
    F=svd_Fa.matrixU()*Da*svd_Fa.matrixV().transpose();

    // undo the effect of coordinates normalization
    return T1.transpose()*F*T2;
}

const IsometryPair essential2transformPair(const Eigen::Matrix3f& E){
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

const Eigen::Isometry3f estimate_transform(const Eigen::Matrix3f k, const IntPairVector& correspondences, 
    const Vector2fVector& p1_img, const Vector2fVector& p2_img){

    Eigen::Matrix3f F=estimate_fundamental(correspondences,p1_img,p2_img);
    Eigen::Matrix3f E=k.transpose()*F*k;    
    const IsometryPair X12=essential2transformPair(E);

    int n_test=0,n_in_front=0;
    Eigen::Isometry3f X_best=Eigen::Isometry3f::Identity();
    Vector3fVector triang;

    Eigen::Isometry3f X_test=std::get<0>(X12);
    n_test=triangulate_points(k,X_test,correspondences,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }
    X_test.translation()=-X_test.translation().eval();
    n_test=triangulate_points(k,X_test,correspondences,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }

    X_test=std::get<1>(X12);
    n_test=triangulate_points(k,X_test,correspondences,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }
    X_test.translation()=-X_test.translation().eval();
    n_test=triangulate_points(k,X_test,correspondences,p1_img,p2_img,triang);
    if (n_test > n_in_front){
        n_in_front=n_test;
        X_best=X_test;
    }
    return X_best;
}

////////////////////////////////////////////////////////


Eigen::Matrix3f normalize_measurement(const Camera& camera, PointCloud& point_cloud) {
    
    float width = camera._cols;
    float height = camera._rows;

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

Eigen::Matrix3f eight_point_algorithm(Camera& camera_params, PointCloud& point_cloud_1, PointCloud& point_cloud_2, CorresponcesPairVector correspondence_vector){
    
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

        Eigen::Vector3f d1, d2;
        d1 << xm, ym, 1;
        d2 << xm_f, ym_f, 1;
        const Eigen::Matrix3f m=d1*d2.transpose();

        A.row(i)<<m(0,0),m(0,1),m(0,2),m(1,0),m(1,1),m(1,2),m(2,0),m(2,1),m(2,2);
        //A.row(i) << 
        //    xm*xm_f, xm*ym_f, xm,
        //    ym*xm_f, ym*ym_f, ym,
        //    xm_f,    ym_f,    1;
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

    // Imposing rank 2 to F matrix
    Eigen::Matrix3f F;
    F = svd_Fa.matrixU()*Da*svd_Fa.matrixV().transpose();

    // Going back to original coordinate
    return T1.transpose()*F*T2;

}

Eigen::Matrix3f compute_essential_matrix(Camera& camera_params, Eigen::Matrix3f F){

    Eigen::Matrix3f K = camera_params._camera_matrix;
    return K.transpose()*F*K;
}

std::tuple<Eigen::Matrix3f, Eigen::Matrix3f, Eigen::Vector3f> compute_rotation_translation(const Eigen::Matrix3f& E) {
    
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    Eigen::Matrix3f W;
    W << 0.f, -1.f, 0.f,
         1.f,  0.f, 0.f,
         0.f,  0.f, 1.f;

    Eigen::Matrix3f R1 = V * W * U.transpose();
    if(R1.determinant() < 0){ R1 = -R1; }

    Eigen::Matrix3f R2 = V * W.transpose() * U.transpose();
    
    Eigen::Vector3f t = U.col(2);

    return std::make_tuple(R1, R2, t);
}

/*
bool triangulate_point(const Eigen::Vector3f& d1, const Eigen::Vector3f& d2, 
    const Eigen::Vector3f& t, Eigen::Vector3f& p) {
    Eigen::Matrix<float, 3, 2> D;
    D.col(0) = -d1;
    D.col(1) = d2;

    Eigen::Vector2f ss = -(D.transpose() * D).ldlt().solve(D.transpose() * t);

    if (ss(0) < 0 || ss(1) < 0){
        std::cout << "[DEBUG] Scartato: ss = " << ss.transpose() << "\n";
        return false;
    }
    std::cout << "[DEBUG] Scartato: ss = " << ss.transpose() << "\n";
        
    Eigen::Vector3f p1_triangulated = ss(0) * d1;
    Eigen::Vector3f p2_triangulated = t + ss(1) * d2;

    p = 0.5f * (p1_triangulated + p2_triangulated); 
    return true;
}

int triangulate_points( const Eigen::Matrix3f& K, const Eigen::Isometry3f& X, 
                        const CorresponcesPairVector& correspondences, 
                        PointCloud& p1, 
                        PointCloud& p2, 
                        Vector3fVector& triangulated_points) {

    //Eigen::Vector3f t = X.inverse().translation();
    const Eigen::Isometry3f iX = X.inverse();
    const Eigen::Matrix3f iK = K.inverse();
    const Eigen::Matrix3f iRiK = iX.linear()*iK;
    const Eigen::Vector3f t= iX.translation();

    int n_success = 0;
    triangulated_points.clear(); triangulated_points.reserve(correspondences.size());

    for (const auto& correspondence : correspondences) {
        const int idx_first = correspondence.first[0];
        const int idx_second = correspondence.second[0];

        Eigen::Vector3f d1, d2;
        float x1 = std::get<0>(p1.getPointWithId(idx_first).normalized_image_point); 
        float y1 = std::get<1>(p1.getPointWithId(idx_first).normalized_image_point); 
        d1 << x1, y1, 1;
        d1 = iK*d1;

        float x2 = std::get<0>(p2.getPointWithId(idx_second).normalized_image_point); 
        float y2 = std::get<1>(p2.getPointWithId(idx_second).normalized_image_point); 
        d2 << x2, y2, 1;
        d2 = iRiK*d2;

        Eigen::Vector3f p;
        if (triangulate_point(d1, d2, t, p)) {
            std::cout << "[TRIANG] p = " << p.transpose() << "\n";
            triangulated_points.push_back(p);
            n_success++;
        }
    }

    triangulated_points.resize(n_success);
    return n_success;
}

// Funzione principale per triangolare i punti e scegliere la rotazione migliore
Eigen::Isometry3f estimate_transform(const Eigen::Matrix3f& K, 
                                    const CorresponcesPairVector& correspondences, 
                                    PointCloud& p1, 
                                    PointCloud& p2,
                                    const Eigen::Matrix3f& R1, const Eigen::Matrix3f& R2, const Eigen::Vector3f& t) {
    
    Eigen::Isometry3f X1 = Eigen::Isometry3f::Identity();
    X1.linear() = R1;
    X1.translation() = t;

    Eigen::Isometry3f X2 = Eigen::Isometry3f::Identity();
    X2.linear() = R2;
    X2.translation() = t;

    Vector3fVector triangulated_points_1, triangulated_points_2;
    int n_success_1 = triangulate_points(K, X1, correspondences, p1, p2, triangulated_points_1);
    int n_success_2 = triangulate_points(K, X2, correspondences, p1, p2, triangulated_points_2);

    if (n_success_1 > n_success_2) {
        std::cout << "Using rotation R1" << std::endl;
        return X1;  
    } else {
        std::cout << "Using rotation R2" << std::endl;
        return X2; 
    }
}
*/