#include "utils_world.h"

void computeFakeCorrespondences(IntPairVector& correspondences,
    const Vector2fVector reference_image_points,
    const Vector2fVector current_image_points){
    correspondences.resize(current_image_points.size());
    int num_correspondences=0;
    assert(reference_image_points.size()==current_image_points.size());

    for (size_t i=0; i<reference_image_points.size(); i++){
        const Eigen::Vector2f& reference_point=reference_image_points[i];
        const Eigen::Vector2f& current_point=current_image_points[i];
        IntPair& correspondence=correspondences[num_correspondences];
        if (reference_point.x()<0 || current_point.x()<0)
            continue;
        correspondence.first=i;
        correspondence.second=i;
        num_correspondences++;
    }
}

void putPointsOnSegment3D(Vector3fVector& dest, 
                            const Eigen::Vector3f& start,
                            const Eigen::Vector3f& end,
                            float density){
    Eigen::Vector3f segment=end-start;
    size_t num_points=segment.norm()*density;
    if (num_points<1)
        num_points=1;
    segment *= (1.f/num_points);
    int k=dest.size();
    dest.resize(k+num_points);
    for (size_t i=0; i<num_points; i++, k++){
        dest[k]=start+segment*(float)i;
    }
}

void makeWorld(Vector3fVector& world_points,
                const Eigen::Vector3f& lower_left_bottom,
                const Eigen::Vector3f& upper_right_top,
                int num_segments,
                float density){
    for (int i=0; i<num_segments; i++){
        Eigen::Vector3f ranges=upper_right_top-lower_left_bottom;
        Eigen::Vector3f p0=
        lower_left_bottom+
        Eigen::Vector3f(ranges.x()*drand48(),
                        ranges.y()*drand48(),
                        ranges.z()*drand48());

        Eigen::Vector3f p1=
        lower_left_bottom+
        Eigen::Vector3f(ranges.x()*drand48(),
                        ranges.y()*drand48(),
                        ranges.z()*drand48());

        putPointsOnSegment3D(world_points, p0, p1, density);
    }
}

void generate_isometry3f(Eigen::Isometry3f& X){
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

std::pair<int, int> counter_equal(PointCloud pc1, PointCloud pc2,  IntPairVector point_pairs){
    int counter_equal = 0;
    int counter_different = 0;
    for (size_t i = 0; i < point_pairs.size(); i++) {
        const auto& tuple = point_pairs[i];
        const auto& idx_v1 = tuple.first;
        const auto& idx_v2 = tuple.second;

        auto v1 = pc1.getPointWithId(idx_v1).appaerance;
        auto v2 = pc2.getPointWithId(idx_v2).appaerance;
    
        bool equal = true;
        for (int j = 0; j < 10; ++j) { 
            if (std::abs(v1[j] - v2[j]) > 0) {
                equal = false;
                counter_different += 1;
                std::cout << "-------\ntuple " << i << " == "<< (equal ? "UGUALI" : "DIVERSI") << "\n\t[1] " << v1.transpose()
                          << "\n\t[2] " << v2.transpose()<< "\n";
                break;
            }
        }
        counter_equal += 1;

        //std::cout << "-------\ntuple " << i << " == "<< (equal ? "UGUALI" : "DIVERSI") << "\n\t[1] " << v1.transpose()
        //          << "\n\t[2] " << v2.transpose()<< "\n";
    }

    return std::make_pair(counter_equal, counter_different);
}