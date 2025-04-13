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
