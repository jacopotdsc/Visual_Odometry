#include "utils_evaluation.h"
#include "defs.h"  

int main() {
    const int num_points = 100;
    Vector3fVector gt_points, estimated_points;

    std::default_random_engine gen;
    std::normal_distribution<float> noise(0.0f, 0.07f); 

    for (int i = 0; i < num_points; ++i){
        float t = i * 0.1f; 
        float x = std::sin(t);  
        float y = t;
        float z = std::cos(t);

        Eigen::Vector3f gt(x, y, z);
        Eigen::Vector3f est(x + noise(gen), y + noise(gen), z + noise(gen));

        gt_points.push_back(gt);
        estimated_points.push_back(est);
    }

    write_trajectory_on_file(gt_points, estimated_points);
}
