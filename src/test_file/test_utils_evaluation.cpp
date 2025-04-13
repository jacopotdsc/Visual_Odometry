#include "utils_evaluation.h"
#include "defs.h"  

int main() {
    const int num_points = 100;
    Vector2fVector gt_points, estimated_points;

    std::default_random_engine gen;
    std::normal_distribution<float> noise(0.0f, 0.07f); 

    for (int i = 0; i < num_points; ++i){
        float t = i * 0.1f; 
        float x = std::sin(t);  
        float y = t;

        Eigen::Vector2f gt(x, y);
        Eigen::Vector2f est(x + noise(gen), y + noise(gen));

        gt_points.push_back(gt);
        estimated_points.push_back(est);
    }

    write_trajecoty_on_file(gt_points, estimated_points);
}
