#include "utils_evaluation.h"

void write_trajectory_on_file(Vector3fVector gt_points, Vector3fVector estimated_points){
    std::ofstream gt_file("gt_trajectory.csv");
    gt_file << "x,y\n";
    for (const auto& p : gt_points)
        gt_file << p.x() << "," << p.y() << "," << p.z() << "\n";
    gt_file.close();

    std::ofstream est_file("estimated_trajectory.csv");
    est_file << "x,y\n";
    for (const auto& p : estimated_points)
        est_file << p.x() << "," << p.y() << "," << p.z()  << "\n";
    est_file.close();

    std::cout << "Curve trajectories saved to gt_trajectory.csv and estimated_trajectory.csv\n";
}