#include "utils_evaluation.h"

#include <fstream>
#include <iostream>

void write_trajectory_on_file(Vector3fVector gt_points, Vector3fVector estimated_points) {
    std::ofstream gt_file("gt_trajectory.txt");
    std::ofstream estimated_file("estimated_trajectory.txt");

    if (!gt_file.is_open() || !estimated_file.is_open()) {
        std::cerr << "Error opening output files!" << std::endl;
        return;
    }

    for (const auto& pt : gt_points) {
        gt_file << pt.x() << " " << pt.y() << " " << pt.z() << std::endl;
    }

    for (const auto& pt : estimated_points) {
        estimated_file << pt.x() << " " << pt.y() << " " << pt.z() << std::endl;
    }

    gt_file.close();
    estimated_file.close();
}
