#include "utils_file.h"
#include "utils_world.h"
#include <iostream>
#include <unordered_map>

int main(int argc, char** argv) {

    std::cout << "[INFO][i-j] eq - dff, pci.size - pcj.size\n";

    int total_eq = 0;
    int total_diff = 0;
    for (int i = 0; i < 120; ++i) {
        int j = i + 1;

        // --------------------- MEASUREMENTS ---------------------
        std::ostringstream oss_i, oss_j;
        oss_i << "../data/meas-" << std::setw(5) << std::setfill('0') << i << ".dat";
        oss_j << "../data/meas-" << std::setw(5) << std::setfill('0') << j << ".dat";

        std::string file_path_meas_i = oss_i.str();
        std::string file_path_meas_j = oss_j.str();
    
        PointCloud pc_i = read_meas_file(file_path_meas_i);
        PointCloud pc_j = read_meas_file(file_path_meas_j);

        // Estrazione punti immagine
        Vector2fVector p1_img = pc_i.extractImagePoints();
        Vector2fVector p2_img = pc_j.extractImagePoints();

        // --------------------- CORRESPONDENCE ---------------------
        IntPairVector index_pairs = perform_correspondences(file_path_meas_i, file_path_meas_j);
        //auto result = perform_correspondences(file_path_meas_i, file_path_meas_j);
        //CorresponcesPairVector point_pairs = result.first;
        //IntPairVector index_pairs = result.second;

        auto result_equal = counter_equal(pc_i, pc_j, index_pairs);
        int counter_equal = result_equal.first;
        int counter_different = result_equal.second;

        total_eq += counter_equal;
        total_diff += counter_different;

        //std::cout << "[INFO][" << i << "-" << j << "]  " << counter_equal 
        //      << "-" << counter_different << ", " << pc_i.size() << "-" << pc_j.size() << " -> correspondences: " << index_pairs.size() << std::endl;

    }

    std::cout << "[INFO][TOTAL] equal:" << total_eq << ", different: " << total_diff << std::endl;

}
