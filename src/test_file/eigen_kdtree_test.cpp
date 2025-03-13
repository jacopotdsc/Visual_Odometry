#include "kdtree/eigen_kdtree.h"
#include "PointCloud.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <filesystem>

using namespace std;


using ContainerType = std::vector<Vectorf<11>>;
using TreeNodeType = TreeNode_<ContainerType::iterator>;

int main(int argc, char** argv) {
    string file_path_1 = "../data/meas-00000.dat";
    string file_path_2 = "../data/meas-00001.dat";

    std::vector<Vectorf<11>> points1 = read_meas_file(file_path_1);
    std::vector<Vectorf<11>> points2 = read_meas_file(file_path_2);

    cout << "Loaded " << points1.size() << " points for cloud1." << endl;
    cout << "Loaded " << points2.size() << " points for cloud2." << endl;

    cout << "Primi 5 punti di points1:" << endl;
    for(size_t i = 0; i < 5 && i < points1.size(); ++i){
        //cout << "Punto " << i << ": " << points1[i].transpose() << endl;
    }

    cout << "\nPrimi 5 punti di points2:" << endl;
    for(size_t i = 0; i < 5 && i < points2.size(); ++i){
        //cout << "Punto " << i << ": " << points2[i].transpose() << endl;
    }
    TreeNodeType  kd_tree(points1.begin(), points1.end(), 10);

    float max_distance = 10.0f;
    ofstream output("output_pairs.txt", ios::out);
    if (!output.is_open()) {
        cerr << "Errore apertura file: " << endl;
        return -1;
    }

    for (const auto& query_point : points2) {
        TreeNode_<vector<Vectorf<11>>::iterator>::AnswerType neighbors;
        kd_tree.fullSearchCustom(neighbors, query_point, max_distance);

        if (!neighbors.empty()) {
            auto nearest_neighbor = *neighbors.front();
            output << nearest_neighbor[0] << " " << nearest_neighbor[1] << endl;
            output << query_point[0] << " " << query_point[1] << endl << endl;
        }
    }

    output.close();
    cout << "Matching complete. Results saved in 'output_pairs.txt'" << endl;

}