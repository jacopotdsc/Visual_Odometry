//#include "kdtree/eigen_01_point_loading.h"
#include "kdtree/eigen_kdtree.h"
#include "utils_file.h"
#include <iostream>
#include <fstream>

using namespace std;
using TreeNodeType = TreeNode_<ContainerType::iterator>;

int main(int argc, char** argv) {
    string file_path_1 = "../data/meas-00000.dat";
    string file_path_2 = "../data/meas-00001.dat";

    std::vector<Vectorf<11>> points1 = read_meas_file(file_path_1);
    std::vector<Vectorf<11>> points2 = read_meas_file(file_path_2);

    cout << "Loaded " << points1.size() << " points for cloud1." << endl;
    cout << "Loaded " << points2.size() << " points for cloud2." << endl;

    TreeNodeType  kd_tree(points1.begin(), points1.end(), 10);

    ofstream output("output_pairs.txt", ios::out);
    if (!output.is_open()) {
        cerr << "Errore apertura file: " << endl;
        return -1;
    }

    
    for (const auto& query_point : points2) {
      Vectorf<11>* neighbors = nullptr;  // Dichiariamo un puntatore a un singolo oggetto
      kd_tree.fullSearchCustom(neighbors, query_point, 1.0f);
  
      // Controlliamo se neighbors non è nullptr
      if (neighbors != nullptr) {
          auto nearest_neighbor = *neighbors;  // Dereferenziamo il puntatore
          output << nearest_neighbor[0] << " " << nearest_neighbor[1] << endl; // Stampa del primo punto
          output << query_point[0] << " " << query_point[1] << endl << endl; // Stampa del punto query
      }
      else{
        cout << "nothing found" << endl;
      }
  }
  
    
    output.close();
    cout << "Matching complete. Results saved in 'output_pairs.txt'" << endl;

}