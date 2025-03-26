#include "utils_file.h"

using namespace std;

int main(int argc, char** argv) {
    string file_path_1 = "../data/meas-00006.dat";
    string file_path_2 = "../data/meas-00007.dat";

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
      Vectorf<11>* nearest_neighbor = kd_tree.fullSearchCustom_v2(query_point, 1.0f);
    
      if (nearest_neighbor != nullptr) {
          for (int i = 0; i < 11; ++i) {
              output << (*nearest_neighbor)[i] << " "; 
          }
          output << endl;  
    
          for (int i = 0; i < 11; ++i) {
              output << query_point[i] << " ";  
          }
          output << endl << endl;  
      } else {
          cout << "Nothing found for this query." << endl;
      }
    }

    output.close();
    cout << "Matching complete. Results saved in 'output_pairs.txt'" << endl;

    string input_file = "output_pairs.txt";  
    string output_file = "output_pairs_cleaned.txt";  

    // Call the function to edit the file
    auto c_vector = compute_correspondences(input_file, output_file);
}