//#include "kdtree/eigen_01_point_loading.h"
#include "kdtree/eigen_kdtree.h"
#include "utils_file.h"
#include <iostream>
#include <fstream>

#include <vector>
#include <set>
#include <sstream>
#include <string>
#include <algorithm>

using namespace std;
using TreeNodeType = TreeNode_<ContainerType::iterator>;

bool all_same_values(const vector<float>& values) {
  return std::all_of(values.begin() + 1, values.end(), [&](float v) { return v == values[1]; });
}

// Function to check if two vectors are identical
bool are_identical(const vector<float>& v1, const vector<float>& v2) {
  return std::equal(v1.begin()+1, v1.end(), v2.begin()+1);
}

// Function to edit the file and remove invalid pairs
void edit_file(const string& input_file, const string& output_file) {
  ifstream input(input_file);
  ofstream output(output_file);

  if (!input.is_open()) {
      cerr << "Error opening file: " << input_file << endl;
      return;
  }

  if (!output.is_open()) {
      cerr << "Error opening output file: " << output_file << endl;
      return;
  }

  while (true) {
    // Read the first line
    string line1, line2, line_dummy;
    if (!getline(input, line1)) { break; }
    if (!getline(input, line2)) { break; }
    if (!getline(input, line_dummy)) {}

    stringstream ss1(line1), ss2(line2);
    vector<float> values1, values2;
    float value;

    while (ss1 >> value) { values1.push_back(value);}
    while (ss2 >> value) { values2.push_back(value); }

    cout << "values1: ";
    for (const float& v : values1) {
        cout << v << " ";  // Print each value followed by a space
    }
    cout << endl;

    cout << "values2: ";
    for (const float& v : values2) {
        cout << v << " ";  // Print each value followed by a space
    }
    cout << endl << endl;  // Add a blank line between the two vectors

    if (all_same_values(values1) || all_same_values(values2)) { continue; }

    if (are_identical(values1, values2)) {
        for (const float& v : values1) {
            output << v << " ";
        }
        output << endl;

        for (const float& v : values2) {
            output << v << " ";
        }
        output << endl << endl;

    }
  }

  input.close();
  output.close();
  cout << "File processed and saved as: " << output_file << endl;
}


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

    /*
    for (const auto& query_point : points2) {
      Vectorf<11> neighbors = Vectorf<11>::Constant(std::numeric_limits<float>::quiet_NaN());  // Dichiariamo un puntatore a un singolo oggetto
      kd_tree.fullSearchCustom(&neighbors, query_point, 1.0f);
  
      // Controlliamo se neighbors non è nullptr
      if (!neighbors.array().isNaN().any()) {
        for (int i = 0; i < 11; ++i) {
          output << neighbors[i] << " ";  // Print each value of the nearest neighbor
        }
        output << endl;  // Newline after the nearest neighbor's vector

        // Print all values of the query point
        for (int i = 0; i < 11; ++i) {
            output << query_point[i] << " ";  // Print each value of the query point
       }
       output << endl << endl;
      }
      else{
        cout << "nothing found" << endl;
      }
    }
    */
    for (const auto& query_point : points2) {
      // Using bestMatchFull to find the closest point
      Vectorf<11>* nearest_neighbor = kd_tree.fullSearchCustom_v2(query_point, 1.0f);
    
      if (nearest_neighbor != nullptr) {
          // Dereference the pointer and print the 11 values of the nearest neighbor
          for (int i = 0; i < 11; ++i) {
              output << (*nearest_neighbor)[i] << " ";  // Print each value of the nearest neighbor
          }
          output << endl;  // Newline after the nearest neighbor's vector
    
          // Print all values of the query point
          for (int i = 0; i < 11; ++i) {
              output << query_point[i] << " ";  // Print each value of the query point
          }
          output << endl << endl;  // Separate the query point from the nearest neighbor
      } else {
          cout << "Nothing found for this query." << endl;
      }
    }

    output.close();
    cout << "Matching complete. Results saved in 'output_pairs.txt'" << endl;

    string input_file = "output_pairs.txt";  // Replace with your actual input file path
    string output_file = "output_pairs_cleaned.txt";  // Replace with your desired output file path

    // Call the function to edit the file
    edit_file(input_file, output_file);
    
}