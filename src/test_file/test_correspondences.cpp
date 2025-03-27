#include "utils_file.h"

using namespace std;
using TreeNodeType = TreeNode_<ContainerType::iterator>;


int main(int argc, char** argv) {
    string file_path_1 = "../data/meas-00006.dat";
    string file_path_2 = "../data/meas-00007.dat";

    auto result_vector = perform_correspondences(file_path_1, file_path_2);

}