#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Core>
#include <string>
#include <vector>
#include <utility>
#include <regex>

#include "kdtree/eigen_kdtree.h"


template <int dim>
using Vectorf = Eigen::Matrix<float, dim, 1>;  // Defining a vector with a general length

// Definitions for kd-tree
using ContainerType = std::vector<Vectorf<11>>; //using ContainerType = std::vector<Vectorf<11>, Eigen::aligned_allocator<Vectorf<11>>>;
using TreeNodeType = TreeNode_<ContainerType::iterator>;

// Definition for correspondences
using PairType = std::vector<float>;
using CorresponcesPairVector = std::vector< std::pair< PairType, PairType > >;
