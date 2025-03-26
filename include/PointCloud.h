#pragma once
#include <vector>
#include <tuple> 
#include "defs.h"
#include "utils_file.h"

struct Point {
    Vectorf<11> local_id_and_appaerance;
    int actual_point_id;
    std::tuple<float, float> image_point; 
};

class PointCloud {
    
    std::vector<Point> point_cloud_vector;

    public:
        PointCloud() = default;

        void addPoint(const Point& point) {
            point_cloud_vector.push_back(point);
        }

        const std::vector<Point>& getPoints() const {
            return point_cloud_vector;
        }

        size_t size() const {
            return point_cloud_vector.size();
        }
};
