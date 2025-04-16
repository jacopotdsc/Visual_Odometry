#pragma once
#include "defs.h"
#include "utils_file.h"

// Structure which represented the single line of meas-xxxxx.dat file
struct Point {
    Vector11f local_id_and_appaerance; // vector composed by POINT_ID_CURRENT_MESUREMENT + APPEARANCE
    int actual_point_id; // ACTUAL_POINT_ID
    std::tuple<float, float> image_point; //IMAGE_POINT, position on the image
    std::tuple<float, float> normalized_image_point; // normalized IMAGE_POINT, used for 8-points algorithm
};

// Class which contain all data read from meas-xxxxx.dat file
class PointCloud {
    
    private:
        std::vector<Point> point_cloud_vector;  // Contain all points of the measurement

    public:
        PointCloud() = default;

        void addPoint(const Point& point) {
            point_cloud_vector.push_back(point);
        }
        
        std::vector<Point>& getPoints(){
            return point_cloud_vector;
        }

        /**
         * @param idx_to_search Correspond to POINT_ID_CURRENT_MESUREMENT
         * @return Point structure with idx_to_search line of the meas-xxxxx.dat file
         */
        Point getPointWithId(int idx_to_search ){
            for (const auto& point : point_cloud_vector) {

                int point_local_id = static_cast<int>( point.local_id_and_appaerance[0] );
                if( point_local_id == idx_to_search){
                    return point;
                }
            }
            std::cerr << "[PointCloud.h] Point with ID = " << idx_to_search << " not found" << std::endl;
            exit(-1);
        }

        /**
         * @brief Useful function to return a vector of local_id_and_appaerance
         *        to used kd-tree in an easier way
         * @return Vector11fVector of point containing local_id_and_appaerance
         */
        Vector11fVector extractLocalIdAndAppearance() const {
            Vector11fVector result;
            for (const auto& point : point_cloud_vector) {
                result.push_back(point.local_id_and_appaerance);
            }
            return result;
        }

        /**
         * @brief Useful function to return a vector image coordinate
         * @return Vector2fVector of point containing image coordinate
         */
        Vector2fVector extractImagePoints() {
            Vector2fVector image_points;
            for (const auto& point : point_cloud_vector) {
                float x = std::get<0>(point.image_point);
                float y = std::get<1>(point.image_point);
                image_points.emplace_back(x, y);
            }
            return image_points;
        }
        
        size_t size() const {
            return point_cloud_vector.size();
        }
};
