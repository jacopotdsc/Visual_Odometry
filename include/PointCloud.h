#pragma once
#include "defs.h"

// Structure which represented the single line of meas-xxxxx.dat file
struct Point {
    Vector10f appaerance; // APPEARANCE
    int local_point_id; // POINT_ID_CURRENT_MESUREMENT
    int actual_point_id; // ACTUAL_POINT_ID
    Eigen::Matrix<float,2,1> image_point; //IMAGE_POINT, position on the image
    Eigen::Matrix<float,2,1> normalized_image_point; // normalized IMAGE_POINT, used for 8-points algorithm
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
        
        std::vector<Point> getPoints() const {
            return point_cloud_vector;
        }

        /**
         * @param idx_to_search Correspond to POINT_ID_CURRENT_MESUREMENT
         * @return Point structure with idx_to_search line of the meas-xxxxx.dat file
         */
        Point getPointWithId(int idx_to_search ) const{
            for (const auto& point : point_cloud_vector) {

                if( point.local_point_id == idx_to_search){
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
                Vector11f combined;
                combined[0] = point.local_point_id;
                for (int i = 0; i < 10; ++i) {
                    combined[i + 1] = point.appaerance[i];
                }
                result.push_back(combined);
            }
            return result;
        }

        Vector10fVector extractAppearance() const {
            Vector10fVector result;
            for (const auto& point : point_cloud_vector) {
                result.push_back(point.appaerance);
            }
            return result;
        }

        Vector3fVector extractLocalIdAndImagePosition() {
            Vector3fVector result;
            for (const auto& point : point_cloud_vector) {
                float id = static_cast<float>(point.local_point_id);
                float x = point.image_point[0];
                float y = point.image_point[1];
                result.emplace_back(id, x, y);
            }
            return result;
        }

        Vector3fVector extractActualIdAndImagePosition() {
            Vector3fVector result;
            for (const auto& point : point_cloud_vector) {
                float id = static_cast<float>(point.actual_point_id);
                float x = point.image_point[0];
                float y = point.image_point[1];
                result.emplace_back(id, x, y);
                //result.push_back(id, point.image_point);
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
                //float x = point.image_point[0];
                //float y = point.image_point[1];
                //image_points.emplace_back(x, y);
                image_points.push_back(point.image_point);
            }
            return image_points;
        }

        size_t size() const {
            return point_cloud_vector.size();
        }

        void clear() {
            point_cloud_vector.clear();
        }

        PointCloud& operator=(const PointCloud& other) {
            if (this != &other) {
                this->point_cloud_vector = other.point_cloud_vector;
            }
            return *this;
        }

};
