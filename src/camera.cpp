#include "camera.h"

Camera::Camera(int rows,
                int cols,
                int z_near,
                int z_far,
                const Eigen::Matrix3f& camera_matrix,
                const Eigen::Isometry3f& world_in_camera_pose):
    _rows(rows),
    _cols(cols),
    _z_near(z_near),
    _z_far(z_far),
    _camera_matrix(camera_matrix),
    _world_in_camera_pose(world_in_camera_pose){}

int Camera::projectPoints(Vector2fVector& image_points,
                          const Vector3fVector& world_points,
                          bool keep_indices){
    image_points.resize(world_points.size());
    int num_image_points=0;
    const Eigen::Vector2f point_outside(-1,-1);
    int num_points_inside=0;
    for(size_t i=0; i<world_points.size(); i++){
      const Eigen::Vector3f world_point=world_points[i];
      Eigen::Vector2f& image_point=image_points[num_image_points];
      bool is_inside=projectPoint(image_point,world_point);
      if (is_inside)
        num_points_inside++;
      else
        image_point=point_outside;
      if (keep_indices||is_inside){
        num_image_points++;
      } 
    }
    image_points.resize(num_image_points);
    return num_points_inside;
}