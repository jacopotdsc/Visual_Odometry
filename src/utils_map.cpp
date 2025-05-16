#include "utils_map.h"

void rel2Glob(CustomVector<Vector3fVector>& vector_world_rel, IsometryVector& est_pose_rel, CustomVector<Vector3fVector>& vector_world_glob){

    assert(vector_world_points.size() == est_pose_rel.size()-1);
    
    Vector3fVector current_world_glob;
    
    /*
    world_frame:             camera_frame:
     
         z                       
         ^                        
         |                        
         |______> y     x <_______
        /                        / |
       x                        z   y

    w_R_c: Ry(-90°)*Rz(90°) Euler angles
    */

    Eigen::Isometry3f rf_camera_rotation = Eigen::Isometry3f::Identity();
    rf_camera_rotation.linear() = Ry( 90 * 3.14159/180 ) * Rz( -90 * 3.14159/180 );

    
    for( size_t i=0; i < vector_world_rel.size(); i++){
        current_world_glob.clear();

        Vector3fVector current_world_cloud = vector_world_rel[i];
        Eigen::Isometry3f current_pose_rel = est_pose_rel[i];
        

        Eigen::Isometry3f pose_global = Eigen::Isometry3f::Identity();
        for(const auto& wp : current_world_cloud){
            pose_global = pose_global * current_pose_rel;
            current_world_glob.push_back( rf_camera_rotation * pose_global * wp );
        }
        vector_world_glob.push_back( current_world_glob );
        
    }
}