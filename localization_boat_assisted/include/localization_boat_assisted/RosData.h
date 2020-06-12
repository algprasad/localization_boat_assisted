//
// Created by alg on 11/06/20.
//

#ifndef SRC_ROSDATA_H
#define SRC_ROSDATA_H
//#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

namespace localization_boat_assisted{

struct RosData {
public:
    Eigen::Vector3d angular_velocity_;
    Eigen::Vector3d linear_velocity_;





    //
public:





};


}


#endif //SRC_ROSDATA_H
