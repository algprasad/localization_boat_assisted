//
// Created by alg on 11/06/20.
//

#ifndef SRC_ROSDATA_H
#define SRC_ROSDATA_H
//#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>

namespace localization_boat_assisted{
/**
 * Contains relevant data from ROS and makes sure the data is latest //TODO: call the resetBools() function at the appropriate location to make sure the data is latest.
 * **/

struct RosData {
public:
    Eigen::Vector3d angular_velocity_;
    Eigen::Vector3d linear_velocity_;
    bool new_angular_velocity_;
    bool new_linear_velocity_;

public:
    void setAngularVelocity(sensor_msgs::Imu imu_message){
        this->angular_velocity_ << imu_message.angular_velocity.x, imu_message.angular_velocity.y, imu_message.angular_velocity.z;
    }

    void setLinearVelocity(geometry_msgs::PoseStamped pose_message){ //TODO: Change the type of message depending on what type of odometry topic is used. maybe overload the function
        this->linear_velocity_ << pose_message.pose.position.x, pose_message.pose.position.y, pose_message.pose.position.z;

    }

    void resetBools(){
        this->new_angular_velocity_ = false;
        this->new_linear_velocity_ = false;
    }

    void setBoolNewAngularVelocity(bool is_new_angular_velocity){
        this->new_linear_velocity_ = is_new_angular_velocity;
    }

    void setBoolNewLinearVelocity(bool is_new_linear_velocity){
        this->new_linear_velocity_ = is_new_linear_velocity;
    }

    bool isNewLinearVelocity(){
        return this->new_linear_velocity_;
    }

    bool isNewAngularVelocity(){
        return this->new_angular_velocity_;
    }







    //
public:





};


}


#endif //SRC_ROSDATA_H
