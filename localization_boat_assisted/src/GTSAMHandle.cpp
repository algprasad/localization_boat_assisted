//
// Created by alg on 12/06/20.
//

#include "localization_boat_assisted/GTSAMHandle.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include "localization_boat_assisted/MarkerFactor.h"


void localization_boat_assisted::GTSAMHandle::addOdometryFactor() {
    /** As the BetweenFactor is added as one factor and not as two different factors
     * Hence IMU and linear velocity, hence we want fresh values for both
     * */
     /// This increases the pose index of drone
    if(ros_handle_.ros_data_.isNewLinearVelocity() && ros_handle_.ros_data_.isNewAngularVelocity()){
        calculateBetweenPose(ros_handle_.ros_data_.linear_velocity_, ros_handle_.ros_data_.angular_velocity_);

        gtsam_data_.factor_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(gtsam::Symbol('d', drone_pose_index_), gtsam::Symbol('d', drone_pose_index_+1),
                calculateBetweenPose(ros_handle_.ros_data_.linear_velocity_, ros_handle_.ros_data_.angular_velocity_),
                gtsam_data_.drone_odometry_noise_);

        drone_pose_index_++;

        ros_handle_.ros_data_.resetBools();
    }


}

gtsam::Pose3 localization_boat_assisted::GTSAMHandle::calculateBetweenPose(Eigen::Vector3d linear_velocity,
                                                                           Eigen::Vector3d angular_velocity) {
    /// This should increase the pose index of boat/marker
    double dt = 1/(ros_handle_.ros_rate_hz);
    return gtsam::Pose3(gtsam::Rot3::Rodrigues(angular_velocity[0]*dt, angular_velocity[1]*dt, angular_velocity[2]*dt),
            gtsam::Point3(linear_velocity[0]*dt, linear_velocity[1]*dt, linear_velocity[2]*dt));


}
gtsam::Pose3 localization_boat_assisted::GTSAMHandle::calculateBetweenMarkerPose(const gtsam::Pose3& constant_velocity) {
    double dt = 1/(ros_handle_.ros_rate_hz);
    return gtsam::Pose3(gtsam::Rot3::Rodrigues(constant_velocity.rotation().roll()*dt,
                                        constant_velocity.rotation().pitch()*dt,
                                        constant_velocity.rotation().yaw()*dt),
                 gtsam::Point3( constant_velocity.x()*dt,
                                constant_velocity.y()*dt,
                                constant_velocity.z()*dt));
}

void localization_boat_assisted::GTSAMHandle::addConstantVelocityFactor(const gtsam::Pose3& constant_velocity) {

    gtsam_data_.factor_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >
            (gtsam::Symbol('m', boat_pose_index_), gtsam::Symbol('m', boat_pose_index_+1),
            calculateBetweenMarkerPose(constant_velocity),
            gtsam_data_.boat_pose_noise_);
    boat_pose_index_++;
}

void localization_boat_assisted::GTSAMHandle::addConstantVelocityFactor() {
    gtsam::Pose3 constant_velocity = this->gtsam_data_.boat_constant_velocity_;
    gtsam_data_.factor_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >
            (gtsam::Symbol('m', boat_pose_index_), gtsam::Symbol('m', boat_pose_index_+1),
             calculateBetweenMarkerPose(constant_velocity),
             gtsam_data_.boat_pose_noise_);
    boat_pose_index_++;
}

void localization_boat_assisted::GTSAMHandle::addMarkerFactor() {
    if(ros_handle_.ros_data_.isNewImage() && !ros_handle_.ros_data_.aruco_marker_.all_marker_in_frame_.empty()){ // if its a new iamge and has a marker add a marker factor
        gtsam_data_.factor_graph_.emplace_shared<gtsam::MarkerFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Cal3_S2> >
        (ros_handle_.ros_data_.aruco_marker_.corner_pixel_measurements_[0],
         ros_handle_.ros_data_.aruco_marker_.corner_pixel_measurements_[1],
         ros_handle_.ros_data_.aruco_marker_.corner_pixel_measurements_[2],
         ros_handle_.ros_data_.aruco_marker_.corner_pixel_measurements_[3],
         gtsam_data_.marker_pixel_noise_,
         gtsam::Symbol('d', drone_pose_index_),
         gtsam::Symbol('m', boat_pose_index_),
         gtsam_data_.K_,
         ros_handle_.ros_data_.aruco_marker_.marker_size_,
         gtsam_data_.body_p_sensor_
         );
    }


}

void localization_boat_assisted::GTSAMHandle::getIncrementalEstimate() {
    ROS_INFO("Incremental pose estimate published");

}

void localization_boat_assisted::GTSAMHandle::buildFactorGraph() {
    addOdometryFactor();
    addConstantVelocityFactor();
    addMarkerFactor();
}

