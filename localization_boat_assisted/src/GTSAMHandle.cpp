//
// Created by alg on 12/06/20.
//

#include "localization_boat_assisted/GTSAMHandle.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>


void localization_boat_assisted::GTSAMHandle::addOdometryFactor() {
    /** As the BetweenFactor is added as one factor and not as two different factors
     * Hence IMU and linear velocity, hence we want fresh values for both
     * */
    if(ros_handle_.ros_data_.isNewLinearVelocity() && ros_handle_.ros_data_.isNewAngularVelocity()){
        calculateBetweenPose(ros_handle_.ros_data_.linear_velocity_, ros_handle_.ros_data_.angular_velocity_);

        gtsam_data_.factor_graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3> >(gtsam::Symbol('d', 0), gtsam::Symbol('d', 1),
                calculateBetweenPose(ros_handle_.ros_data_.linear_velocity_, ros_handle_.ros_data_.angular_velocity_),
                gtsam_data_.drone_odometry_noise_);

        ros_handle_.ros_data_.resetBools();
    }


}

gtsam::Pose3 localization_boat_assisted::GTSAMHandle::calculateBetweenPose(Eigen::Vector3d linear_velocity,
                                                                           Eigen::Vector3d angular_velocity) {
    double dt = 1/(ros_handle_.ros_rate_hz);
    return gtsam::Pose3(gtsam::Rot3::Rodrigues(angular_velocity[0]*dt, angular_velocity[1]*dt, angular_velocity[2]*dt),
            gtsam::Point3(linear_velocity[0]*dt, linear_velocity[1]*dt, linear_velocity[2]*dt));

}

gtsam::Pose3 localization_boat_assisted::GTSAMHandle::addConstantVelocityFactor(const gtsam::Pose3& constant_velocity = gtsam_data_.boat_constant_velocity_) {
    double dt = 1/(ros_handle_.ros_rate_hz);

    return gtsam::Pose3(gtsam::Rot3::Rodrigues(constant_velocity.rotation().roll()*dt,
            constant_velocity.rotation().pitch()*dt,
            constant_velocity.rotation().yaw()*dt),
                        gtsam::Point3( constant_velocity.x()*dt,
                                       constant_velocity.y()*dt,
                                       constant_velocity.z()*dt));
}

void localization_boat_assisted::GTSAMHandle::addMarkerFactor() {

}

