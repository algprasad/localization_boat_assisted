//
// Created by alg on 12/06/20.
//

#ifndef SRC_GTSAMHANDLE_H
#define SRC_GTSAMHANDLE_H

/**
 * @description: Class with methods to build factor graph and optimize.
 * */
#include <gtsam/nonlinear/Values.h>
#include "RosHandle.hpp"
#include "GTSAMData.h"

namespace localization_boat_assisted{
class GTSAMHandle {
public:
        /** Initial and final estimates obtained from factor graph*/
        gtsam::Values initial_estimate_;
        gtsam::Values result_;

        /** ROSHandle object*/
        RosHandle ros_handle_;

        /** Keep track of poses of boat and drone*/ //making these non static
        int drone_pose_index_;
        int boat_pose_index_;




        /** GTSAMData*/ //making it non static
        GTSAMData gtsam_data_;


        /**
         * Constructor
         * @param ros_handle interfaces with ROS
         * **/
        GTSAMHandle(RosHandle ros_handle) : ros_handle_(ros_handle) {
            boat_pose_index_ = 0;
            drone_pose_index_ = 0;
        }

        /** Helper function to calculate between odometry poses*/
        gtsam::Pose3 calculateBetweenPose(Eigen::Vector3d linear_velocity, Eigen::Vector3d angular_velocity);

        /** Adds odometry factor between drone nodes */
        void addOdometryFactor();

        /** Adds IMU factor between boat/marker nodes */
        void addConstantVelocityFactor(const gtsam::Pose3& constant_velocity); //includes both linear and angular velocity
        void addConstantVelocityFactor(); //using default value from the config file

        gtsam::Pose3 calculateBetweenMarkerPose(const gtsam::Pose3& constant_velocity);

        /** Adds MarkerFactor between drone and boat/Marker */
        void addMarkerFactor();

        /** Adds initial pose estimates to boat/marker nodes */
        void addInitialValueMarker(int marker_pose_number, gtsam::Pose3 pose_value);

        /** Adds initial pose estimates to drone */
        void addInitialValueDrone(int drone_pose_number, gtsam::Pose3 pose_value);

        /** Gets batch result */
        void getBatchEstimate();

        /** Get incremental estimate */
        void getIncrementalEstimate();

        /** Build Factor graph by calling the right addFactor functions and adding initial values */
        void buildFactorGraph();


};


}


#endif //SRC_GTSAMHANDLE_H
