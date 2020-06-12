#pragma once

#include "localization_boat_assisted/Algorithm.hpp"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>

namespace localization_boat_assisted {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RosHandle
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  RosHandle(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RosHandle();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * IMU topic callback method.
   * @param for imu messages given by pixhawk.
   */
  void imuCallback(const sensor_msgs::Imu& message);

  /*!
   * IMU topic callback method.
   * @param odometry message given by optical flow
   */
  void odometryCallback(const sensor_msgs::Temperature& message);


  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber imu_subscriber_;
  ros::Subscriber odometry_subscriber_;

  //! ROS topic name to subscribe to.
  std::string imu_subscriber_topic_;
  std::string odometry_subscriber_topic_;

  //! ROS service server.
  ros::ServiceServer serviceServer_;

  //! Algorithm computation object.
  Algorithm algorithm_;
};

} /* namespace */
