#include "localization_boat_assisted/RosHandle.hpp"

// STD
#include <string>

namespace localization_boat_assisted {

RosHandle::RosHandle(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  imu_subscriber_ = nodeHandle_.subscribe(imu_subscriber_topic_, 1,
                                          &RosHandle::imuCallback, this);
  odometry_subscriber_ = nodeHandle_.subscribe(odometry_subscriber_topic_, 1, &RosHandle::odometryCallback, this );

  serviceServer_ = nodeHandle_.advertiseService("future_addition",
                                                &RosHandle::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

RosHandle::~RosHandle()
{
}

bool RosHandle::readParameters()
{
  if (!nodeHandle_.getParam("sub_imu_topic", imu_subscriber_topic_)) return false;
  if (!nodeHandle_.getParam("sub_odometry_topic", odometry_subscriber_topic_)) return false;

  return true;
}

void RosHandle::imuCallback(const sensor_msgs::Imu& message)
{
  //algorithm_.addData(message.angular_velocity);
}

void RosHandle::odometryCallback(const sensor_msgs::Temperature& message){
    //get odometry values from optical_flow

}

bool RosHandle::serviceCallback(std_srvs::Trigger::Request& request,
                                std_srvs::Trigger::Response& response)
{
  //response.success = true;
  //response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}

} /* namespace */