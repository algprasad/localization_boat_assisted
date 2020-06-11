#include "localization_boat_assisted/LocalizationUWDroneBoat.hpp"

// STD
#include <string>

namespace localization_boat_assisted {

LocalizationUWDroneBoat::LocalizationUWDroneBoat(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &LocalizationUWDroneBoat::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",
                                                &LocalizationUWDroneBoat::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

LocalizationUWDroneBoat::~LocalizationUWDroneBoat()
{
}

bool LocalizationUWDroneBoat::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void LocalizationUWDroneBoat::topicCallback(const sensor_msgs::Temperature& message)
{
  algorithm_.addData(message.temperature);
}

bool LocalizationUWDroneBoat::serviceCallback(std_srvs::Trigger::Request& request,
                                              std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}

} /* namespace */
