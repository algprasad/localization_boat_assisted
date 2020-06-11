#include <ros/ros.h>
#include "localization_boat_assisted/LocalizationUWDroneBoat.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization_boat_assisted");
  ros::NodeHandle nodeHandle("~");

  localization_boat_assisted::LocalizationUWDroneBoat rosPackageTemplate(nodeHandle);

  ros::spin();
  return 0;
}
