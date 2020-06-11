#include <ros/ros.h>
#include "localization_boat_assisted/LocalizationUWDroneBoat.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization_boat_assisted");
  ros::NodeHandle nodeHandle("~");

  localization_boat_assisted::LocalizationUWDroneBoat loc_drone_boat(nodeHandle);
  while(ros::ok()){
      //call the algorithm here.. not exceeding one or two sentences
      ros::spinOnce();
  }

  return 0;
}
