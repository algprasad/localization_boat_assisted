#include <ros/ros.h>
#include "localization_boat_assisted/RosHandle.hpp"
#include "localization_boat_assisted/GTSAMData.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization_boat_assisted");
  ros::NodeHandle nodeHandle("~");

  localization_boat_assisted::RosHandle loc_drone_boat(nodeHandle);
  localization_boat_assisted::GTSAMData gtsam_data;
  while(ros::ok()){
      //call the algorithm here.. not exceeding one or two sentences
      ros::spinOnce();
  }

  return 0;
}
