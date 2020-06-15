#include <ros/ros.h>
#include <localization_boat_assisted/GTSAMHandle.h>
#include "localization_boat_assisted/RosHandle.hpp"
#include "localization_boat_assisted/GTSAMData.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization_boat_assisted");
  ros::NodeHandle node_handle("~");


  localization_boat_assisted::RosHandle ros_handle(node_handle); //FIXME: Should not have to initialize the ros_handle object and then pass it to GTSAMHandle. Should be directly initialized in GTSAMHAndle
  localization_boat_assisted::GTSAMHandle gtsam_handle(ros_handle);

  while(ros::ok()){
      //making the graph
      gtsam_handle.buildFactorGraph();

      //optimizing
      gtsam_handle.getIncrementalEstimate(); //TODO: Change according to what is suggested in the config file. Incremental or batch

      ros::spinOnce();
  }

  return 0;
}
