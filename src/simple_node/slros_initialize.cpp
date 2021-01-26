#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "simple_node";

// For Block simple_node/Subscribe
SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_simple_node_nav_msgs_Odometry> Sub_simple_node_2;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

