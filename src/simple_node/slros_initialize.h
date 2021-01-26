#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block simple_node/Subscribe
extern SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_simple_node_nav_msgs_Odometry> Sub_simple_node_2;

void slros_node_init(int argc, char** argv);

#endif
