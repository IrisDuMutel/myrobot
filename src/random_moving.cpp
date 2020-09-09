#include "ros/ros.h"
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <cstdlib>
//I'm going to implement user-specified twist values later.
int main (int argc, char **argv)
{
  ros::init(argc,argv,"robot_mover_mark_two");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
  ros::ServiceClient client2= n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  geometry_msgs::Pose start_pose;
  gazebo_msgs::GetModelState currPosel;
  start_pose.position.x = currPosel.response.pose.position.x+0.01;
  start_pose.position.y = currPosel.response.pose.position.x+0.01;
  start_pose.position.z = 1.0;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 0.0;
  
  geometry_msgs::Twist start_twist;
  start_twist.linear.x = 1.1;
  start_twist.linear.y = 0;
  start_twist.linear.z = 0;
  start_twist.angular.x = 0.0;
  start_twist.angular.y = 0.0;
  start_twist.angular.z = 0.0;
  //---------------------------------------------------------------------
  gazebo_msgs::SetModelState setmodelstate;
  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = "moving_cube"; 
  modelstate.reference_frame = "world";
  modelstate.twist = start_twist;
  modelstate.pose = start_pose;
  setmodelstate.request.model_state = modelstate;
  if (client.call(setmodelstate))
  {
    ROS_INFO("BRILLIANT!!!");
    ROS_INFO("%f, %f",modelstate.pose.position.x,modelstate.pose.position.y);
  }
  else
  {
    ROS_ERROR("Failed to call service ");
    return 1;
  }
  return 0;}