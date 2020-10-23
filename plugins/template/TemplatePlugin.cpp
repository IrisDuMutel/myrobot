/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Dave Coleman
 *  \desc   Example ROS plugin for Gazebo
 */

#include <TemplatePlugin.h>
#include <ros/ros.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TemplatePlugin);

////////////////////////////////////////////////////////////////////////////////
// Constructor
TemplatePlugin::TemplatePlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
TemplatePlugin::~TemplatePlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void TemplatePlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    ROS_INFO("Template plugin is working FINEEEEE");
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "Template" ) );
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( robot_namespace_, "robotNamespace", "/" );
    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopicServo", "cmd_servo" );
    gazebo_ros_->getParameter<double> ( servo_torque, "servoTorque", 10 );
    gazebo_ros_->getParameter<double> ( servo_diameter, "diameter_servo", 0.004 );
    gazebo_ros_->getParameter<double> ( update_rate_, "updateRatesg90", 100.0 );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "servoodometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "servoBaseFrame", "base_footprint" );
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishServoTF", false );
    gazebo_ros_->getParameterBoolean ( publishOdomTF_, "publishservoOdomTF", true);
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishServoJointState", false );
    gazebo_ros_->getParameterBoolean ( legacy_mode_, "servolegacyMode", true );
    
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "servoodometrySource", odomOptions, WORLD );
    servo_joint_=gazebo_ros_->getJoint(model, "motor_joint", "joint");
    // servo_joint_->SetParam ( "fservomax", 0, servo_torque );
    
    ROS_INFO( "%s: Advertise command topic", command_topic_.c_str());
  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("template", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void TemplatePlugin::UpdateChild()
{
}

}