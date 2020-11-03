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
#include <algorithm>
#include <assert.h>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <template_plugin.h>
#include <ros/ros.h>

namespace gazebo
{


////////////////////////////////////////////////////////////////////////////////
// Constructor
TemplatePlugin::TemplatePlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
TemplatePlugin::~TemplatePlugin()
{
    FiniChild();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void TemplatePlugin::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "Template" ) );
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( robot_namespace_, "robotNamespace", "/" );
    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopicServo", "cmd_servo" );
    gazebo_ros_->getParameter<double> ( torque_, "servoTorque", 10 );
    gazebo_ros_->getParameter<double> ( diameter_, "diameter_servo", 0.004 );
    gazebo_ros_->getParameter<double> ( accel_, "servoAcceleration", 0);
    gazebo_ros_->getParameter<double> ( update_rate_, "updateRatesg90", 100.0 );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "servoodometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "servoodometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "servoBaseFrame", "base_footprint" );
    gazebo_ros_->getParameterBoolean ( publishTF_, "publishTF", false );
    gazebo_ros_->getParameterBoolean ( publishOdomTF_, "publishservoOdomTF", true);
    gazebo_ros_->getParameterBoolean ( publishJointState_, "publishJointState", false );
    gazebo_ros_->getParameterBoolean ( legacy_mode_, "servolegacyMode", true );
    gazebo_ros_->getParameterBoolean ( publishJointState_, "publishJointState", false );

    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );
    
    //TODO this is different from diff drive
    joint_=gazebo_ros_->getJoint(parent, "motor_joint", "joint");
    joint_->SetParam ( "fmax", 0, torque_ );
    
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
    ROS_FATAL_STREAM_NAMED("template", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
    }
    ROS_INFO( "Advertise command topic: %s ", command_topic_.c_str());

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = parent->GetWorld()->SimTime();
#else
    last_update_time_ = parent->GetWorld()->GetSimTime();
#endif
    // Initialize velocity stuff
    speed_ = 0;
    // Initialize velocity support stuff
    speed_instr_ = 0;
    // x_ = 0;
    rot_ = 0;
    alive_ = true;

    if (this->publishJointState_)
    {
      joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
      ROS_INFO("%s: Advertise joint_states", gazebo_ros_->info());
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO( "%s: Try to subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
              boost::bind(&TemplatePlugin::cmdVelCallback, this, _1), ros::VoidPtr(), &queue_);
    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO( "%s: Subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());


    if (this->publishTF_)
    {
      odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
      ROS_INFO_NAMED("template", "%s: Advertise odom on %s ", gazebo_ros_->info(), odometry_topic_.c_str());
    }

    ROS_INFO("Template plugin ready");

    // start custom queue for actuator plugin
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &TemplatePlugin::QueueThread, this ) );
     // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &TemplatePlugin::UpdateChild, this ) );

}


void TemplatePlugin::Reset()
{
  #if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = parent->GetWorld()->SimTime();
  #else
    last_update_time_ = parent->GetWorld()->GetSimTime();
  #endif
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  // x_ = 0;
  rot_ = 0;
  joint_->SetParam ( "fmax", 0, torque_ );
}


//not working for now
void TemplatePlugin::publishJointState()
{   
// To publish in desired topic the state of the joints
} 



void TemplatePlugin::publishTF()
{
    
    ros::Time current_time = ros::Time::now();
    std::string servo_frame = gazebo_ros_->resolveTF(joint_->GetChild()->GetName ());
    std::string servo_parent_frame = gazebo_ros_->resolveTF(joint_->GetParent()->GetName ());

#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d poseServo = joint_->GetChild()->RelativePose();
#else
        ignition::math::Pose3d poseServo = joint_->GetChild()->GetRelativePose().Ign();
#endif

    tf::Quaternion qt ( poseServo.Rot().X(), poseServo.Rot().Y(), poseServo.Rot().Z(), poseServo.Rot().W() );
    tf::Vector3 vt ( poseServo.Pos().X(), poseServo.Pos().Y(), poseServo.Pos().Z() );
    tf::Transform tfServo ( qt, vt );
    transform_broadcaster_->sendTransform (
        tf::StampedTransform ( tfServo, current_time, servo_parent_frame, servo_frame ) );
    
}




////////////////////////////////////////////////////////////////////////////////
// Update the controller
void TemplatePlugin::UpdateChild()
{
        
    if ( fabs(torque_ -joint_->GetParam ( "fmax", 0 )) > 1e-6 ) {
      joint_->SetParam ( "fmax", 0, torque_ );
    }
    if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();

    #if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = parent->GetWorld()->SimTime();
    #else
        common::Time current_time = parent->GetWorld()->GetSimTime();
    #endif    
        double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {
        if (this->publishTF_) publishOdometry ( seconds_since_last_update );
        if ( publishTF_ ) publishTF();
        if ( publishJointState_ ) publishJointState();
        // Update robot in case new velocities have been requested
        getVelocity();
        double current_speed;
        current_speed = joint_->GetVelocity ( 0 ) ;
        if ( accel_ == 0 ||
                ( fabs ( speed_ - current_speed ) < 0.01 )) {
            //if max_accel == 0, or target speed is reached
            joint_->SetParam ( "vel", 0, speed_);
        } else {
            if ( speed_>=current_speed)
                speed_instr_+=fmin ( speed_-current_speed,  accel_ * seconds_since_last_update );
            else
                speed_instr_+=fmax ( speed_-current_speed, -accel_ * seconds_since_last_update );
            joint_->SetParam ( "vel", 0, speed_instr_ );
    }
   last_update_time_+= common::Time ( update_period_ );
  }
}


// Finalize the controller
void TemplatePlugin::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}


void TemplatePlugin::getVelocity()
{
    boost::mutex::scoped_lock scoped_lock ( lock );

    double va = rot_;
    speed_ = va; 
}

/// \brief Callback funtion for ROS subscriber
void TemplatePlugin::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
  boost::mutex::scoped_lock scoped_lock ( lock );
  x_ = cmd_msg->linear.x;
  rot_ = cmd_msg->angular.z;
}


/// \brief ROS helper function that processes messages
void TemplatePlugin::QueueThread()
{
static const double timeout = 0.01;
  while ( alive_ && gazebo_ros_->node()->ok() ) {
      queue_.callAvailable ( ros::WallDuration ( timeout ) );
  }
}



void TemplatePlugin::UpdateOdometryEncoder()
{
    double vel = joint_->GetVelocity ( 0 );
    #if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = parent->GetWorld()->SimTime();
    #else
        common::Time current_time = parent->GetWorld()->GetSimTime();
    #endif
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double s = vel  * seconds_since_last_update;
    double sdiff;
    sdiff = s;
    

    double dx = 0;
    double dy = 0;
    double dtheta = sdiff;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double w = dtheta/seconds_since_last_update;
    double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = w;
    odom_.twist.twist.linear.x = dx/seconds_since_last_update;
    odom_.twist.twist.linear.y = dy/seconds_since_last_update;
}



//not working for now
void TemplatePlugin::publishOdometry ( double step_time )
{ 

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

    }
    if ( odom_source_ == WORLD ) {
        // getting data from gazebo world
    #if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent->WorldPose();
    #else
            ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
    #endif        
    qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
    vt = tf::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );
    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();
    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
    #if GAZEBO_MAJOR_VERSION >= 8
            linear = parent->WorldLinearVel();
            odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();
    #else
            linear = parent->GetWorldLinearVel().Ign();
            odom_.twist.twist.angular.z = parent->GetWorldAngularVel().Ign().Z();
    #endif
        linear = parent->WorldLinearVel();
        odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();


        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();
    }

    if (publishOdomTF_ == true){
        tf::Transform base_footprint_to_odom ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( base_footprint_to_odom, current_time,
                                   odom_frame, base_footprint_frame ) );
    }


    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish ( odom_ );
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TemplatePlugin);
}