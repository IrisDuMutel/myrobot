/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 */

#ifndef GAZEBO_ROS_TEMPLATE_HH
#define GAZEBO_ROS_TEMPLATE_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{

   class TemplatePlugin : public ModelPlugin
   {
       enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };
      /// \brief Constructor
      public: TemplatePlugin();

      /// \brief Destructor
      public: virtual ~TemplatePlugin();

      /// \brief Load the controller
      public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      /// \brief Update the controller
      protected: virtual void UpdateChild();
      

      public: 
        double current_joint_angle;  

      private:
        
        double x_;
        double rot_;
        bool alive_;
        

        // Update Rate
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;
        
        OdomSource odom_source_;
        geometry_msgs::Pose2D pose_encoder_;
        common::Time last_odom_update_;
        event::ConnectionPtr update_connection_;    

        double servo_speed_;
	      double servo_accel;
        double servo_diameter;
        double servo_speed_instr_;
        double servo_torque;
        // ROS STUFF
        ros::Publisher odometry_publisher_;
        ros::Subscriber cmd_vel_subscriber_;
        boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
        sensor_msgs::JointState joint_state_;
        ros::Publisher joint_state_publisher_;
        nav_msgs::Odometry odom_;
        std::string tf_prefix_;
        std::string command_topic_;
        std::string robot_namespace_;
        std::string odometry_topic_;
        std::string odometry_frame_;
        std::string robot_base_frame_;
        GazeboRosPtr gazebo_ros_;
        ros::Publisher pub_;
        transport::SubscriberPtr sub;
        physics::JointPtr servo_joint_;
        physics::ModelPtr model;
        physics::ModelPtr parent;
         // Flags
        bool publishWheelTF_;
        bool publishOdomTF_;
        bool publishWheelJointState_;
        bool publish_tf_;
        bool legacy_mode_;

        boost::mutex lock;
        // Custom Callback Queue
        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;
        void QueueThread();
        // DiffDrive stuff
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
        void getServoVelocity();
        void UpdateOdometryEncoder();


   };

}

#endif