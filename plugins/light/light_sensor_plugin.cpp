/*
 * Copyright 2013 Open Source Robotics Foundation
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
 @mainpage
   Desc: GazeboRosCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
*/
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "light_sensor_plugin.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLight)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosLight::GazeboRosLight():
_nh("light_sensor_plugin"),
  _fov(6),
  _range(10)
  {
  }


////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosLight::~GazeboRosLight()
{
  ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

void GazeboRosLight::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("camera", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;

  GazeboRosCameraUtils::Load(_parent, _sdf);
  this->rosnode_ = new ros::NodeHandle("light_sensor");
  // this->parentSensor = dynamic_pointer_cast<sensors::ContactSensor>(_parent);
  this->light_pub_ = this->rosnode_->advertise<sensor_msgs::Illuminance>(
    std::string("light_data"), 1);
  ROS_INFO_NAMED("light_sensor", "Starting light sensor plugin");
  ROS_INFO("Light sensor ready");
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosLight::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{   

  common::Time sensor_update_time = this->parentSensor_->LastMeasurementTime();

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
    {
      if (sensor_update_time < this->last_update_time_)
      {
          ROS_WARN_NAMED("camera", "Negative sensor update time difference detected.");
          this->last_update_time_ = sensor_update_time;
      }

      if (sensor_update_time - this->last_update_time_ >= this->update_period_)
      {
        static int seq=0;
        this->msg.header.stamp = ros::Time::now();
        this->msg.header.frame_id = "";
        this->msg.header.seq = seq;
        int startingPix = _width * ( (int)(_height/2) - (int)( _fov/2)) - (int)(_fov/2);
        double illum = 0;
        for (int i=0; i<_fov ; ++i)
        {
          int index = startingPix + i*_width;
          for (int j=0; j<_fov ; ++j)
            illum += _image[index+j];
        }
        this->msg.illuminance = illum/(_fov*_fov);
        this->msg.variance = 0.0;
        light_pub_.publish(msg);
        seq++;
        this->PutCameraData(_image, sensor_update_time);
        this->PublishCameraInfo(sensor_update_time);
        this->last_update_time_ = sensor_update_time;
      }
    }
  }
}

} 
