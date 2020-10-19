#include <functional>
#include <ros/ros.h>
#include <ignition/common/Profiler.hh>
#include "MotorPlugin.hh"

namespace gazebo
{
//////////////////////////////////////////////////
/// \brief Calculate torque due to the input conditions for an electric motor
/// model. Simplified from http://lancet.mit.edu/motors/motors3.html#power
/// \param[in] _speed Input velocity.
/// \param[in] _properties Static properties of this actuator
/// \return Torque according to the model.
float ElectricMotorModel(const float _speed, const float /*_torque*/,
  const MotorProperties &_properties)
{
  if (_speed > _properties.maximumVelocity)
    return _properties.power / _properties.maximumVelocity;

  float torque = _properties.power / _speed;

  if (torque > _properties.maximumTorque)
    return _properties.maximumTorque;

  return torque;
}

//////////////////////////////////////////////////
/// \brief A simple velocity limiting motor model. Returns the maximum torque
/// if speed is above the allowed limit, and returns the input torque otherwise.
/// \param[in] _speed Input velocity.
/// \param[in] _torque Input torque.
/// \param[in] _properties Static properties of this actuator
/// \return Torque according to the model.
float VelocityLimiterModel(const float _speed, const float _torque,
  const MotorProperties &_properties)
{
  if (_speed > _properties.maximumVelocity)
    return _properties.maximumTorque;

  return _torque;
}

//////////////////////////////////////////////////
/// \brief The null motor model. Nothing exciting happening here.
/// \return Torque according to the model, which will always be zero.
float NullModel(const float /*_speed*/, const float /*_torque*/,
                const MotorProperties &/*_properties*/)
{
  return 0;
}

enum {
    RIGHT,
    LEFT,
};
////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosMotorPlugin::GazeboRosMotorPlugin() {}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosMotorPlugin::~GazeboRosMotorPlugin() 
{
	FiniChild();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosMotorPlugin::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "MotorPlugin" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    
    // Read the SDF
  if (_sdf->HasElement("motor"))
  {
    for (sdf::ElementPtr elem = _sdf->GetElement("motor"); elem != NULL;
         elem = elem->GetNextElement("motor"))
    {
      // Get actuator properties
      MotorProperties properties;

      // actuator name is currently an optional property
      if (elem->HasElement("name"))
        properties.name = elem->Get<std::string>("name");

      if (!elem->HasElement("joint"))
      {
        gzwarn << "Invalid SDF: got actuator element without joint."
               << std::endl;
        continue;
      }
      std::string jointName = elem->Get<std::string>("joint");
      properties.modelFunction = NullModel;
      if (elem->HasElement("type"))
      {
        std::string modelType = elem->Get<std::string>("type");
        if (modelType.compare("electric_motor") == 0)
        {
          if (!elem->HasElement("power") ||
              !elem->HasElement("max_velocity") ||
              !elem->HasElement("max_torque"))
          {
            gzwarn << "Invalid SDF: Missing required elements for motor model "
                   << modelType << "." << std::endl;
            continue;
          }

          properties.power = elem->Get<float>("power");
          properties.maximumVelocity = elem->Get<float>("max_velocity");
          properties.maximumTorque = elem->Get<float>("max_torque");

          properties.modelFunction = ElectricMotorModel;
        }
        else if (modelType.compare("velocity_limiter") == 0)
        {
          if (!elem->HasElement("max_velocity") ||
              !elem->HasElement("max_torque"))
          {
            gzwarn << "Invalid SDF: Missing required elements for motor model "
                   << modelType << "." << std::endl;
            continue;
          }
          properties.maximumVelocity = elem->Get<float>("max_velocity");
          properties.maximumTorque = elem->Get<float>("max_torque");
          properties.modelFunction = VelocityLimiterModel;
        }
        else if (modelType.compare("null") != 0)
        {
          gzwarn << "Unknown motor model specified, selecting NullModel."
                 << std::endl;
        }
      }
      else
      {
        gzwarn << "No motor model specified, selecting NullModel."
               << std::endl;
      }
      if (elem->HasElement("index"))
      {
        properties.jointIndex = elem->Get<unsigned int>("index");
      }
      else
      {
        properties.jointIndex = 0;
      }
      if (elem->HasElement("commandTopic"))
      {
        std::string velTopic = elem->Get<std::string>("commandTopic");
      }
      else
      {
        gzwarn << "No velocity topic specified, selecting cmd_vel:("
               << velTopic << "cmd_vel" << std::endl;
      }
      if (elem->HasElement("odometryTopic"))
      {
        std::string odomTopic = elem->Get<std::string>("odometryTopic");
      }
      else
      {
        gzwarn << "No velocity topic specified, selecting odom:("
               << velTopic << "odometryTopic" << std::endl;
      }

      // Store pointer to the joint we will actuate
      physics::JointPtr joint = _parent->GetJoint(jointName);
      if (!joint)
      {
        gzwarn << "Invalid SDF: actuator joint " << jointName << " does not "
               << "exist!" << std::endl;
        continue;
      }
      joint->SetEffortLimit(properties.jointIndex, properties.maximumTorque);
      this->joints.push_back(joint);
      this->actuators.push_back(properties);
    }
    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO_NAMED("motor_plugin", "%s: Try to subscribe to %s", gazebo_ros_->info(), velTopic.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(velTopic, 1,
                boost::bind(&GazeboRosMotorPlugin::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("motor_plugin", "%s: Subscribe to %s", gazebo_ros_->info(), velTopic.c_str());

    if (this->publish_tf_)
    {
      odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odomTopic, 1);
      ROS_INFO_NAMED("diff_drive", "%s: Advertise odom on %s ", gazebo_ros_->info(), odomTopic.c_str());
    }

    // Set up a physics update callback
    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&MotorPlugin::WorldUpdateCallback, this)));
  }
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller

//////////////////////////////////////////////////
void GazeboRosMotorPlugin::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    x_ = cmd_msg->linear.x;
    rot_ = cmd_msg->angular.z;
}

void MotorPlugin::WorldUpdateCallback()
{
  IGN_PROFILE("MotorPlugin::WorldUpdateCallback");
  IGN_PROFILE_BEGIN("WorldUpdateCallback");
  // Update the stored joints according to the desired model.
  for (unsigned int i = 0; i < this->joints.size(); i++)
  {
    const int index = this->actuators[i].jointIndex;
    const float velocity = this->joints[i]->GetVelocity(index);
    float curForce = this->joints[i]->GetForce(index);
    float maxForce = this->actuators[i].modelFunction(velocity, curForce,
              this->actuators[i]);
    this->joints[i]->SetEffortLimit(index, maxForce);
  }
  IGN_PROFILE_END();
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface



void GazeboRosMotorPlugin::Reset()
{
#if GAZEBO_MAJOR_VERSION >= 8
  last_update_time_ = parent->GetWorld()->SimTime();
#else
  last_update_time_ = parent->GetWorld()->GetSimTime();
#endif
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  x_ = 0;
  rot_ = 0;
  joints_[LEFT]->SetParam ( "fmax", 0, wheel_torque );
  joints_[RIGHT]->SetParam ( "fmax", 0, wheel_torque );
}