#ifndef _ACTUATOR_PLUGIN_HH_
#define _ACTUATOR_PLUGIN_HH_

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <vector>
#include <string>
#include <ros/ros.h>
namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class ActuatorPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ActuatorPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      std::string joint_name = "";
      if ( _sdf->HasElement("motor") ) 
      {
        for (sdf::ElementPtr elem = _sdf->GetElement("motor"); elem != NULL;
           elem = elem->GetNextElement("motor"))
        {
          if (!elem->HasElement("joint"))
          {
          gzwarn << "Invalid SDF: got motor without joint."
                 << std::endl;
          continue;
          }
          std::string joint_name = elem->Get<std::string>("joint");
          if (elem->HasElement("joint_index"))
          {
            jointIndex = elem->Get<unsigned int>("joint_index");
          }
          else
          {
            jointIndex = 0;
          }
        }
      }
      else 
        {
            ROS_WARN_NAMED("ActuatorPlugin", "ActuatorPlugin missing <motor>!!!");
        }
      
      // Store the model pointer for convenience.
      this->model = _model;
      float maximumTorque = 10;
      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      // Store pointer to the joint we will actuate
      physics::JointPtr joints = _model->GetJoint(joint_name);// Store pointer to the joint we will actuate
      joints->SetEffortLimit(jointIndex, maximumTorque);
      this->joint = joints;
      
      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);
      
      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);

      // Set the joint's target velocity. This target velocity is just
      // for demonstration purposes.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), 10.0);
    
      // Default to zero velocity
      double velocity = 0;
      ROS_INFO_NAMED("actuator_plugin", "Getting velocity from sdf");
      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");
      
      // Set the joint's target velocity. This target velocity is just
      // for demonstration purposes.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), velocity);
      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif
      
      // Create a topic name
      std::string topicName = "/sg90_vel";
      
      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &ActuatorPlugin::OnMsg, this);
     
   }
   /// \brief Set the velocity of the Velodyne
/// \param[in] _vel New target velocity
   public: void SetVelocity(const double &_vel)
   {
     // Set the joint's target velocity.
     this->model->GetJointController()->SetVelocityTarget(
         this->joint->GetScopedName(), _vel);
   }
   /// \brief Handle incoming message
/// \param[in] _msg Repurpose a vector3 message. This function will
/// only use the x component.
   private: void OnMsg(ConstVector3dPtr &_msg)
   {
     this->SetVelocity(_msg->x());
   }
  /// \brief Pointer to the model.
  private: physics::ModelPtr model;
  
  /// \brief Pointer to the joint.
  private: physics::JointPtr joint;
  
  /// \brief A PID controller for the joint.
  private: common::PID pid; 
  /// \brief A node used for transport
  private: transport::NodePtr node;

  /// \brief A subscriber to a named topic.
  private: transport::SubscriberPtr sub;
  /// \brief The joints we want to actuate
  private: std::vector<physics::JointPtr> joints;
  /// \brief Which joint index is actuated by this actuator.
  public: int jointIndex;
  };

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(ActuatorPlugin)
}
#endif