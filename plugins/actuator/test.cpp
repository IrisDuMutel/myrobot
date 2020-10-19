#include <functional>
#include <ros/ros.h>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <algorithm>
#include <assert.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{

  class YourPlugin : public ModelPlugin
  {
    public: YourPlugin() {}
  
    
  
     /// \brief Load function gets called by Gazebo when the plugin is inserted in simulation 
     /// via URDF/SDF
     /// \param[in] _model A pointer to the model that this plugin is attached to.
     /// \param[in] _sdf A pointer to the plugin's SDF element.
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Store the model ptr for convenience.
        this->model_ = _model;
  
        // Get the name of your actuator/sensor joint from the URDF/SDF
        // Assuming:
        // <gazebo reference = "SENSORLINKNAME" />
        //     <actuatorJoint>NAMEOFJOINT</actuatorJoint>
        // </gazebo>
        //
        std::string joint_name = "";
        if ( _sdf->HasElement("actuatorJoint") ) {
            joint_name = _sdf->Get<std::string>("actuatorJoint");
        } else {
            ROS_WARN_NAMED("your_plugin", "YourPlugin missing <actuatorJoint>!!!");
        }
  
        // Store the actuatorjoint pointer.
        this->joint_ = this->model_->GetJoint(joint_name);
  
        // OTHER BOILERPLATE STUFF
  
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin (
        boost::bind ( &YourPlugin::OnUpdate, this, _1 ) );
    }
  
    // SETUP GAZEBO/ROS MESSAGING
  
    /// \brief Function called at each simulation interation
    void OnUpdate(const common::UpdateInfo & _info)
    {
        common::Time current_time = this->model_->GetWorld()->GetSimTime();
        current_joint_angle = this->joint_->GetAngle(0).Radian() // Get joint angle
  
        // DO COOL STUFF / PUBLISH JOINT ANGLE
        // this->target_torque_ = CALL TO PID CONTROLLER HERE
  
        this->joint_->SetForce(1, this->target_torque_);
    }
  };
  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(YourPlugin)
}