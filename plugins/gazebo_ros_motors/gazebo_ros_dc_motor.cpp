#include <algorithm>
#include <assert.h>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>

#include "gazebo_ros_dc_motor.h"

namespace gazebo {

// Constructor
GazeboRosMotor::GazeboRosMotor() {
}

// Destructor
GazeboRosMotor::~GazeboRosMotor() {
  FiniChild();
}

// Load the controller
void GazeboRosMotor::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf ) {
    this->parent = _parent;
    this->plugin_name_ = _sdf->GetAttribute("name")->GetAsString();

    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, plugin_name_ ) );
    gazebo_ros_->isInitialized();

    // start custom queue
    this->callback_queue_thread_ = std::thread ( std::bind ( &GazeboRosMotor::QueueThread, this ) );

    // global parameters
    gazebo_ros_->getParameter<std::string> ( command_topic_,  "command_topic",  "/motor/voltage_norm" );
    gazebo_ros_->getParameter<double> ( update_rate_, "update_rate", 100.0 );

    // motor model parameters
    gazebo_ros_->getParameter<double> ( motor_nominal_voltage_, "motor_nominal_voltage", 24.0 ); // Datasheet 24.0V
    ROS_INFO_NAMED(plugin_name_, "motor_nominal_voltage_ = %f", motor_nominal_voltage_);
    gazebo_ros_->getParameter<double> ( moment_of_inertia_, "moment_of_inertia", 0.001 ); // 0.001 kgm^2
    ROS_INFO_NAMED(plugin_name_, "moment_of_inertia_ = %f", moment_of_inertia_);
    gazebo_ros_->getParameter<double> ( damping_ratio_, "damping_ratio", 0.0001 ); // Nm/(rad/s)
    ROS_INFO_NAMED(plugin_name_, "damping_ratio_ = %f", damping_ratio_);
    gazebo_ros_->getParameter<double> ( electromotive_force_constant_, "electromotive_force_constant", 0.08 ); // Datasheet: 1.8Nm / 22A or 24V / 300 (rad/s)
    ROS_INFO_NAMED(plugin_name_, "electromotive_force_constant_ = %f", electromotive_force_constant_);
    gazebo_ros_->getParameter<double> ( electric_resistance_, "electric_resistance", 1.0 ); // 1 Ohm
    ROS_INFO_NAMED(plugin_name_, "electric_resistance_ = %f", electric_resistance_);
    gazebo_ros_->getParameter<double> ( electric_inductance_, "electric_inductance", 0.001 ); // 1 mH
    ROS_INFO_NAMED(plugin_name_, "electric_inductance_ = %f", electric_inductance_);

    // noise parameters
    gazebo_ros_->getParameter<double> ( velocity_noise_, "velocity_noise", 0.0 );

    // gearbox parameters
    gazebo_ros_->getParameter<double> ( gear_ratio_, "gear_ratio", 1.0 ); // Reduction!

    // encoder parameters
    gazebo_ros_->getParameterBoolean  ( publish_velocity_, "publish_velocity", true );
    gazebo_ros_->getParameterBoolean  ( publish_encoder_, "publish_encoder", false );
    gazebo_ros_->getParameterBoolean  ( publish_current_, "publish_current", true );
    gazebo_ros_->getParameter<int>    ( encoder_pulses_per_revolution_, "encoder_ppr", 4096 );

    gazebo_ros_->getParameter<std::string> ( velocity_topic_, "velocity_topic", "/motor/velocity" );
    gazebo_ros_->getParameter<std::string> ( encoder_topic_,  "encoder_topic",  "/motor/encoder"  );
    gazebo_ros_->getParameter<std::string> ( current_topic_,  "current_topic",  "/motor/current"  );
    gazebo_ros_->getParameter<std::string> ( supply_topic_,  "supply_topic",  "/motor/supply_voltage"  );
    
    // motor joint
    joint_ = gazebo_ros_->getJoint ( parent, "motor_shaft_joint", "shaft_joint" );

    // shaft link
    gazebo_ros_->getParameter<std::string> ( wrench_frame_,  "motor_wrench_frame", "wheel_link" );
    this->link_ = parent->GetLink(this->wrench_frame_);
    if (!this->link_) {
      ROS_FATAL_NAMED(plugin_name_, "link named: %s does not exist\n",this->wrench_frame_.c_str());
      return;
    }

    // joint state publisher
    gazebo_ros_->getParameterBoolean  ( publish_motor_joint_state_, "publish_motor_joint_state", false );
    if (this->publish_motor_joint_state_) {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>(joint_->GetName()+"/joint_state", 1000);
        ROS_INFO_NAMED(plugin_name_, "%s: Advertise joint_state", gazebo_ros_->info());
    }

    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1 / this->update_rate_; else this->update_period_ = 0.0;
    last_update_time_ = parent->GetWorld()->SimTime();

    // command subscriber
    ROS_INFO_NAMED(plugin_name_, "%s: Trying to subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());
    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32> (
        command_topic_,
        1,
        boost::bind(&GazeboRosMotor::cmdVelCallback, this, _1),
        ros::VoidPtr(),
        &queue_
    );
    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED(plugin_name_, "%s: Subscribed to %s", gazebo_ros_->info(), command_topic_.c_str());

    // supply voltage subscriber
    ROS_INFO_NAMED(plugin_name_, "%s: Trying to subscribe to %s", gazebo_ros_->info(), supply_topic_.c_str());
    ros::SubscribeOptions sov = ros::SubscribeOptions::create<std_msgs::Float32> (
        supply_topic_,
        1,
        boost::bind(&GazeboRosMotor::supplyVoltageCallBack, this, _1),
        ros::VoidPtr(),
        &queue_
    );
    supply_voltage_subscriber_ = gazebo_ros_->node()->subscribe(sov);
    ROS_INFO_NAMED(plugin_name_, "%s: Subscribed to %s", gazebo_ros_->info(), command_topic_.c_str());

    // encoder publishers
    if (this->publish_velocity_){
      velocity_publisher_ = gazebo_ros_->node()->advertise<std_msgs::Float32>(velocity_topic_, 1);
      ROS_INFO_NAMED(plugin_name_, "%s: Advertising motor shaft (before gearbox) velocity on %s ", gazebo_ros_->info(), velocity_topic_.c_str());
    }
    if (this->publish_encoder_){
      encoder_publisher_ = gazebo_ros_->node()->advertise<std_msgs::Int32>(encoder_topic_, 1);
      ROS_INFO_NAMED(plugin_name_, "%s: Advertising encoder counts on %s ", gazebo_ros_->info(), encoder_topic_.c_str());
    }
    if (this->publish_current_){
      current_publisher_ = gazebo_ros_->node()->advertise<std_msgs::Float32>(current_topic_, 1);
      ROS_INFO_NAMED(plugin_name_, "%s: Advertising actual motor current on %s ", gazebo_ros_->info(), current_topic_.c_str());
    }

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosMotor::UpdateChild, this ) );

    input_ = 0;
    encoder_counter_ = 0;
    internal_current_ = 0;
    internal_omega_ = 0;
    supply_voltage_ = motor_nominal_voltage_;


}

void GazeboRosMotor::Reset() {
  last_update_time_ = parent->GetWorld()->SimTime();
  input_ = 0;
  encoder_counter_ = 0;
  internal_current_ = 0;
  internal_omega_ = 0;
  supply_voltage_ = motor_nominal_voltage_;
}



void GazeboRosMotor::publishWheelJointState(double velocity, double effort) {
    if (this->publish_motor_joint_state_){
    ros::Time current_time = ros::Time::now();
    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( 1 );
    joint_state_.position.resize ( 1 );
    joint_state_.velocity.resize ( 1 );
    joint_state_.effort.resize ( 1 );
    physics::JointPtr joint = joint_;
    double position = joint->Position ( 0 );
    joint_state_.name[0] = joint->GetName();
    joint_state_.position[0] = position;
    joint_state_.velocity[0] = velocity;
    joint_state_.effort[0] = effort;
    joint_state_publisher_.publish ( joint_state_ );
    }
}

// Velocity publisher
void GazeboRosMotor::publishRotorVelocity(double m_vel){
  std_msgs::Float32 vel_msg;
  vel_msg.data = m_vel; // (rad/sec)
  if (this->publish_velocity_) velocity_publisher_.publish(vel_msg);
}

// Simple incremental encoder emulation
void GazeboRosMotor::publishEncoderCount(double m_vel, double dT){
  std_msgs::Int32 counter_msg;
  double rev_in_rad = m_vel * dT;
  encoder_counter_ += round( ((rev_in_rad)/2*M_PI) * encoder_pulses_per_revolution_ );
  counter_msg.data = encoder_counter_;
  if (this->publish_encoder_) encoder_publisher_.publish(counter_msg);
}

void GazeboRosMotor::publishMotorCurrent(){
  std_msgs::Float32 c_msg;
  c_msg.data = internal_current_; // (amps)
  if (this->publish_current_) current_publisher_.publish(c_msg);
}

// Motor Model update function
void GazeboRosMotor::motorModelUpdate(double dt, double output_shaft_omega, double actual_load_torque) {
    if (input_ > 1.0) {
        input_ = 1.0;
    } else if (input_ < -1.0) {
        input_ = -1.0;
    }
    double T = actual_load_torque / gear_ratio_; // external loading torque converted to internal side
    // double V = input_ * supply_voltage_; // power supply voltage * (command input for motor velocity)
    internal_omega_ = output_shaft_omega * gear_ratio_; // external shaft angular veloc. converted to internal side
    // DC motor exact solution for current and angular velocity (omega)
 

    // Our model with Euler integration
    const double Kc = 0.00086626;
    const double Kem = 0.0023;
    const double f = 0.00000010651;
    const double C0 = 0.000029658;
    const double J_ours = 0.000000023;
    const double tau = 1;
    const double Cr = 0;
    const double res = 3.1;
    double V = 3;
    ROS_INFO( "Value of V: %f ", V);
    // double i0 = internal_current_;
    double o0 = internal_omega_;
    double A = (-Kc*Kem/res - f)/J_ours;
    ROS_INFO( "Value of A: %f ", A);

    double B = (Kc*V/res -C0)/J_ours;
    ROS_INFO( "Value of B: %f ", B);

    double w_m = o0+0.001*(A*o0+B);
    ROS_INFO( "Value of omega: %f ", w_m);

    double C_m = w_m*(-Kc*Kem/res-f)+Kc*V/res;
    ignition::math::Vector3d applied_torque;
    ROS_INFO( "Value of dt: %f ", dt);
    // TODO: axis as param
    applied_torque.Z() = C_m * 1/tau; // motor torque T_ext = K * i * n_gear
    internal_omega_ = w_m;
    ROS_INFO( "Value of applied torque: %f ", applied_torque.Z());

    this->link_->AddRelativeTorque(applied_torque);

    ///////////////////////////////////////////////////7

}

// Plugin update function

// TODO: change ths update funtion to something that matches our model. 
void GazeboRosMotor::UpdateChild() {
    common::Time current_time = parent->GetWorld()->SimTime();     // Get current time
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double(); 
    double current_output_speed = joint_->GetVelocity( 0u );       // Get current velocity
    ignition::math::Vector3d current_torque = this->link_->RelativeTorque();
    double actual_load = current_torque.Z();                       // Load last torque value

    motorModelUpdate(seconds_since_last_update, current_output_speed, actual_load);// keep integrating and computing w_m and C_m

    if ( seconds_since_last_update > update_period_ ) {  // Enter every 1/update_rate_ seconds 
        publishWheelJointState( current_output_speed, current_torque.Z() );
        publishMotorCurrent();
        auto dist = std::bind(std::normal_distribution<double>{current_output_speed, velocity_noise_},
                              std::mt19937(std::random_device{}()));
        double current_noisy_output_speed = dist();
        publishRotorVelocity( current_noisy_output_speed );
        publishEncoderCount( current_noisy_output_speed , seconds_since_last_update );
        last_update_time_+= common::Time ( update_period_ );
    }

    // If there was a parameter update, notify server
    // paramServerUpdate();
}

// Finalize the controller
void GazeboRosMotor::FiniChild() {
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

// Callback from custom que
void GazeboRosMotor::cmdVelCallback ( const std_msgs::Float32::ConstPtr& cmd_msg ) {
    input_ = cmd_msg->data;
}

void GazeboRosMotor::supplyVoltageCallBack ( const std_msgs::Float32::ConstPtr& voltage ) {
    supply_voltage_ = voltage->data;
}

void GazeboRosMotor::QueueThread() {
    static const double timeout = 0.01;
    while ( gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosMotor )
// eof_ns
}
