// #ifndef GAZEBO_PLUGINS_MOTOR_HH_
// #define GAZEBO_PLUGINS_MOTOR_HH_
#include <ignition/transport/Node.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"


namespace gazebo
{

  class YourPlugin : public ModelPlugin{
    /// \brief Constructor
    public: YourPlugin();

    /// \brief Destructor
    public: virtual ~YourPlugin();
      /// Documentation inherited
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    /// \brief Callback on world update event.
    private: void OnUpdate();


    private: physics::ModelPtr model;


  };
}