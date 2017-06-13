

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <iostream>
namespace gazebo
{
  class StereoCamera : public ModelPlugin
  {

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
            {
      	      // Store the pointer to the model
      	      this->model = _parent;

      	      // Listen to the update event. This event is broadcast every
      	      // simulation iteration.
      	      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      	          boost::bind(&StereoCamera::OnUpdate, this, _1));
     

            }
    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
    	  gazebo::sensors::SensorManager *mgr =
    	    gazebo::sensors::SensorManager::Instance();
      // Apply a small linear velocity to the model.
    	  gazebo::sensors::MultiCameraSensorPtr stereocam = static_pointer_cast
    	                                            < gazebo::sensors::MultiCameraSensor > (mgr->GetSensor("stereo_camera"));

    	  if (!stereocam) {
    	     std::cout << "ERROR: pointer stereocamera not found!"
    	          << std::endl;
    	        }
    	  std::cout<<"stereocamera pointer found succesfully!"<<std::endl;
    	  std::cout<<stereocam->CameraCount()<<std::endl;
    }

    // Pointer to the model
    private: physics::ModelPtr model;


    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

  };
  GZ_REGISTER_MODEL_PLUGIN(StereoCamera);
}

