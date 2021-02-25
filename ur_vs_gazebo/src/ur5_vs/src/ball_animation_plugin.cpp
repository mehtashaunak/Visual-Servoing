#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>

#include "ode_perfect_linear.hh"
#include "ode_perfect_angular.hh"
//#include "ode_perfect_velocity.hh"
#include "pid_link.hh"

#define PI 3.14159265

namespace gazebo
{
  //////////////////////////////////////////////////
  /// \brief Sets velocity on a link or joint
  class SetLinkVelocityPlugin : public ModelPlugin
  {
//    public: OdePerfectVelocityController odePerfectVelocity;
//    public: OdePerfectLinearVelController odePerfectLinear;
//    public: OdePerfectAngularVelController odePerfectAngular;
    public: PIDLinkVelocityController pidLinkController0;
    public: PIDLinkVelocityController pidLinkController1;
    public: PIDLinkVelocityController pidLinkController2;


    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        this->model = _model;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SetLinkVelocityPlugin::Update, this, std::placeholders::_1));
      }

    public: void Update(const common::UpdateInfo &_info)
      {
        if (update_num == 0)
        {
          // Link velocity instantaneously without applying forces

          model->GetLink("link_beer")->SetAngularVel({0, 0, 2});
	  model->GetLink("link_beer1")->SetAngularVel({0, 0, 2});

        }
        else if (update_num == 200)
        {
//          odePerfectVelocity.Stop();
//          odePerfectLinear.Stop();
 //         odePerfectAngular.Stop();
          pidLinkController0.Stop();
          pidLinkController1.Stop();
          pidLinkController2.Stop();
        }
        update_num++;
      }

    /// \brief a pointer to the model this plugin was loaded by
private: physics::ModelPtr model;
    /// \brief object for callback connection
private: event::ConnectionPtr updateConnection;
    /// \brief number of updates received
private: int update_num = 0;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SetLinkVelocityPlugin)
}
