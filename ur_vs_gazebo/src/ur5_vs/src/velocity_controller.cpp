#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <string>
#include <ur5_vs/joint_vel.h>
#include <ur5_vs/sim_variables.h>
#include <ur5_vs/joint_angles.h>
//#include "joint_angles.pb.h"
//#include "joint_velocities.pb.h"
//#include "sim_variables.pb.h"

namespace gazebo
{
    typedef const boost::shared_ptr<const ur5_vs::joint_vel> JointVelocityPtr;
    typedef const boost::shared_ptr<const ur5_vs::joint_angles> JointAnglePtr;
    typedef const boost::shared_ptr<const ur5_vs::sim_variables> SimVariablePtr;

	class VelocityControllerPlugin : public ModelPlugin
	{
		public: VelocityControllerPlugin() {}
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			this->world = _model->GetWorld();
			this->world->SetGravity(ignition::math::Vector3d(0,0,0));

			std::cout << "Loading VelocityController Plugin \n";

			// Store the model pointer for convenience.
			this->model = _model;

			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&VelocityControllerPlugin::OnUpdate, this, _1));

			this->joints.push_back(_model->GetJoint("ur5::shoulder_pan_joint"));
			this->joints.push_back(_model->GetJoint("ur5::shoulder_lift_joint"));
			this->joints.push_back(_model->GetJoint("ur5::elbow_joint"));
			this->joints.push_back(_model->GetJoint("ur5::wrist_1_joint"));
			this->joints.push_back(_model->GetJoint("ur5::wrist_2_joint"));
			this->joints.push_back(_model->GetJoint("ur5::wrist_3_joint"));

			// Setup a P-controller, with a gain of 0.1.
			this->pid = common::PID(10, 0, 0);

			// Apply the P-controller to the joint.
			this->model->GetJointController()->SetVelocityPID(this->joints[0]->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joints[1]->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joints[2]->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joints[3]->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joints[4]->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joints[5]->GetScopedName(), this->pid);

			this->model->GetJointController()->SetVelocityTarget(this->joints[0]->GetScopedName(), 0);
			this->model->GetJointController()->SetVelocityTarget(this->joints[1]->GetScopedName(), 0);
			this->model->GetJointController()->SetVelocityTarget(this->joints[2]->GetScopedName(), 0);
			this->model->GetJointController()->SetVelocityTarget(this->joints[3]->GetScopedName(), 0);
			this->model->GetJointController()->SetVelocityTarget(this->joints[4]->GetScopedName(), 0);
			this->model->GetJointController()->SetVelocityTarget(this->joints[5]->GetScopedName(), 0);


			this->joints[0]->SetPosition(0,0);
	    	this->joints[1]->SetPosition(0,-1);
	    	this->joints[2]->SetPosition(0,2);
	    	this->joints[3]->SetPosition(0,-1);
	    	this->joints[4]->SetPosition(0,0);
	    	this->joints[5]->SetPosition(0,0);

			// Create the node
			this->node = transport::NodePtr(new transport::Node());
			this->node->Init("Model_Contoller_Plugin_Node");

			// Subscribe to the topic, and register a callback
			this->sub_joint_vel = this->node->Subscribe("/joint_vel_cmd",&VelocityControllerPlugin::jointVelocityCallback, this);
			this->sub_joint_pos = this->node->Subscribe("/joint_angles_cmd",&VelocityControllerPlugin::joinAngleCallback, this);
			this->sub_sim_variable = this->node->Subscribe("/sim_variable_cmd",&VelocityControllerPlugin::simVariableCallback, this);
            this->pub = this->node->Advertise<ur5_vs::joint_angles>("/joint_states");
    }

    public: void OnUpdate(const common::UpdateInfo & )
    {
        ur5_vs::joint_angles msg;
        msg.ang0.data = this->joints[0]->GetAngle(0).Radian();
        msg.ang0.data = this->joints[1]->GetAngle(0).Radian();
        msg.ang0.data = this->joints[2]->GetAngle(0).Radian();
        msg.ang0.data = this->joints[3]->GetAngle(0).Radian();
        msg.ang0.data = this->joints[4]->GetAngle(0).Radian();
        msg.ang0.data = this->joints[5]->GetAngle(0).Radian();
    	this->pub->Publish(msg);
    }

    private: void simVariableCallback(SimVariablePtr &msg)
    {
        this->world->EnablePhysicsEngine(msg->sim_enable.data);
    }

    private: void joinAngleCallback(JointAnglePtr &msg)
    {
    	
        this->joints[0]->SetPosition(0,msg->ang0.data);
        this->joints[1]->SetPosition(0,msg->ang1.data);
        this->joints[2]->SetPosition(0,msg->ang2.data);
        this->joints[3]->SetPosition(0,msg->ang3.data);
        this->joints[4]->SetPosition(0,msg->ang4.data);
        this->joints[5]->SetPosition(0,msg->ang5.data);
		
    }  

    private: void jointVelocityCallback(JointVelocityPtr &msg)
    {
    	/*
    	std::cout << "---------------------------\n";
    	std::cout << msg->vel1() << std::endl;
    	std::cout << msg->vel2() << std::endl;
    	std::cout << msg->vel3() << std::endl;
    	std::cout << msg->vel4() << std::endl;
    	std::cout << msg->vel5() << std::endl;
    	std::cout << msg->vel6() << std::endl;
    	std::cout << "---------------------------\n";
    	*/

        this->model->GetJointController()->SetVelocityTarget(this->joints[0]->GetScopedName(), msg->vel0.data);
        this->model->GetJointController()->SetVelocityTarget(this->joints[1]->GetScopedName(), msg->vel1.data);
        this->model->GetJointController()->SetVelocityTarget(this->joints[2]->GetScopedName(), msg->vel2.data);
        this->model->GetJointController()->SetVelocityTarget(this->joints[3]->GetScopedName(), msg->vel3.data);
        this->model->GetJointController()->SetVelocityTarget(this->joints[4]->GetScopedName(), msg->vel4.data);
        this->model->GetJointController()->SetVelocityTarget(this->joints[5]->GetScopedName(), msg->vel5.data);
    }

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: gazebo::physics::WorldPtr world;

    /// \brief A node used for transport
    private: transport::NodePtr node;

    private: transport::PublisherPtr pub;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub_joint_vel;
    private: transport::SubscriberPtr sub_joint_pos;
    private: transport::SubscriberPtr sub_sim_variable;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \Vector of joints
    private: std::vector<physics::JointPtr> joints;

    /// \brief A PID controller for the joint.
    private: common::PID pid;
	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(VelocityControllerPlugin)
}

