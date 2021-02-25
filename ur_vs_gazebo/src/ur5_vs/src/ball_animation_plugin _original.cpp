#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>

#define PI 3.14159265

namespace gazebo
{
class AnimatedBox : public ModelPlugin
{
public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
                    //name the animation "test",
                    //make it last 10 seconds,
                    // and set it on a repeat loop
                    new gazebo::common::PoseAnimation("test", 96.0, true));

        gazebo::common::PoseKeyFrame *key;
        key = anim->CreateKeyFrame(0.0);
       // 0.35 0.7 1.35
        key->Translation(ignition::math::Vector3d(-0.2, 1.4, 1.2));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set starting location of the box
        for(int i=0;i<=6280;i+=1){
            double x_new = 0.5+0.25*cos(i*PI/180);
            double z_new = 0.001*i;
            //key = anim->CreateKeyFrame(2*i/30);
            key = anim->CreateKeyFrame(i/30);
            key->Translation(ignition::math::Vector3d(-0.2, 1.4, 1.2));
            key->Rotation(ignition::math::Quaterniond(0.0, 0, z_new));
        }

        //set final location equal to starting location
        //key = anim->CreateKeyFrame(10);
        //key->Translation(ignition::math::Vector3d(3, 0.975, 1.2));
        //key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set the animation
        _parent->SetAnimation(anim);
    }

    // Pointer to the model
private: physics::ModelPtr model;

    // Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}

