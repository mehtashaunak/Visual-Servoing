// velocity and position test

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <my_funcs.h>
#include <gazebo/gazebo.hh>
#include <ur5_vs/joint_vel.h>
#include <ur5_vs/joint_angles.h>
#include <ur5_vs/sim_variables.h>

using namespace std;
using namespace cv;

#define VEL_TEST 0
#define POS_TEST 1


/*----Main Function------*/
int main (int argc, char** argv)
{
    ros::init(argc,argv,"reconfigure_robot_node");
    ros::NodeHandle nh;

    ros::Subscriber jointStateSub = nh.subscribe("/my_joint_states",10,jointStateCallbackGazebo);

    ros::Publisher JointVelPub = nh.advertise<ur5_vs::joint_vel>("/joint_vel_cmd",1);
    ros::Publisher JointAngPub = nh.advertise<ur5_vs::joint_angles>("/joint_angles_cmd",1);
    ros::Publisher SimVarPub = nh.advertise<ur5_vs::sim_variables>("/sim_variables_cmd",1);

    cout<<"Reconfigurung robot to initil position, press ctrl+z when done !!"<<endl;

    while (ros::ok)
    {
        ur5_vs::joint_vel msg_vel;
        msg_vel.vel0.data = 0;
        msg_vel.vel1.data = 0;
        msg_vel.vel2.data = 0;
        msg_vel.vel3.data = 0;
        msg_vel.vel4.data = 0;
        msg_vel.vel5.data = 0;

        double set_angles_list[6] = {0, -1, 2, -1, 0, 0};
        ur5_vs::joint_angles msg_pos;
        msg_pos.ang0.data = (set_angles_list[0]);
        msg_pos.ang1.data = (set_angles_list[1]);
        msg_pos.ang2.data = (set_angles_list[2]);
        msg_pos.ang3.data = (set_angles_list[3]);
        msg_pos.ang4.data = (set_angles_list[4]);
        msg_pos.ang5.data = (set_angles_list[5]);

        ur5_vs::sim_variables msg_simvar;
        msg_simvar.sim_enable.data = false;
        msg_simvar.reconfigure.data = true;

        JointVelPub.publish(msg_vel);
        JointAngPub.publish(msg_pos);
        SimVarPub.publish(msg_simvar);

        gazebo::common::Time::MSleep(3000);

        //break;
    }

    return 0;
}



