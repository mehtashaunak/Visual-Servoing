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

#define VEL_TEST 1
#define POS_TEST 0


/*----Main Function------*/
int main (int argc, char** argv)
{
    ros::init(argc,argv,"velcoity_test_node");
    ros::NodeHandle nh;

    ros::Subscriber jointStateSub = nh.subscribe("/my_joint_states",10,jointStateCallbackGazebo);

    ros::Publisher JointVelPub = nh.advertise<ur5_vs::joint_vel>("/joint_vel_cmd",1);
    ros::Publisher JointAngPub = nh.advertise<ur5_vs::joint_angles>("/joint_angles_cmd",1);
    ros::Publisher SimVarPub = nh.advertise<ur5_vs::sim_variables>("/sim_variables_cmd",1);

    double initial_time = gazebo::common::Time::GetWallTime().Double();
    double duration = 0;
    int initialization=1;
    ros::Rate rate(100);



    while (ros::ok)
    {
#if(POS_TEST)
        //if (initialization)
        //{
        cout<<"inside initialization"<<endl;
        ur5_vs::joint_vel msg_vel;
        msg_vel.vel0.data = 0;
        msg_vel.vel1.data = 0;
        msg_vel.vel2.data = 0;
        msg_vel.vel3.data = 0;
        msg_vel.vel4.data = 0;
        msg_vel.vel5.data = 0;

        double set_angles_list[4] = {1, -1, 2, -1};
        ur5_vs::joint_angles msg_pos;
        msg_pos.ang0.data = (set_angles_list[0]);//set_angles_list[0]+new_array[0]);//0
        msg_pos.ang1.data = (set_angles_list[1]);//set_angles_list[1]+new_array[1]);//-1//set_angles_list[1]
        msg_pos.ang2.data = (set_angles_list[2]);//set_angles_list[2]+new_array[2]);//2
        msg_pos.ang3.data = (set_angles_list[3]);//set_angles_list[3]+new_array[3]);//-1
        msg_pos.ang4.data = 0;
        msg_pos.ang5.data = 0;
        cout<<"**************Printing angles**********"<<endl;
        cout << set_angles_list[0]  <<", "<< set_angles_list[1] <<", "<<set_angles_list[2] <<", "<<set_angles_list[3]<<", "<<endl;
        ur5_vs::sim_variables msg_simvar;
        msg_simvar.sim_enable.data = false;
        msg_simvar.reconfigure.data = true;

        JointVelPub.publish(msg_vel);
        JointAngPub.publish(msg_pos);
        SimVarPub.publish(msg_simvar);

        gazebo::common::Time::MSleep(3000);
        //getchar();
        initialization=0;
        //}
#endif
        //End-effector velocity
        Mat V_ee = cv::Mat(6, 1, CV_64F, 0.0);
        V_ee.at<double>(0,0)=0.0; //X
        V_ee.at<double>(1,0)=-0.05;//Y
        V_ee.at<double>(2,0)=0.0; //Z
        V_ee.at<double>(3,0)=0.0; //Rx
        V_ee.at<double>(4,0)=0.0; //Ry
        V_ee.at<double>(5,0)=0.0; //Rz

        //jacobian
        ros::spinOnce();
        double th[6]={jp1g, jp2g, jp3g, jp4g, jp5g, jp6g};
        //cout << "current joint angles " << jp1g << " " << jp2g << " " << jp3g << " " << jp4g << " " << jp5g << " " << jp6g << endl;
        //getchar();
        Mat J= cv::Mat(6, 6, CV_64F, 0.0);
        getRobotJacobian(th,J);
        //cout << "Jacobian " << J << endl;

        Mat Jinv= cv::Mat(6, 6, CV_64F, 0.0);
        cv::invert(J, Jinv, cv::DECOMP_SVD);

        //joint velocity
        Mat theta_dot= cv::Mat(6, 1, CV_64F, 0.0);
        theta_dot = Jinv * V_ee;
        //cout << endl << "Joint velocity " << theta_dot << endl;

#if(VEL_TEST)
        double joint_velocities[6];
        joint_velocities[0]=theta_dot.at<double>(0,0);
        joint_velocities[1]=theta_dot.at<double>(1,0);
        joint_velocities[2]=theta_dot.at<double>(2,0);
        joint_velocities[3]=theta_dot.at<double>(3,0);
        joint_velocities[4]=theta_dot.at<double>(4,0);
        joint_velocities[5]=theta_dot.at<double>(5,0);

        //fixed
        //        joint_velocities[0]=0.1;
        //        joint_velocities[1]=0;
        //        joint_velocities[2]=0;
        //        joint_velocities[3]=0;
        //        joint_velocities[4]=0;
        //        joint_velocities[5]=0;

        //sending velocity to gazebo controller
        double t1 =  gazebo::common::Time::GetWallTime().Double();
        ur5_vs::joint_vel msg_vel;
        msg_vel.vel0.data = joint_velocities[0];
        msg_vel.vel1.data = joint_velocities[1];
        msg_vel.vel2.data = joint_velocities[2];
        msg_vel.vel3.data = joint_velocities[3];
        msg_vel.vel4.data = joint_velocities[4];
        msg_vel.vel5.data = joint_velocities[5];

        ur5_vs::sim_variables msg_simvar;
        msg_simvar.sim_enable.data = true;
        msg_simvar.reconfigure.data = false;

        //std::cout << "----------START----------\n";
        SimVarPub.publish(msg_simvar);
        JointVelPub.publish(msg_vel);
        gazebo::common::Time::MSleep(100);
        msg_simvar.sim_enable.data = false;
        SimVarPub.publish(msg_simvar);
        //std::cout << "----------STOP----------\n";
        double t = (gazebo::common::Time::GetWallTime().Double() - t1);
        std::cout << "Joint Velocities Applied for : " <<  t << " seconds"<<std::endl;
        //        std::cout << "Applied values\n";
        //        std::cout << joint_velocities[0] << std::endl;
        //        std::cout << joint_velocities[1] << std::endl;
        //        std::cout << joint_velocities[2] << std::endl;
        //        std::cout << joint_velocities[3] << std::endl;
        //        std::cout << joint_velocities[4] << std::endl;
        //        std::cout << joint_velocities[5] << std::endl;
        //        std::cout << "-----------------------------\n";

#endif
        duration = gazebo::common::Time::GetWallTime().Double() - initial_time;
        cout<<"duration = "<<duration<<endl;

        if (duration>5)
            break;
    }

    return 0;
}


