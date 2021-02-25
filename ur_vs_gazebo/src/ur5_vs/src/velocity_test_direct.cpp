// velocity test

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <my_funcs.h>

using namespace std;
using namespace cv;

/*----Main Function------*/
int main (int argc, char** argv)
{
    ros::init(argc,argv,"velcoity_test_node");
    ros::NodeHandle nh;

    ros::Subscriber jointStateSub = nh.subscribe("/joint_states",10,jointStateCallback);

    ros::Publisher jointVelPub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 1);
    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint trajPoint;

    double total_time = ros::Time::now().toSec();
    double duration = 0;
    ros::Rate rate(100);
    while (ros::ok)
    {
        //End-effector velocity
        Mat V_ee = cv::Mat(6, 1, CV_64F, 0.0);
        V_ee.at<double>(0,0)=0.00;
        V_ee.at<double>(1,0)=0.00;
        V_ee.at<double>(2,0)=0.01;
        V_ee.at<double>(3,0)=0.00;
        V_ee.at<double>(4,0)=0.00;
        V_ee.at<double>(5,0)=0.00;

        //jacobian
        ros::spinOnce();
        double th[6]={jp1, jp2, jp3, jp4, jp5, jp6};
        //cout << "current joint angles " << jp1 << " " << jp2 << " " << jp3 << " " << jp4 << " " << jp5 << " " << jp6 << endl;
        Mat J= cv::Mat(6, 6, CV_64F, 0.0);
        newgetRobotJacobian(th,J);
        //cout << "Jacobian " << J << endl;

        Mat Jinv= cv::Mat(6, 6, CV_64F, 0.0);
        cv::invert(J, Jinv, cv::DECOMP_SVD);

        //joint velocity
        Mat theta_dot= cv::Mat(6, 1, CV_64F, 0.0);
        theta_dot = Jinv * V_ee;
        cout << endl << "Joint velocity " << theta_dot << endl;

        //sending velocity to controller
        traj.points.clear();
        trajPoint.velocities.clear();
        trajPoint.accelerations.clear();

        //applying velocity limit for safety
        double vel_limit = 1.0;
        double acc = 2;
        if (abs(theta_dot.at<double>(0,0))<vel_limit && abs(theta_dot.at<double>(1,0))<vel_limit && abs(theta_dot.at<double>(2,0))<vel_limit && abs(theta_dot.at<double>(3,0))<vel_limit && abs(theta_dot.at<double>(4,0))<vel_limit && abs(theta_dot.at<double>(5,0))<vel_limit )

        {
            trajPoint.velocities.push_back(theta_dot.at<double>(0));
            trajPoint.velocities.push_back(theta_dot.at<double>(1));
            trajPoint.velocities.push_back(theta_dot.at<double>(2));
            trajPoint.velocities.push_back(theta_dot.at<double>(3));
            trajPoint.velocities.push_back(theta_dot.at<double>(4));
            trajPoint.velocities.push_back(theta_dot.at<double>(5));

            //            trajPoint.velocities.push_back(0);
            //            trajPoint.velocities.push_back(0);
            //            trajPoint.velocities.push_back(0);
            //            trajPoint.velocities.push_back(0);
            //            trajPoint.velocities.push_back(0);
            //            trajPoint.velocities.push_back(0);


            trajPoint.accelerations.push_back(acc);
            trajPoint.accelerations.push_back(acc);
            trajPoint.accelerations.push_back(acc);
            trajPoint.accelerations.push_back(acc);
            trajPoint.accelerations.push_back(acc);
            trajPoint.accelerations.push_back(acc);

        }
        else
        {
            trajPoint.velocities.push_back(0);
            trajPoint.velocities.push_back(0);
            trajPoint.velocities.push_back(0);
            trajPoint.velocities.push_back(0);
            trajPoint.velocities.push_back(0);
            trajPoint.velocities.push_back(0);

            trajPoint.accelerations.push_back(0);
            trajPoint.accelerations.push_back(0);
            trajPoint.accelerations.push_back(0);
            trajPoint.accelerations.push_back(0);
            trajPoint.accelerations.push_back(0);
            trajPoint.accelerations.push_back(0);
        }

        traj.points.push_back(trajPoint);
        jointVelPub.publish(traj); //joint speed publisher

        ros::spinOnce();
        rate.sleep();

        duration = ros::Time::now().toSec() - total_time;
        cout << "duration " << duration << endl;

        //        if (duration > 10)
        //            break;

        //sleep(1);

    }

    return 0;
}


