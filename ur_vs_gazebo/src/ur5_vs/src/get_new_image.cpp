// image based servoing

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>

//for object detect
#include<new_detection/points_four_image_data.h>

//for sending velocity to robot
#include <control_speed/velocity_control.h>

//for moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf/transform_listener.h>

#include <my_funcs.h>
#include <vector>
#include <cmath>

using namespace Eigen;

using namespace std;
using namespace cv;

ros::Publisher pb;
ros::Subscriber sb;
trajectory_msgs::JointTrajectory jt;
double joint_pos_global;
double angle_degree;

const double lambda = 1;
sensor_msgs::ImageConstPtr imageIn;
cv::Mat image;


/*----------------------------------function declarations-------------------------------------- */

void ptCloudcallBack(const sensor_msgs::PointCloud2::ConstPtr &input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud, bool &flag)
{
    pcl::fromROSMsg( *input, *ptCloud );
    flag = true;
}

void getBaseFrameToKinect(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
    transform_listener.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform("/base_link", "/camera_link", ros::Time(0), transform);
    return;
}

void getEETocamera(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
    transform_listener.waitForTransform("/camera_link","/ee_link",  ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform( "/camera_link","/ee_link", ros::Time(0), transform);
    return;
}


void getworldTobase(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
    transform_listener.waitForTransform("/world", "/base", ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform("/world", "/base", ros::Time(0), transform);
    return;
}

void callback(const sensor_msgs::JointState & msg)
{
    joint_pos_global = (double)msg.velocity[4];
}

double x_point, y_point;
float centroid_depth;
double points_corner[8];


void img_proc_callback(const new_detection::points_four_image_data  msg)
{
    x_point = msg.detected_point_x.data;
    y_point = msg.detected_point_y.data;
    centroid_depth = msg.detected_point_depth.data;
    points_corner[0]=  (msg.detected_point_0.data-320)/(531.15/640);
    points_corner[1] = (msg.detected_point_1.data-240)/(531.15/480);
    points_corner[2] = (msg.detected_point_2.data-320)/(531.15/640);
    points_corner[3] = (msg.detected_point_3.data-240)/(531.15/480);
    points_corner[4] = (msg.detected_point_4.data-320)/(531.15/640);
    points_corner[5] = (msg.detected_point_5.data-240)/(531.15/480);
    points_corner[6] = (msg.detected_point_6.data-320)/(531.15/640);
    points_corner[7] = (msg.detected_point_7.data-240)/(531.15/480);
}

/*-------------------------------------------Main Function----------------------------------------------*/
//function here
void get_new_image (cv::Mat qnew)
{
    /*---------------------------------------Declarations-----------------------------------------------*/
//    ros::Subscriber img_proc = nh.subscribe< new_detection::points_four_image_data>("/object_points", 10, img_proc_callback);
//    sleep(1);
//    ros::spinOnce();

    vector<double> goal;

    goal.push_back(qnew.at<double>(0,0));
    goal.push_back(qnew.at<double>(1,0));
    goal.push_back(qnew.at<double>(2,0));
    goal.push_back(qnew.at<double>(3,0));
    goal.push_back(qnew.at<double>(4,0));
    goal.push_back(qnew.at<double>(5,0));

    arm_group.getJointValueTarget(goal);
    arm_group.move();

//    bool goal_achieved = false;

//    double time = ros::Time::now().toSec();
//    double duration;

//    while(!goal_achieved)
//    {
//        vector<double> current_jts = this->getCurrentJoints(arm_group);

//        double avg_pos_diff = 0;
////        cout << "Current: [";
//        for(int i = 0; i < 6; i++)
//        {
//            avg_pos_diff += fabs(current_jts[i] - desired_jts[i]);
////            cout << current_jts[i] << " "  << desired_jts[i] << " ";
//        }
////        cout << "]" << endl;
//        avg_pos_diff /= desired_jts.size();
////        cout << "Error: " << avg_pos_diff << endl;
//        // wait until avg joint position diff is approx. 0.57 degrees
//        if(avg_pos_diff < 0.01)
//            goal_achieved = true;

//        duration = ros::Time::now().toSec() - time;
//        if(duration > 5) // if in loop for more than 60 secs then break out
//        {
//            ROS_INFO("Time limit exceeded to reach desired position");
//            goal_achieved = true;
//            break;
//        }
//    }



//        cout<<"xpoint"<<x_point<<endl;
//        cout<<"ypoint"<<y_point<<endl;

//        UU.at<double>(0,0) = x_point; //Obtaining from img_proc_callback
//        UU.at<double>(1,0) = y_point; //Obtaining from img_proc_callback


//        for(int i = 0; i < 2; i++)
//        {
//            u_c[i]=UU.at<double>(i,0);
//        }

//        depth_p[0]=(double)centroid_depth;
//        depth_p[1]=(double)centroid_depth;

//        /*-------------------------------------------------------------*/

//        m[0]= (u_c[0]-320)/(531.15/640);    //(u-u0)/px
//        m[1]= (u_c[1]-240)/(531.15/480);    //(v-v0)/py
//        m_t[0]= (u_t[0]-320)/(531.15/640);    //(u-u0)/px
//        m_t[1]= (u_t[1]-240)/(531.15/480);    //(v-v0)/py

//    }


}

