// image based servoing

#include <ros/ros.h>
#include <ros/package.h>
//#include <tf/transform_broadcaster.h>
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/imgproc/imgproc.hpp"
//#include <cv_bridge/cv_bridge.h>
//#include <std_srvs/Empty.h>
//#include <cmath>
#include <iostream>
//#include <ctime>
//#include <vector>
//#include <cstdlib>
//#include <iterator>
//#include <fstream>
//#include <std_msgs/Empty.h>
//#include <signal.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <std_msgs/Bool.h>

//for object detect
//#include<new_detection/points_four_image_data.h>
#include<new_detection/sift_points.h>


//for velocity control
//#include <trajectory_msgs/JointTrajectory.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>
//#include <std_msgs/Float64.h>
//#include <std_msgs/Bool.h>
//#include <sensor_msgs/JointState.h>
#include <control_speed/velocity_control.h>

//for moveit
#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit_msgs/RobotState.h>
//#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/robot_trajectory/robot_trajectory.h>
//#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf/transform_listener.h>

//for geometric jacobian
#include <ur5_geo_jacobian.h>

//#include <std_msgs/String.h>
//#include <stdlib.h>
//#include <Eigen/Core>
//#include <Eigen/Sparse>
//#include <Eigen/Geometry>
//#include <Eigen/SVD>
//#include <cmath>
#include <my_funcs.h>
//#include <std_msgs/MultiArrayDimension.h>
//#include <std_msgs/MultiArrayLayout.h>
//#include <std_msgs/Float64MultiArray.h>

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
int dp=5;


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

float x_obj[2*dp], y_obj[2*dp], x_scene[2*dp], y_scene[2*dp];
int size_msg;


void img_proc_callback(const new_detection::sift_points::ConstPtr  &msg)
{
//   size_msg = msg.size_array.data;
   cout << "size= " << size_msg << endl;
    cout <<"callback"<< endl;

//for(int i=0; i< 10; i++)
//{
//    x_obj[i] = msg.x_obj.data.data();
////       y_obj[i] = msg.y_obj.data[i];
////       x_scene[i] = msg.x_scene.data[i];
////       y_scene[i] = msg.y_scene.data[i];
//       cout << "x_point" << x_obj<< endl;
//}
    int i=0;
    for(std::vector<double>::const_iterator it1 = msg->x_obj.data.begin(); it1!= msg->x_obj.data.end(); ++it1)
    {
        x_obj[i] = *it1;
        i++;
//          cout << "x1 = "<<x_obj[i] << endl;
    }
    for(std::vector<double>::const_iterator it2 = msg->y_obj.data.begin(); it2!= msg->y_obj.data.end(); ++it2)
    {
        y_obj[i] = *it2;
        i++;
//          cout << "y1 = "<<y_obj[i] << endl;
    }

    for(std::vector<double>::const_iterator it3 = msg->x_scene.data.begin(); it3!= msg->x_scene.data.end(); ++it3)
    {
        y_obj[i] = *it3;
        i++;
//          cout << "x2 = "<<x_scene[i] << endl;
    }

    for(std::vector<double>::const_iterator it4 = msg->y_scene.data.begin(); it4!= msg->y_scene.data.end(); ++it4)
    {
        y_obj[i] = *it4;
        i++;
//          cout << "y2 = "<<y_scene[i] << endl;
    }


}


/*-------------------------------------------Main Function----------------------------------------------*/
int main (int argc, char** argv)
{
    ros::init(argc,argv,"test_node");

    ros::NodeHandle nh, mh;
    cv::startWindowThread();

    moveit::planning_interface::MoveGroup arm_group("manipulator");
    arm_group.setPlanningTime(25);// set the maximum planning time

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    /*---------------------------------------Declarations-----------------------------------------------*/

    sb = mh.subscribe("joint_states",10,callback);

    pb= nh.advertise<control_speed::velocity_control>("/calc_velocity",10);

    cv::Mat Jm = cv::Mat(6, 6, CV_64F, 0.0);
    cv::Mat Jg = cv::Mat(6, 6, CV_64F, 0.0);
    cv::Mat Jginv = cv::Mat(6, 6, CV_64F, 0.0);
    cv::Mat mul_Jacob = cv::Mat(6, 6, CV_64F, 0.0);
    cv::Mat new_Jacob = cv::Mat(6, 6, CV_64F, 0.0);

    cv::Mat vector_ec = cv::Mat(1, 3, CV_64F, 0.0);

    cv::Mat Theta_dot = cv::Mat(6,1, CV_64F, 0.0);

    cv::Mat Error_p = cv::Mat(2*dp, 1, CV_64F, 0.0);
    cv::Mat deri_error = cv::Mat(2, 1, CV_64F, 0.0);
    cv::Mat deri_term = cv::Mat(2, 1, CV_64F, 0.0);

    cv::Mat pre_Error_p = cv::Mat(2, 1, CV_64F, 0.0);
    cv::Mat L = cv::Mat(2*dp, 6, CV_64F, 0.0);
    cv::Mat L_plane = cv::Mat(2, 2, CV_64F, 0.0);
    cv::Mat L_3 = cv::Mat(2, 3, CV_64F, 0.0);
    cv::Mat Linv = cv::Mat(6, 2*dp, CV_64F, 0.0);
    cv::Mat Linv_plane = cv::Mat(2, 2, CV_64F, 0.0);
    cv::Mat L_3inv = cv::Mat(3, 2, CV_64F, 0.0);
    cv::Mat Rot = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat New_Rot = cv::Mat(3, 3, CV_64F, 0.0);

    cv::Mat t_Rot = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat V = cv::Mat(6, 1, CV_64F, 0.0);
    cv::Mat V_temp = cv::Mat(2, 1, CV_64F, 0.0);
    cv::Mat pre_V = cv::Mat(2, 1, CV_64F, 0.0);
    cv::Mat V_3 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V1 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V2 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat VV = cv::Mat(6, 1, CV_64F, 0.0);

    cv::Mat V11 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V22 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat UU = cv::Mat(2, 1, CV_64F, 0.0);

    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;

    tf::StampedTransform transform;
    tf::Transform tf;
    tf::Vector3 tfVec;
    tf::Vector3 test_pose;
    tf::Matrix3x3 tfR;
    tf::Quaternion quat;

    ofstream f1("/home/ilab/plots/error.txt", std::ios_base::trunc);
    ofstream f2("/home/ilab/plots/cvel1.txt", std::ios_base::trunc);
    ofstream f3("/home/ilab/plots/cvel2.txt", std::ios_base::trunc);
    ofstream f4("/home/ilab/plots/cvel3.txt", std::ios_base::trunc);
    ofstream f5("/home/ilab/plots/cvel4.txt", std::ios_base::trunc);
    ofstream f6("/home/ilab/plots/cvel5.txt", std::ios_base::trunc);
    ofstream f7("/home/ilab/plots/cvel6.txt", std::ios_base::trunc);

    ofstream f8("/home/ilab/plots/joint_vel1.txt", std::ios_base::trunc);
    ofstream f9("/home/ilab/plots/joint_vel2.txt", std::ios_base::trunc);
    ofstream f10("/home/ilab/plots/joint_vel3.txt", std::ios_base::trunc);
    ofstream f11("/home/ilab/plots/joint_vel4.txt", std::ios_base::trunc);
    ofstream f12("/home/ilab/plots/joint_vel5.txt", std::ios_base::trunc);
    ofstream f13("/home/ilab/plots/joint_vel6.txt", std::ios_base::trunc);



    double theta[6];

    double u_c[20];
    double eta = 0.04; // 100 controller convergence parameter 0.03 15 5
//    double eta = 0.1;
    double depth_p[2]={0,0},prev_depth[2], u_t[20];
    double m[20];
    double m_t[20];

    /*----------------------------------------------getting image -----------------------------------------*/

    ros::Subscriber img_proc = nh.subscribe< new_detection::sift_points>("/object_points", 10, img_proc_callback);
    sleep(2);
    ros::spinOnce();

//cout << "x_obj" <<x_obj <<endl;
//cout <<"y_obj" << y_obj <<endl;
//cout << "x_scene " << x_scene << endl;
//cout << "y_scene" << y_scene << endl;

//    UU.at<double>(0,0) = x_point; //Obtaining from img_proc_callback
//    UU.at<double>(1,0) = y_point; //Obtaining from img_proc_callback

    /*---------------------------------------iteration starts here--------------------------------------*/

    while(ros::ok())
    {
        for(int i = 0; i < 10; i++)
        {

            u_c[2*i] = x_scene[i];
            u_c[(2*i)+1] = y_scene[i];

//            cout <<"captured point" << u_c[i] << endl;
        }


    //    depth_p[0]=(double)centroid_depth;
    //    depth_p[1]=(double)centroid_depth;

    //    for (int p=0;p<2;p++)
    //    {
    //        prev_depth[p] = depth_p[p];
    //    }

        /*-------------------Desired points---------------------------*/
        for(int i = 0; i < dp; i++)
        {
            \
            u_t[2*i] = x_obj[i];
            u_t[(2*i)+1] = y_obj[i];
        }
        /*-------------------------------------------------------------*/
    for(int i = 0; i < dp; i++)
    {
        m[2*i]= (u_c[2*i]-320)/(531.15/640);    //(u-u0)/px
        m[(2*i)+1]= (u_c[(2*i)+1]-240)/(531.15/480);    //(v-v0)/py
        m_t[2*i]= (u_t[2*i]-320)/(531.15/640);    //(u-u0)/px
        m_t[(2*i)+1]= (u_t[(2*i)+1]-240)/(531.15/480);    //(v-v0)/py
//        cout << "m" << m[i] << endl;
//        cout << "m_t" << m_t[i] << endl;
    }

        for(int i = 0; i < 2*dp; i++)
        {
            Error_p.at<double>(i,0) = m[i]-m_t[i];
//            cout << "error" << Error_p << endl;
        }
        pre_Error_p.at<double>(0,0)=Error_p.at<double>(0,0);
        pre_Error_p.at<double>(1,0)=Error_p.at<double>(1,0);



        /*------Computing image jacobian--------*/

                depth_p[0]=0.01;
                depth_p[1]=0.01;

        compute_image_jacobian_sift(m_t, depth_p, L);
        cout << "jacobian calculated" << endl;
//                compute_image_jacobian(m_t, depth_p, L);

//        L_plane.at<double>(0,0) = L.at<double>(0,0);
//        L_plane.at<double>(1,0) = L.at<double>(1,0);
//        L_plane.at<double>(0,1) = L.at<double>(0,1);
//        L_plane.at<double>(1,1) = L.at<double>(1,1);



        cv::invert(L, Linv, cv::DECOMP_SVD);
//        cv::invert(L_plane, Linv_plane, cv::DECOMP_SVD);


        /*--------Find camera velocity----------*/

//        deri_error = Error_p/1000 - pre_Error_p/1000;
//        cout << "deri_error =" <<deri_error.at<double>(0,0) <<endl;

//        cout << "crnt_error" << Error_p.at<double>(0,0) << endl;
//        cout << "pre_error" << pre_Error_p.at<double>(0,0) << endl;

//        deri_term = deri_error - pre_V;

//        cout << "deri_term =" <<deri_term.at<double>(0,0) <<endl;

//        pre_Error_p.at<double>(0,0)=Error_p.at<double>(0,0);
//        pre_Error_p.at<double>(1,0)=Error_p.at<double>(1,0);


//        V_temp = -eta * Linv_plane * Error_p;

//        cout << "V_temp =" <<V_temp.at<double>(0,0) <<endl;
        V = -eta * Linv * Error_p;

//        V = V_temp - deri_term;

//        cout << "V =" <<V.at<double>(0,0) <<endl;

//        pre_V.at<double>(0,0)=V.at<double>(0,0);
//        pre_V.at<double>(1,0)=V.at<double>(1,0);


//        V1.at<double>(0,0)=V.at<double>(0,0);
//        V1.at<double>(1,0)=V.at<double>(1,0);
//        V1.at<double>(2,0)=0;
//        V2.at<double>(0,0)=0;
//        V2.at<double>(1,0)=0;
//        V2.at<double>(2,0)=0;

        //                V1.at<double>(0,0)=0.01;
        //                V1.at<double>(1,0)=0.0;
        //                V1.at<double>(2,0)=0.0;
        //                V2.at<double>(0,0)=0.0;
        //                V2.at<double>(1,0)=0.0;
        //                V2.at<double>(2,0)=0.0;

                V1.at<double>(0,0)=V.at<double>(0,0);
                V1.at<double>(1,0)=V.at<double>(1,0);
                V1.at<double>(2,0)=V.at<double>(2,0);
                V2.at<double>(0,0)=V.at<double>(5,0);
                V2.at<double>(1,0)=V.at<double>(4,0);
                V2.at<double>(2,0)=V.at<double>(3,0);

        /*----------------------------------------------------------------------------*/
        getEETocamera(transform);

        /*----Finding rotation matrix (Camera to base frame)-------*/

        tfVec = transform.getOrigin();
        test_pose = tfVec;
        //      cout<<"vector from reference frame to to child frame: "<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
        tfR = transform.getBasis();
        //    cout<<"orientation of child frame w/rt reference frame: "<<endl;
        tfVec = tfR.getRow(0);
        Rot.at<double>(0,0)=tfVec.getX();  Rot.at<double>(0,1)=tfVec.getY(); Rot.at<double>(0,2)=tfVec.getZ();
        //  cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
        tfVec = tfR.getRow(1);
        Rot.at<double>(1,0)=tfVec.getX();  Rot.at<double>(1,1)=tfVec.getY(); Rot.at<double>(1,2)=tfVec.getZ();
        //cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
        tfVec = tfR.getRow(2);
        Rot.at<double>(2,0)=tfVec.getX();  Rot.at<double>(2,1)=tfVec.getY(); Rot.at<double>(2,2)=tfVec.getZ();
        // cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
        quat = tf.getRotation();

        vector_ec.at<double>(0,0)=test_pose.getX();
        vector_ec.at<double>(0,1)=test_pose.getY();
        vector_ec.at<double>(0,2)=test_pose.getZ();
        /*----------------------------------------------------------------------------*/

        for (int i =0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                New_Rot.at<double>(i,j) = Rot.at<double>(i,j) ;
            }
        }

        /*------Finding rotation matrix (Camera to base frame)-----*/
        getBaseFrameToKinect(transform);

        tfVec = transform.getOrigin();
        test_pose = tfVec;
        //      cout<<"vector from reference frame to to child frame: "<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
        tfR = transform.getBasis();
        //    cout<<"orientation of child frame w/rt reference frame: "<<endl;
        tfVec = tfR.getRow(0);
        Rot.at<double>(0,0)=tfVec.getX();  Rot.at<double>(0,1)=tfVec.getY(); Rot.at<double>(0,2)=tfVec.getZ();
        //  cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
        tfVec = tfR.getRow(1);
        Rot.at<double>(1,0)=tfVec.getX();  Rot.at<double>(1,1)=tfVec.getY(); Rot.at<double>(1,2)=tfVec.getZ();
        //cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
        tfVec = tfR.getRow(2);
        Rot.at<double>(2,0)=tfVec.getX();  Rot.at<double>(2,1)=tfVec.getY(); Rot.at<double>(2,2)=tfVec.getZ();
        // cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
        quat = tf.getRotation();

        cv::transpose(Rot,t_Rot);

        /*----velocity in base frame---*/

        V11=Rot*V1;
        V22=Rot*V2;

        /*-------------obtaining matrix to specify camera velocity interms of ee velocity. (multiplying J with  mul_Jacob transformation matrix)-------------------*/
        matrix_for_ee_to_camera(vector_ec,New_Rot,mul_Jacob);


        /*-------Obtaining robot Jacobian------------*/

        /*---------------------------moveit jacobian------------------------------------*/

        std::vector<double> current_angles;
        current_angles = arm_group.getCurrentJointValues();

        std::vector<double> joint_values;

        kinematic_state->setJointGroupPositions(joint_model_group,current_angles);

        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }

        kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                     reference_point_position,
                                     jacobian);

        ROS_INFO_STREAM("Moveit Jacobian: " << endl << jacobian);

        for (int i =0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                Jm.at<double>(i,j) = jacobian(i,j);
            }
        }

        /*---------------------------geometric jacobian------------------------------------*/

        double th[6]={joint_values[0], joint_values[1], joint_values[2], joint_values[3],joint_values[4], joint_values[5]};
        compute_new_geo_jacobian(th,Jg);
        ROS_INFO_STREAM("Geometric Jacobian: " << endl << Jg);

        /*-----------------------------------------------------------------*/
//        new_Jacob = mul_Jacob*Jm; //for moveit jacobian
        new_Jacob = mul_Jacob*Jg; //for geometric jacobian

        /*-----------------------------------------------------------------*/

        cv::invert(new_Jacob, Jginv, cv::DECOMP_SVD);

        /*---------Obtaining joint velocities-----------*/
        VV.at<double>(0,0)=V11.at<double>(0,0);
        VV.at<double>(1,0)=V11.at<double>(1,0);
        VV.at<double>(2,0)=V11.at<double>(2,0);
        VV.at<double>(3,0)=V22.at<double>(0,0);
        VV.at<double>(4,0)=V22.at<double>(1,0);
        VV.at<double>(5,0)=V22.at<double>(2,0);

//        VV.at<double>(0,0)=V11.at<double>(0,0);
//        VV.at<double>(1,0)=V11.at<double>(1,0);
//        VV.at<double>(2,0)=0;
//        VV.at<double>(3,0)=V22.at<double>(0,0);
//        VV.at<double>(4,0)=V22.at<double>(1,0);
//        VV.at<double>(5,0)=V22.at<double>(2,0);

        f2 << VV.at<double>(0,0) <<endl;
        f3 << VV.at<double>(1,0) <<endl;
        f4 << VV.at<double>(2,0) <<endl;
        f5 << VV.at<double>(3,0) <<endl;
        f6 << VV.at<double>(4,0) <<endl;
        f7 << VV.at<double>(5,0) <<endl;


        //        VV.at<double>(0,0)=V1.at<double>(0,0);
        //        VV.at<double>(1,0)=V1.at<double>(1,0);
        //        VV.at<double>(2,0)=V1.at<double>(2,0);
        //        VV.at<double>(3,0)=V2.at<double>(0,0);
        //        VV.at<double>(4,0)=V2.at<double>(1,0);
        //        VV.at<double>(5,0)=V2.at<double>(2,0);

        Theta_dot = Jginv * VV;

        f8 << Theta_dot.at<double>(0,0) <<endl;
        f9 << Theta_dot.at<double>(1,0) <<endl;
        f10 << Theta_dot.at<double>(2,0) <<endl;
        f11 << Theta_dot.at<double>(3,0) <<endl;
        f12 << Theta_dot.at<double>(4,0) <<endl;
        f13 << Theta_dot.at<double>(5,0) << endl;

        cout<< "vel_moveit = " <<" "<< (Theta_dot.at<double>(0))*180/3.14<<" "<< (Theta_dot.at<double>(1))*180/3.14<<" "<< (Theta_dot.at<double>(2))*180/3.14<<" "<< (Theta_dot.at<double>(3))*180/3.14<<" "<< (Theta_dot.at<double>(4))*180/3.14<<" "<< (Theta_dot.at<double>(5))*180/3.14<<endl;

        control_speed::velocity_control velocity;

        double vel_limit0 = abs(Theta_dot.at<double>(0))*180/3.14;
        double vel_limit1 = abs(Theta_dot.at<double>(1))*180/3.14;
        double vel_limit2 = abs(Theta_dot.at<double>(2))*180/3.14;
        double vel_limit3 = abs(Theta_dot.at<double>(3))*180/3.14;
        double vel_limit4 = abs(Theta_dot.at<double>(4))*180/3.14;
        double vel_limit5 = abs(Theta_dot.at<double>(5))*180/3.14;

        if (vel_limit0 < 12 && vel_limit1 < 10 && vel_limit2 < 10 && vel_limit3 < 10 && vel_limit4 < 10 && vel_limit5 < 10 )

        {
        velocity.vel0.data = Theta_dot.at<double>(0);
        velocity.vel1.data = Theta_dot.at<double>(1);
        velocity.vel2.data = Theta_dot.at<double>(2);
        velocity.vel3.data = Theta_dot.at<double>(5);
        velocity.vel4.data = Theta_dot.at<double>(4);
        velocity.vel5.data = Theta_dot.at<double>(3);
        }
        else
        {
            velocity.vel0.data = 0;
            velocity.vel1.data = 0;
            velocity.vel2.data = 0;
            velocity.vel3.data = 0;
            velocity.vel4.data = 0;
            velocity.vel5.data = 0;
        }

        /*--------sending velocity to control_speed node-------*/

        pb.publish(velocity);
        ros::spinOnce();

//        cout<<"xpoint"<<x_point<<endl;
//        cout<<"ypoint"<<y_point<<endl;

//        UU.at<double>(0,0) = x_point;
//        UU.at<double>(1,0) = y_point;


//        for(int i = 0; i < 2; i++)
//        {
//            u_c[i]=UU.at<double>(i,0);
//        }

//        m[0]= (u_c[0]-320)/(531.15/640);    //(u-u0)/px
//        m[1]= (u_c[1]-240)/(531.15/480);    //(v-v0)/py

//        if (isnan(centroid_depth))
//        {

//            depth_p[0]=prev_depth[0];
//            depth_p[1]=prev_depth[1];

//        }
//        else

//        {
//            depth_p[0]=(double)centroid_depth;
//            depth_p[1]=(double)centroid_depth;

//        }

////        cout<<"depth"<<(double)centroid_depth<<endl;

//        for (int p=0;p<2;p++)
//        {
//            prev_depth[p] = depth_p[p];
//        }

        double mean_error = 0;
        for(int i = 0; i < 2*dp; i++)
        {
            Error_p.at<double>(i,0) = m[i]-m_t[i];
            mean_error = mean_error + Error_p.at<double>(i,0)*Error_p.at<double>(i,0);
        }

        cout << "mean_Error_p = "<< mean_error << endl;
        f1 << mean_error << endl;

//                loop_rate.sleep();

    }
    f1.close();
    f2.close();
    f3.close();
    f4.close();
    f5.close();
    f6.close();
    f7.close();
    f8.close();
    f9.close();
    f10.close();
    f11.close();
    f12.close();
    f13.close();


    return 0;
}

