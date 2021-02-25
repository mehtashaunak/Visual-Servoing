// image based servoing
#include <ros/ros.h>
#include <ros/package.h>
#include <apc_simulation.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <iostream>
#include <ctime>
#include <vector>
#include <cstdlib>
#include <iterator>
#include <fstream>
#include <std_msgs/Empty.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <ur5_control/UR5Goal.h>
//#include "plannig/UR5Goal.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include<std_msgs/Bool.h>
#include<moveit/move_group_interface/move_group.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/RobotState.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

//for velocity control
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include<sensor_msgs/JointState.h>
#include<control_speed/velocity_control.h>

//for jacobian
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf/transform_listener.h>

#include<std_msgs/String.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include<ur5model.h>
#include<cmath>
#include<gnuplot_ci.h>
#include <my_funcs.h>
#include<object_detect/image_data.h>
#include<std_msgs/MultiArrayDimension.h>
#include<std_msgs/MultiArrayLayout.h>
#include<std_msgs/Float64MultiArray.h>

using namespace MODUR5;
using namespace gnuplot_ci;
using namespace Eigen;

using namespace std;
using namespace cv;
//using namespace numpy;

ros::Publisher pb;
ros::Subscriber sb;
trajectory_msgs::JointTrajectory jt;
double joint_pos_global;
double angle_degree;


#define NF 2
const double lambda = 1;
sensor_msgs::ImageConstPtr imageIn;
cv::Mat image;

/*-------- point cloud call back function------------- */
void ptCloudcallBack(const sensor_msgs::PointCloud2::ConstPtr &input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud, bool &flag)
{
    //    std::cout << input->header.frame_id << endl;
    //    std::cout << "in call back" << endl;
    pcl::fromROSMsg( *input, *ptCloud );
    flag = true;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imageIn=msg;
    try
    {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void kinectToBaseFrame_new(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listner;
    transform_listner.waitForTransform("/base_link","/camera_depth_optical_frame",ros::Time(0), ros::Duration(3));
    transform_listner.lookupTransform("/base_link","/camera_depth_optical_frame",ros::Time(0), transform);
}

void getBaseFrameToEE(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
    //  tf::StampedTransform transform_kinect_j1
    //  base to ee
    transform_listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), transform);
    //  ee to base
    //  transform_listener.waitForTransform("/ee_link", "/base_link", ros::Time(0), ros::Duration(3));
    //  transform_listener.lookupTransform("/ee_link", "/base_link", ros::Time(0), transform);

    return;
}
void getEETocamera(tf::StampedTransform &transform)
{
    tf::TransformListener transform_listener;
    //  tf::StampedTransform transform_kinect_j1
    //  base to ee
    //        transform_listener.waitForTransform("/ee_link", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(3));
    //        transform_listener.lookupTransform("/ee_link", "/camera_depth_optical_frame", ros::Time(0), transform);

//    transform_listener.waitForTransform("/camera_depth_optical_frame","/ee_link",  ros::Time(0), ros::Duration(3));
//    transform_listener.lookupTransform( "/camera_depth_optical_frame","/ee_link", ros::Time(0), transform);

    transform_listener.waitForTransform("/camera_link","/ee_link",  ros::Time(0), ros::Duration(3));
    transform_listener.lookupTransform( "/camera_link","/ee_link", ros::Time(0), transform);

    //        ee to base
//      transform_listener.waitForTransform("/camera_link", "/base_link", ros::Time(0), ros::Duration(3));
//      transform_listener.lookupTransform("/camera_link", "/base_link", ros::Time(0), transform);

    return;
}


//callback for velocity control

void callback(const sensor_msgs::JointState & msg)
{
    joint_pos_global = (double)msg.velocity[4];

    //            cout<<"direct_pos"<<(double)msg.position[4]<<endl;
    //            cout<<"velocity"<<" "<<joint_pos_global<<endl;
    //        angle_degree = joint_pos_global*180/3.14;
    //            cout<<"angle"<<" "<<angle_degree<<endl;
}
double x_point, y_point;
float centroid_depth;

void img_proc_callback(const object_detect::image_data  msg)
{
    x_point = msg.detected_point_x.data;
    y_point = msg.detected_point_y.data;
    centroid_depth = msg.detected_point_depth.data;
}



/*-----------Main Function----------------*/
int main (int argc, char** argv)
{

    UR5 robot; // Define an UR5 object

    ros::init(argc,argv,"test_node");
    ros::AsyncSpinner spinner(1);
    // spinner.start();

    ros::NodeHandle nh, node, mh;
    cv::startWindowThread();

    moveit::planning_interface::MoveGroup arm_group("Arm1");
    arm_group.setPlanningTime(25);// set the maximum planning time

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    //  kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("Arm1");
    //  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    //  Service call
    ros::ServiceClient ur5SingleGoalService = nh.serviceClient<ur5_control::UR5Goal>("/ur5/single_goal");
    //        ros::ServiceClient ur5TrajGoalService = nh.serviceClient<ur5_control::UR5Goal>("/ur5/trajectory_goal");
    ur5_control::UR5Goal ur5_goal;


    //  ros::ServiceClient client1 = p.serviceClient<plannig::UR5Goal>("/position/goal");
    //  plannig::UR5Goal srv1;

    /*--------------------Declarations------------------------*/
    double theta_max[NL], theta_min[NL];
    robot.get_angle_range(theta_max, theta_min);
    ros::Rate loop_rate(10); //25 125 10
    sb = mh.subscribe("joint_states",10,callback);

    pb= nh.advertise<control_speed::velocity_control>("/calc_velocity",10);


    cv::Mat Jg = cv::Mat(NW, NL, CV_64F, 0.0);
    cv::Mat Jginv = cv::Mat(NL, NW, CV_64F, 0.0);
    cv::Mat mul_Jacob = cv::Mat(NL, NW, CV_64F, 0.0);
    cv::Mat new_Jacob = cv::Mat(NL, NW, CV_64F, 0.0);

    cv::Mat S_aec = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat S_aec_mul = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat vector_ec = cv::Mat(1, 3, CV_64F, 0.0);

    cv::Mat Theta_dot = cv::Mat(NL,1, CV_64F, 0.0);
    cv::Mat Theta_dot_3 = cv::Mat(3,1, CV_64F, 0.0);


    cv::Mat Error_p = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat L = cv::Mat(3, NW, CV_64F, 0.0);
    cv::Mat L_3 = cv::Mat(2, 3, CV_64F, 0.0);
    cv::Mat Linv = cv::Mat(NW, 3, CV_64F, 0.0);
    cv::Mat L_3inv = cv::Mat(3, 2, CV_64F, 0.0);
    cv::Mat Rot = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat New_Rot = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat tNew_Rot = cv::Mat(3, 3, CV_64F, 0.0);

    cv::Mat t_Rot = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat V = cv::Mat(NW, 1, CV_64F, 0.0);
    cv::Mat V_3 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V1 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V2 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat VV = cv::Mat(NW, 1, CV_64F, 0.0);

    cv::Mat V_e1 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V_e2 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V11 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V11_3 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V22 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat Cj = cv::Mat(NR,NC,CV_64F,0.0);
    cv::Mat UU = cv::Mat(2, 1, CV_64F, 0.0);;

    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;

    tf::StampedTransform transform;
    tf::Transform tf;
    tf::Vector3 tfVec;
    tf::Vector3 test_pose;
    tf::Matrix3x3 tfR;
    tf::Quaternion quat;
    /*--------------files declaration------------*/

    std::ofstream f4;
    std::ofstream f5;
    std::ofstream f6;


    std::vector<KeyPoint> keypoints_new;
    cv::Mat UUU;

    double theta[NO_JOINTS];

//    double theta_deg[NO_JOINTS] = {-147.23,-61.28,44.71,-89.45,-83.73,86.56}; //new values for real robot
//    double theta_deg[NO_JOINTS] = {-19.14,-81.94,36.40,-73.78,-92.24,47.24};
     double theta_deg[NO_JOINTS] = {-175.55,-80.62,-76.09,-113.22,94.48,53.44};

    double pose[NW];
    double u_c[2]={0,0};
    double eta = 0.0001; // 100 controller convergence parameter 0.03 15
    double x_t[NC],depth_p[2]={0,0},prev_depth[2], u_t[2];
    double m[2]={0,0};
    double m_t[8]={0,0};
    double ang_deg4plot[6];
    double ang_vel_deg4plot[6];

    int count=0;
    int j=0;
    float vel[6]={0,0,0,0,0,0};

    for (int i=0;i<NO_JOINTS;i++)
    {
        theta[i] = DEG2RAD(theta_deg[i]);
    }

    /*------------go to start position---------------------*/

    cout << "Start position call" << endl;
    ur5_goal.request.goal.data.clear();
    for( int i = 0; i < NO_JOINTS; i++ )
    {
        ur5_goal.request.goal.data.push_back( theta[i] );
        cout << "angles" << ur5_goal.request.goal.data[i];

    }

    if( ur5SingleGoalService.call( ur5_goal ) )
        cout << "start position reached " << endl;
    else
        cout << "Failed to reach start position" << endl;

    /*----------------current location-----------------------*/
    robot.set_joint_angles(theta);
    robot.fwd_kin_pose(pose);
    robot.joint_position(Cj);

    /*----------------getting image -----------------------------*/

    ros::Subscriber img_proc = nh.subscribe< object_detect::image_data>("/object_points", 10, img_proc_callback);
        sleep(1);
    ros::spinOnce();

    cout<<"xpoint"<<x_point<<endl;
    cout<<"ypoint"<<y_point<<endl;

    UU.at<double>(0,0) = x_point; //Obtaining from img_proc_callback
    UU.at<double>(1,0) = y_point; //Obtaining from img_proc_callback


    for(int i = 0; i < 2; i++)
    {
        u_c[i]=UU.at<double>(i,0);
    }

    depth_p[0]=(double)centroid_depth;
    depth_p[1]=(double)centroid_depth;

    for (int p=0;p<2;p++)
    {
        prev_depth[p] = depth_p[p];
    }

    cout<<"depth"<<(double)centroid_depth<<endl;

    /*-------------------Desired points---------------------------*/


    u_t[0] = 320;
    u_t[1] = 240;
 /*--------------------------------------------------------------*/

    m[0]= (u_c[0]-320)/(531.15/640);    //(u-u0)/px
    m[1]= (u_c[1]-240)/(531.15/480);    //(v-v0)/py
    m_t[0]= (u_t[0]-320)/(531.15/640);    //(u-u0)/px
    m_t[1]= (u_t[1]-240)/(531.15/480);    //(v-v0)/py

    for(int i = 0; i < 2; i++)
    {
        Error_p.at<double>(i,0) = m[i]-m_t[i];
    }
    Error_p.at<double>(2,0) = depth_p[0]-0.35 ;
    /*-----------------------iteration starts here-----------------------*/

    while(ros::ok())
    {
        /*------Computing image jacobian--------*/

        depth_p[0]=0.01;
        depth_p[1]=0.01;

//        compute_image_jacobian(m, depth_p, L);
//        compute_image_jacobian(m_t, depth_p, L);
        compute_image_jacobian_depth(m, depth_p, L);


        L_3.at<double>(0,0) = L.at<double>(0,0);
        L_3.at<double>(0,1) = L.at<double>(0,1);
        L_3.at<double>(0,2) = L.at<double>(0,2);
        L_3.at<double>(1,0) = L.at<double>(1,0);
        L_3.at<double>(1,1) = L.at<double>(1,1);
        L_3.at<double>(1,2) = L.at<double>(1,2);

        cout<<"jacobian"<<endl;
        cv::invert(L, Linv, cv::DECOMP_SVD);
        cv::invert(L_3, L_3inv, cv::DECOMP_SVD);

        cout<<"inverse"<<endl;
        /*-------Find camera velocity----------*/

        V = -eta * Linv * Error_p;
//        V_3 = -eta * L_3inv * Error_p;


        cout<<"velocity"<<endl;

        /*------Splitting linear and angualar velocities-----*/

        V1.at<double>(0,0)=V.at<double>(0,0);
        V1.at<double>(1,0)=V.at<double>(1,0);
        V1.at<double>(2,0)=V.at<double>(2,0);
        V2.at<double>(0,0)=V.at<double>(5,0);
        V2.at<double>(1,0)=V.at<double>(4,0);
        V2.at<double>(2,0)=V.at<double>(3,0);

        /*----------------------------------------------------------------------------*/
        getEETocamera(transform);
        //            cout<<"frame_id: "<<transform.frame_id_<<endl;
        //            cout<<"child_frame_id: "<<transform.child_frame_id_<<endl;

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
        V_e1=Rot*V1;
        V_e2=Rot*V2;

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
        /*----------------------------------------------------------------------------*/
        getBaseFrameToKinect(transform);
        //            cout<<"frame_id: "<<transform.frame_id_<<endl;
        //            cout<<"child_frame_id: "<<transform.child_frame_id_<<endl;

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

        cv::transpose(Rot,t_Rot);

        /*------------------------------------------------------------*/
        //            cout<<"xpose"<<test_pose.getX()<<endl;
        //            cout<<"ypose"<<test_pose.getY()<<endl;
        //            cout<<"zpose"<<test_pose.getZ()<<endl;

        /*------------------------------------------------------------*/
        //            V11=Rot*V_e1;
        //            V22=Rot*V_e2;

        V11=Rot*V1;
        V22=Rot*V2;


//        V11_3 = Rot*V_3;

//        V11=t_Rot*V1;
//        V22=t_Rot*V2;

        /*-------------obtaining matrix to specify camera velocity interms of ee velocity. (multiplying J with  mul_Jacob transformation matrix)-------------------*/
        matrix_for_ee_to_camera(vector_ec,New_Rot,mul_Jacob);

        /*------------------------------------------------------------*/
        //          V11=t_Rot*V1;
        //          V22=t_Rot*V2;
        //          V11=V1;
        //          V22=V2;

        /*---Obtaining robot Jacobian------*/

        //            arm_group.getCurrentState()->copyJointGroupPositions(arm_group.getCurrentState()->getRobotModel()->getJointModelGroup(arm_group.getName()), group_variable_values);
        jacobian= arm_group.getCurrentState()->getJacobian(joint_model_group, reference_point_position);
//        ROS_INFO_STREAM("Jacobian: " << endl << jacobian);

        //  kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position,jacobian);
        //  ROS_INFO_STREAM("Jacobian: " << endl << jacobian);

        for (int i =0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                Jg.at<double>(i,j) = jacobian(i,j);
            }
        }
        //  robot.compute_geometric_jacobian(Jg);

        /*-----------------------------------------------------------------*/

        new_Jacob = mul_Jacob*Jg;
        /*-----------------------------------------------------------------*/

        /*----Obtaining Inverse of robot jacobian--------*/
//         cv::invert(Jg, Jginv, cv::DECOMP_SVD);

        cv::invert(new_Jacob, Jginv, cv::DECOMP_SVD);

        //            cv::invert(Jg, Jginv, cv::DECOMP_SVD);

        /*---------Obtaining joint velocities-----------*/
        VV.at<double>(0,0)=V11.at<double>(0,0);
        VV.at<double>(1,0)=V11.at<double>(1,0);
        VV.at<double>(2,0)=V11.at<double>(2,0);
        VV.at<double>(3,0)=V22.at<double>(0,0);
        VV.at<double>(4,0)=V22.at<double>(1,0);
        VV.at<double>(5,0)=V22.at<double>(2,0);


//        VV.at<double>(0,0)=V1.at<double>(0,0);
//        VV.at<double>(1,0)=V1.at<double>(1,0);
//        VV.at<double>(2,0)=V1.at<double>(2,0);
//        VV.at<double>(3,0)=V2.at<double>(0,0);
//        VV.at<double>(4,0)=V2.at<double>(1,0);
//        VV.at<double>(5,0)=V2.at<double>(2,0);

        Theta_dot = Jginv * VV;

//        Theta_dot_3.at<double>(0,0) = Jginv.at<double>(0,0)*V11_3.at<double>(0,0)+Jginv.at<double>(0,1)*V11_3.at<double>(0,1)+Jginv.at<double>(0,2)*V11_3.at<double>(0,2);
//        Theta_dot_3.at<double>(0,1) = Jginv.at<double>(1,0)*V11_3.at<double>(0,0)+Jginv.at<double>(1,1)*V11_3.at<double>(0,1)+Jginv.at<double>(1,2)*V11_3.at<double>(0,2);
//        Theta_dot_3.at<double>(0,2) = Jginv.at<double>(2,0)*V11_3.at<double>(0,0)+Jginv.at<double>(2,1)*V11_3.at<double>(0,1)+Jginv.at<double>(2,2)*V11_3.at<double>(0,2);

        cout<< "vel0 = " <<" "<< (Theta_dot.at<double>(0))*180/3.14<<" "<< (Theta_dot.at<double>(1))*180/3.14<<" "<< (Theta_dot.at<double>(2))*180/3.14<<" "<< (Theta_dot.at<double>(3))*180/3.14<<" "<< (Theta_dot.at<double>(4))*180/3.14<<" "<< (Theta_dot.at<double>(5))*180/3.14<<endl;


        control_speed::velocity_control velocity;

        velocity.vel0.data=Theta_dot.at<double>(0);
        velocity.vel1.data =Theta_dot.at<double>(1);
        velocity.vel2.data =Theta_dot.at<double>(2);
        velocity.vel3.data =Theta_dot.at<double>(5);
        velocity.vel4.data =Theta_dot.at<double>(4);
        velocity.vel5.data =Theta_dot.at<double>(3);

//        velocity.vel0.data=Theta_dot_3.at<double>(0);
//        velocity.vel1.data =Theta_dot_3.at<double>(1);
//        velocity.vel2.data =Theta_dot_3.at<double>(2);
//        velocity.vel3.data =Theta_dot.at<double>(3);
//        velocity.vel4.data =Theta_dot.at<double>(4);
//        velocity.vel5.data =Theta_dot.at<double>(5);

        /*--------sending velocity to control_speed node-------*/

        pb.publish(velocity);
        ros::spinOnce();


        /*----------------------------Finding corner points from image--------------------------------------*/

//        ros::Subscriber img_proc = nh.subscribe<object_detect::image_data>("/object_points", 10, img_proc_callback);
//        sleep(1);
//        ros::spinOnce();

        cout<<"xpoint"<<x_point<<endl;
        cout<<"ypoint"<<y_point<<endl;

        UU.at<double>(0,0) = x_point;
        UU.at<double>(1,0) = y_point;


        for(int i = 0; i < 2; i++)
        {
            u_c[i]=UU.at<double>(i,0);
        }

        m[0]= (u_c[0]-320)/(531.15/640);    //(u-u0)/px
        m[1]= (u_c[1]-240)/(531.15/480);    //(v-v0)/py

        cout<<"depth"<<centroid_depth<<endl;

        if (isnan(centroid_depth))
        {

            depth_p[0]=prev_depth[0];
            depth_p[1]=prev_depth[1];

        }
        else

        {
            depth_p[0]=(double)centroid_depth;
            depth_p[1]=(double)centroid_depth;

        }

        cout<<"depth"<<(double)centroid_depth<<endl;

        for (int p=0;p<2;p++)
        {
            prev_depth[p] = depth_p[p];
        }

        double mean_error = 0;
        for(int i = 0; i < 2; i++)
        {
            Error_p.at<double>(i,0) = m[i]-m_t[i];
            mean_error = mean_error + Error_p.at<double>(i,0)*Error_p.at<double>(i,0);
        }
        Error_p.at<double>(2,0) = depth_p[0]-0.1 ;
        cout << "mean_Error_p = "<< mean_error << endl;

//        loop_rate.sleep();

    }

    return 0;
}
