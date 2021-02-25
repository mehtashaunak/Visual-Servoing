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
#include<new_detection/points_four_image_data.h>

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

void img_proc_callback(const new_detection::points_four_image_data  msg)
{
    x_point = msg.detected_point_x.data;
    y_point = msg.detected_point_y.data;
    centroid_depth = msg.detected_point_depth.data;
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

    cv::Mat Error_p = cv::Mat(2, 1, CV_64F, 0.0);
    cv::Mat deri_error = cv::Mat(2, 1, CV_64F, 0.0);
    cv::Mat deri_term = cv::Mat(2, 1, CV_64F, 0.0);

    cv::Mat pre_Error_p = cv::Mat(2, 1, CV_64F, 0.0);
    cv::Mat L = cv::Mat(2, 6, CV_64F, 0.0);
    cv::Mat L_plane = cv::Mat(2, 2, CV_64F, 0.0);
    cv::Mat L_3 = cv::Mat(2, 3, CV_64F, 0.0);
    cv::Mat Linv = cv::Mat(6, 2, CV_64F, 0.0);
    cv::Mat Linv_plane = cv::Mat(2, 2, CV_64F, 0.0);
    cv::Mat L_3inv = cv::Mat(3, 2, CV_64F, 0.0);
    cv::Mat Rot = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat New_Rot = cv::Mat(3, 3, CV_64F, 0.0);

    cv::Mat t_Rot = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat V = cv::Mat(2, 1, CV_64F, 0.0);
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

    double theta[6];

    double u_c[2]={0,0};
    double eta = 0.1; // 100 controller convergence parameter 0.03 15 5
    double depth_p[2]={0,0},prev_depth[2], u_t[2];
    double m[2]={0,0};
    double m_t[8]={0,0};

    /*----------------------------------------------getting image -----------------------------------------*/

    ros::Subscriber img_proc = nh.subscribe< new_detection::points_four_image_data>("/object_points", 10, img_proc_callback);
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

    /*-------------------Desired points---------------------------*/
    u_t[0] = 320;
    u_t[1] = 240;
    /*-------------------------------------------------------------*/

    m[0]= (u_c[0]-320)/(531.15/640);    //(u-u0)/px
    m[1]= (u_c[1]-240)/(531.15/480);    //(v-v0)/py
    m_t[0]= (u_t[0]-320)/(531.15/640);    //(u-u0)/px
    m_t[1]= (u_t[1]-240)/(531.15/480);    //(v-v0)/py

    for(int i = 0; i < 2; i++)
    {
        Error_p.at<double>(i,0) = m[i]-m_t[i];
    }

    /*---------------------------------------iteration starts here--------------------------------------*/

    while(ros::ok())
    {
        /*------Computing image jacobian--------*/

//                depth_p[0]=0.01;
//                depth_p[1]=0.01;

        compute_image_jacobian(m, depth_p, L);
//                compute_image_jacobian(m_t, depth_p, L);

        L_plane.at<double>(0,0) = L.at<double>(0,0);
        L_plane.at<double>(1,0) = L.at<double>(1,0);
        L_plane.at<double>(0,1) = L.at<double>(0,1);
        L_plane.at<double>(1,1) = L.at<double>(1,1);



//        cv::invert(L, Linv, cv::DECOMP_SVD);
        cv::invert(L_plane, Linv_plane, cv::DECOMP_SVD);


        /*--------Find camera velocity----------*/

        deri_error = Error_p/0.01 - pre_Error_p/0.01;

        deri_term = deri_error - pre_V;
        pre_Error_p.data = Error_p.data;

        V = (-eta * Linv_plane * Error_p)-deri_term;
        pre_V.data = V.data;

        V1.at<double>(0,0)=V.at<double>(0,0);
        V1.at<double>(1,0)=V.at<double>(1,0);
        V1.at<double>(2,0)=0;
        V2.at<double>(0,0)=0;
        V2.at<double>(1,0)=0;
        V2.at<double>(2,0)=0;

        //                V1.at<double>(0,0)=0.01;
        //                V1.at<double>(1,0)=0.0;
        //                V1.at<double>(2,0)=0.0;
        //                V2.at<double>(0,0)=0.0;
        //                V2.at<double>(1,0)=0.0;
        //                V2.at<double>(2,0)=0.0;

        //        V1.at<double>(0,0)=V.at<double>(0,0);
        //        V1.at<double>(1,0)=V.at<double>(1,0);
        //        V1.at<double>(2,0)=V.at<double>(2,0);
        //        V2.at<double>(0,0)=V.at<double>(3,0);
        //        V2.at<double>(1,0)=V.at<double>(4,0);
        //        V2.at<double>(2,0)=V.at<double>(5,0);

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

        //        VV.at<double>(0,0)=V1.at<double>(0,0);
        //        VV.at<double>(1,0)=V1.at<double>(1,0);
        //        VV.at<double>(2,0)=V1.at<double>(2,0);
        //        VV.at<double>(3,0)=V2.at<double>(0,0);
        //        VV.at<double>(4,0)=V2.at<double>(1,0);
        //        VV.at<double>(5,0)=V2.at<double>(2,0);

        Theta_dot = Jginv * VV;

        cout<< "vel_moveit = " <<" "<< (Theta_dot.at<double>(0))*180/3.14<<" "<< (Theta_dot.at<double>(1))*180/3.14<<" "<< (Theta_dot.at<double>(2))*180/3.14<<" "<< (Theta_dot.at<double>(3))*180/3.14<<" "<< (Theta_dot.at<double>(4))*180/3.14<<" "<< (Theta_dot.at<double>(5))*180/3.14<<endl;

        control_speed::velocity_control velocity;

        velocity.vel0.data=Theta_dot.at<double>(0);
        velocity.vel1.data =Theta_dot.at<double>(1);
        velocity.vel2.data =Theta_dot.at<double>(2);
        velocity.vel3.data =Theta_dot.at<double>(5);
        velocity.vel4.data =Theta_dot.at<double>(4);
        velocity.vel5.data =Theta_dot.at<double>(3);

        /*--------sending velocity to control_speed node-------*/

        pb.publish(velocity);
        ros::spinOnce();

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

//        cout<<"depth"<<(double)centroid_depth<<endl;

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

        cout << "mean_Error_p = "<< mean_error << endl;

        //        loop_rate.sleep();

    }

    return 0;
}
