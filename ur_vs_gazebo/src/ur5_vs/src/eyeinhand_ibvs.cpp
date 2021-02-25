/* Image based visual servoing using two features
 * System : UR5 Robot with realsense camera mounted on end-effector
 * Author : Deepak Raina (Emp ID: 1445893)
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <pcl_conversions/pcl_conversions.h>
#include <control_speed/velocity_control.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <my_funcs.h>

using namespace Eigen;
using namespace std;
using namespace cv;

ros::Publisher pb;
ros::Subscriber sb;
Mat image;

/*-------Main Function---------*/
int main (int argc, char** argv)
{
    ros::init(argc,argv,"ibvs_two_features");
    ros::NodeHandle nh;
    cv::startWindowThread();

    //joint states subscriber
    ros::Subscriber jointStateSub = nh.subscribe("/joint_states",10,jointStateCallback);
    //publisher for velocity
    ros::Publisher jointVelPub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur_driver/joint_speed", 10);
    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint trajPoint;
    //image features subscriber
    ros::Subscriber imgFeaturesSub = nh.subscribe<object_detection::image_data>("/object_points", 10, imgFeatureCallback);
    sleep(1);

    //declaration of varibales
    Mat Error_p = Mat(2, 1, CV_64F, 0.0);
    Mat L = Mat(2, 6, CV_64F, 0.0);
    Mat Linv = Mat(6, 2, CV_64F, 0.0);
    Mat Vc = Mat(6, 1, CV_64F, 0.0);
    Mat V1 = Mat(3, 1, CV_64F, 0.0);
    Mat V2 = Mat(3, 1, CV_64F, 0.0);
    Mat VV = Mat(6, 1, CV_64F, 0.0);
    Mat V11 = Mat(3, 1, CV_64F, 0.0);
    Mat V22 = Mat(3, 1, CV_64F, 0.0);
    Mat Rot = Mat(3, 3, CV_64F, 0.0);
    Mat vector_ec = Mat(1, 3, CV_64F, 0.0);
    Mat Jg = Mat(6, 6, CV_64F, 0.0);
    Mat Jacob_ec = Mat(6, 6, CV_64F, 0.0);
    Mat Jacob = Mat(6, 6, CV_64F, 0.0);
    Mat Jacob_inv = Mat(6, 6, CV_64F, 0.0);
    Mat theta_dot = Mat(6,1, CV_64F, 0.0);

    tf::StampedTransform transform;
    tf::Vector3 tfVec;
    tf::Vector3 test_pose;
    tf::Matrix3x3 tfR;

    double eta = 0.4; //controller convergence parameter

    //Getting current feature
    ros::spinOnce();
    double u_c[2]={0,0};
    u_c[0]=x_point, u_c[1]=y_point;
    double depth_p[2]={0,0}, prev_depth[2]; //depth of centroid
    depth_p[0]=(double)centroid_depth, depth_p[1]=(double)centroid_depth;
    prev_depth[0]=depth_p[0], prev_depth[1]=depth_p[1];

    double m[2]={0,0};
    m[0]= (u_c[0]-320)/(531.15/640);    //(u-u0)/px
    m[1]= (u_c[1]-240)/(531.15/480);    //(v-v0)/py

    //Desired image features
    double u_d[2]={0,0};
    u_d[0] = 320; u_d[1] = 240;
    double m_d[2]={0,0};
    m_d[0]= (u_d[0]-320)/(531.15/640);    //(u-u0)/px
    m_d[1]= (u_d[1]-240)/(531.15/480);    //(v-v0)/py

    //Error
    double mean_error = 0;
    for(int i = 0; i < 2; i++)
    {
        Error_p.at<double>(i,0) = m[i]-m_d[i];
        mean_error = mean_error + Error_p.at<double>(i,0)*Error_p.at<double>(i,0);
    }
    double sqrt_mean_error = sqrt(mean_error);
    cout << "-------------------------------------------------------" << endl;
    cout << "Current Feature = [" << m[0] << " " << m[1] << "]" << endl;
    cout << "Desired Feature = [" << m_d[0] << " " << m_d[1] << "]" << endl;
    cout << "Mean Error = " << sqrt_mean_error << endl;
    cout << "-------------------------------------------------------" << endl;
    cout << "Press enter to start visual servoing " << endl;
    getchar();

    double time = ros::Time::now().toSec();

    while(ros::ok())
    {
        //Image Jacobian
        computeImageJacobian(m,depth_p,L);
        cv::invert(L, Linv, cv::DECOMP_SVD);

        //camera velocity
        Vc = -eta * Linv * Error_p;

        //linear(V1) and angular(V2) camera velocity
        V1.at<double>(0,0)=Vc.at<double>(0,0);
        V1.at<double>(1,0)=Vc.at<double>(1,0);
        V1.at<double>(2,0)=Vc.at<double>(2,0);
        V2.at<double>(0,0)=Vc.at<double>(3,0);
        V2.at<double>(1,0)=Vc.at<double>(4,0);
        V2.at<double>(2,0)=Vc.at<double>(5,0);

        //converting camear velocity from camera frame to base frame
        getCameraToBase(transform);

        tfVec = transform.getOrigin();
        test_pose = tfVec;
        tfR = transform.getBasis();
        tfVec = tfR.getRow(0);
        Rot.at<double>(0,0)=tfVec.getX();  Rot.at<double>(0,1)=tfVec.getY(); Rot.at<double>(0,2)=tfVec.getZ();
        tfVec = tfR.getRow(1);
        Rot.at<double>(1,0)=tfVec.getX();  Rot.at<double>(1,1)=tfVec.getY(); Rot.at<double>(1,2)=tfVec.getZ();
        tfVec = tfR.getRow(2);
        Rot.at<double>(2,0)=tfVec.getX();  Rot.at<double>(2,1)=tfVec.getY(); Rot.at<double>(2,2)=tfVec.getZ();

        V11=Rot*V1;
        V22=Rot*V2;

        VV.at<double>(0,0)=V11.at<double>(0,0);
        VV.at<double>(1,0)=V11.at<double>(1,0);
        VV.at<double>(2,0)=V11.at<double>(2,0);
        VV.at<double>(3,0)=V22.at<double>(0,0);
        VV.at<double>(4,0)=V22.at<double>(1,0);
        VV.at<double>(5,0)=V22.at<double>(2,0);

        //Jacobian from end-effector to camera
        getEETocamera(transform);

        tfVec = transform.getOrigin();
        test_pose = tfVec;
        tfR = transform.getBasis();
        tfVec = tfR.getRow(0);
        Rot.at<double>(0,0)=tfVec.getX();  Rot.at<double>(0,1)=tfVec.getY(); Rot.at<double>(0,2)=tfVec.getZ();
        tfVec = tfR.getRow(1);
        Rot.at<double>(1,0)=tfVec.getX();  Rot.at<double>(1,1)=tfVec.getY(); Rot.at<double>(1,2)=tfVec.getZ();
        tfVec = tfR.getRow(2);
        Rot.at<double>(2,0)=tfVec.getX();  Rot.at<double>(2,1)=tfVec.getY(); Rot.at<double>(2,2)=tfVec.getZ();

        vector_ec.at<double>(0,0)=test_pose.getX();
        vector_ec.at<double>(0,1)=test_pose.getY();
        vector_ec.at<double>(0,2)=test_pose.getZ();

        JacobianEEtoCamera(vector_ec,Rot,Jacob_ec);

        //Obtaining robot Jacobian
        double th[6]={jp1, jp2, jp3, jp4, jp5, jp6};
        //cout << "current joint angles " << jp1 << " " << jp2 << " " << jp3 << " " << jp4 << " " << jp5 << " " << jp6 << endl;
        getRobotJacobian(th,Jg);
        //ROS_INFO_STREAM("Geometric Jacobian " << endl << Jg);

        //Obtaining joint velocities
        Jacob = Jacob_ec*Jg;
        cv::invert(Jacob, Jacob_inv, cv::DECOMP_SVD);
        theta_dot = Jacob_inv * VV;
        //cout << "Joint velocity " << theta_dot << endl;

        //Sending velocity to controller
        traj.points.clear();
        trajPoint.velocities.clear();
        trajPoint.accelerations.clear();
        double vel_limit = 1.0; //i.e. 1 radian/sec //applying velocity limit for safety
        double acc = 0.1;
        if (theta_dot.at<double>(0,0) < vel_limit && theta_dot.at<double>(1,0)< vel_limit && theta_dot.at<double>(2,0) < vel_limit && theta_dot.at<double>(3,0) < vel_limit && theta_dot.at<double>(4,0) < vel_limit && theta_dot.at<double>(5,0) < vel_limit )

        {
            trajPoint.velocities.push_back(theta_dot.at<double>(0));
            trajPoint.velocities.push_back(theta_dot.at<double>(1));
            trajPoint.velocities.push_back(theta_dot.at<double>(2));
            trajPoint.velocities.push_back(theta_dot.at<double>(3));
            trajPoint.velocities.push_back(theta_dot.at<double>(4));
            trajPoint.velocities.push_back(theta_dot.at<double>(5));

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

        //cout << "Press enter to start robot ... " << endl;
        //getchar();
        traj.points.push_back(trajPoint);
        jointVelPub.publish(traj); //joint speed publisher

        //finding current error
        ros::spinOnce();
        u_c[0]=x_point;
        u_c[1]=y_point;

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
        prev_depth[0] = depth_p[0],prev_depth[1] = depth_p[1];

        mean_error=0;
        for(int i = 0; i < 2; i++)
        {
            Error_p.at<double>(i,0) = m[i]-m_d[i];
            mean_error = mean_error + Error_p.at<double>(i,0)*Error_p.at<double>(i,0);
        }
        sqrt_mean_error = sqrt(mean_error);

        cout << "----------------------------------------------" << endl;
        cout << "Current Feature = [" << m[0] << " " << m[1] << "]" << endl;
        cout << "Desired Feature = [" << m_d[0] << " " << m_d[1] << "]" << endl;
        cout << "Mean Error = " << sqrt_mean_error << endl;
        cout << "-----------------------------------------------" << endl;

        if (sqrt_mean_error < 1)
            break;
    }
    double duration = ros::Time::now().toSec() - time;
    cout << "Reached Target Feature in " << duration << " seconds" << endl;
    return 0;
}

