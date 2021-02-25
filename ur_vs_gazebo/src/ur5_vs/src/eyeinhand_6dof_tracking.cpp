/* Image based visual servoing using two features
 * System : UR5 Robot with realsense camera mounted on end-effector
 * Author : Deepak Raina (Emp ID: 1445893)
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
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

    //move-it initializers for getting the jacobian
    moveit::planning_interface::MoveGroup arm_group("manipulator");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    //joint states subscriber
    ros::Subscriber jointStateSub = nh.subscribe("/joint_states",10,jointStateCallback);
    //publisher for velocity
    ros::Publisher velPub= nh.advertise<control_speed::velocity_control>("/calc_velocity",10);
    //image features subscriber
    ros::Subscriber imgFeaturesSub = nh.subscribe<object_detection::image_data>("/object_points", 10, imgFeatureCallback);
    sleep(1);

    //declaration of varibales
    Mat Jm = Mat(6, 6, CV_64F, 0.0);
    Mat Jg = Mat(6, 6, CV_64F, 0.0);
    Mat Jacob_ec = Mat(6, 6, CV_64F, 0.0);
    Mat Jacob = Mat(6, 6, CV_64F, 0.0);
    Mat Jacob_inv = Mat(6, 6, CV_64F, 0.0);
    Mat vector_ec = Mat(1, 3, CV_64F, 0.0);
    Mat theta_dot = Mat(6,1, CV_64F, 0.0);
    Mat Error_p = Mat(2, 1, CV_64F, 0.0);
    cv::Mat L = cv::Mat(2, 6, CV_64F, 0.0);
    cv::Mat Linv = cv::Mat(6, 2, CV_64F, 0.0);
    cv::Mat Rot = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat Vc = cv::Mat(6, 1, CV_64F, 0.0);
    cv::Mat V1 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V2 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat VV = cv::Mat(6, 1, CV_64F, 0.0);
    cv::Mat V11 = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V22 = cv::Mat(3, 1, CV_64F, 0.0);

    tf::StampedTransform transform;
    tf::Vector3 tfVec;
    tf::Vector3 test_pose;
    tf::Matrix3x3 tfR;

    double eta = 0.1; //controller convergence parameter

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
    cout << "Current Feature " << m[0] << " " << m[1] << endl;
    cout << "Desired Feature " << m_d[0] << " " << m_d[1] << endl;
    cout << "Mean Error = " << sqrt_mean_error << endl;
    cout << "Press enter to start visual servoing " << endl;
    getchar();

    //while(ros::ok() || (sqrt_mean_error > 0))
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
        V2.at<double>(0,0)=Vc.at<double>(5,0);
        V2.at<double>(1,0)=Vc.at<double>(4,0);
        V2.at<double>(2,0)=Vc.at<double>(3,0);

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
        std::vector<double> current_angles;
        current_angles = arm_group.getCurrentJointValues();
        std::vector<double> joint_values;
        kinematic_state->setJointGroupPositions(joint_model_group,current_angles);
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
        Eigen::MatrixXd jacobian;
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

        //Numearical jacobian
        double th[6]={joint_values[0], joint_values[1], joint_values[2], joint_values[3],joint_values[4], joint_values[5]};
        cout << "current joint angles " << th[0] << " " << th[1] << " " << th[2] << " " << th[3] << " " << th[4] << " " << th[5] << endl;
        getRobotJacobian(th,Jg);
        ROS_INFO_STREAM("Geometric Jacobian: " << endl << Jg);
        double th1[6]={jp1, jp2, jp3, jp4, jp5, jp6};
        cout << "callback joint angles " << jp1 << " " << jp2 << " " << jp3 << " " << jp4 << " " << jp5 << " " << jp6 << endl;
        getRobotJacobian(th1,Jg);
        ROS_INFO_STREAM("Geometric Jacobian (callback): " << endl << Jg);


        //Obtaining joint velocities
        //Jacob = Jacob_ec*Jm; //for moveit jacobian
        Jacob = Jacob_ec*Jg; //for geometric jacobian

        cv::invert(Jacob, Jacob_inv, cv::DECOMP_SVD);

        theta_dot = Jacob_inv * VV;

        //Sending velocity to controller
        //cout<< "vel_moveit = " <<" "<< (Theta_dot.at<double>(0))*180/3.14<<" "<< (Theta_dot.at<double>(1))*180/3.14<<" "<< (Theta_dot.at<double>(2))*180/3.14<<" "<< (Theta_dot.at<double>(3))*180/3.14<<" "<< (Theta_dot.at<double>(4))*180/3.14<<" "<< (Theta_dot.at<double>(5))*180/3.14<<endl;

        control_speed::velocity_control velocity;

        double vel_limit0 = abs(theta_dot.at<double>(0))*180/3.14;
        double vel_limit1 = abs(theta_dot.at<double>(1))*180/3.14;
        double vel_limit2 = abs(theta_dot.at<double>(2))*180/3.14;
        double vel_limit3 = abs(theta_dot.at<double>(3))*180/3.14;
        double vel_limit4 = abs(theta_dot.at<double>(4))*180/3.14;
        double vel_limit5 = abs(theta_dot.at<double>(5))*180/3.14;

        if (vel_limit0 < 12 && vel_limit1 < 10 && vel_limit2 < 10 && vel_limit3 < 10 && vel_limit4 < 10 && vel_limit5 < 10 )

        {
            velocity.vel0.data = theta_dot.at<double>(0);
            velocity.vel1.data = theta_dot.at<double>(1);
            velocity.vel2.data = theta_dot.at<double>(2);
            velocity.vel3.data = theta_dot.at<double>(5);
            velocity.vel4.data = theta_dot.at<double>(4);
            velocity.vel5.data = theta_dot.at<double>(3);
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

        //Publishing velocity
        velPub.publish(velocity);

        //finding error
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
        cout << "-------------------------------------------------------" << endl;
        cout << "Current Feature " << m[0] << " " << m[1] << endl;
        cout << "Desired Feature " << m_d[0] << " " << m_d[1] << endl;
        cout << "Mean Error = " << sqrt_mean_error << endl;
        cout << "-------------------------------------------------------" << endl;

    }

    cout << "Reached Target Feature" << endl;
    return 0;
}
