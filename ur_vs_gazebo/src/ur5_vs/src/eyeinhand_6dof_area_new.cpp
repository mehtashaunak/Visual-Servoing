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

//for geometric jacobian
#include <ur5_geo_jacobian.h>

#include <my_funcs.h>

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


double x_point, y_point, area_img_manual, area_img_manual_pix, area_img_current, area_img_desired;
float centroid_depth;
double points_corner_pix[8];
double points_corner[8];

void img_proc_callback(const new_detection::points_four_image_data  msg)
{
    x_point = msg.detected_point_x.data;
    y_point = msg.detected_point_y.data;
    centroid_depth = msg.detected_point_depth.data;
    area_img_current = msg.area.data;
    points_corner[0]=  (msg.detected_point_0.data-320)/(531.15/640);
    points_corner[1] = (msg.detected_point_1.data-240)/(531.15/480);
    points_corner[2] = (msg.detected_point_2.data-320)/(531.15/640);
    points_corner[3] = (msg.detected_point_3.data-240)/(531.15/480);
    points_corner[4] = (msg.detected_point_4.data-320)/(531.15/640);
    points_corner[5] = (msg.detected_point_5.data-240)/(531.15/480);
    points_corner[6] = (msg.detected_point_6.data-320)/(531.15/640);
    points_corner[7] = (msg.detected_point_7.data-240)/(531.15/480);

    points_corner_pix[0]=  msg.detected_point_0.data;
    points_corner_pix[1] = msg.detected_point_1.data;
    points_corner_pix[2] = msg.detected_point_2.data;
    points_corner_pix[3] = msg.detected_point_3.data;
    points_corner_pix[4] = msg.detected_point_4.data;
    points_corner_pix[5] = msg.detected_point_5.data;
    points_corner_pix[6] = msg.detected_point_6.data;
    points_corner_pix[7] = msg.detected_point_7.data;

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
    cv::Mat Theta_dot_prev = cv::Mat(6,1, CV_64F, 0.0);

    cv::Mat Error_p = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat Error_p_area = cv::Mat(1, 1, CV_64F, 0.0);
    cv::Mat L = cv::Mat(3, 6, CV_64F, 0.0);
    cv::Mat L_area = cv::Mat(1, 6, CV_64F, 0.0);
    cv::Mat L_3 = cv::Mat(2, 3, CV_64F, 0.0);
    cv::Mat Linv = cv::Mat(6, 3, CV_64F, 0.0);
    cv::Mat L_3inv = cv::Mat(3, 2, CV_64F, 0.0);
    cv::Mat L_area_inv = cv::Mat(6, 1, CV_64F, 0.0);
    cv::Mat Rot = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat New_Rot = cv::Mat(3, 3, CV_64F, 0.0);

    cv::Mat t_Rot = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat V = cv::Mat(6, 1, CV_64F, 0.0);
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
//    double eta = 0.08; // 100 controller convergence parameter 0.03 15 5 2
    double eta = 0.1;
    double eta_area = 0.1;
    double depth_p[2]={0,0},prev_depth[2], u_t[2];
    double m[3]={0,0,0};
    double m_t[3]={0,0,0};

    /*----------------------------------------------getting image -----------------------------------------*/

    ros::Subscriber img_proc = nh.subscribe< new_detection::points_four_image_data>("/object_points", 10, img_proc_callback);
    sleep(1);
    ros::spinOnce();


    /*-------------------Desired points---------------------------*/
    u_t[0] = 252;
    u_t[1] = 168;
    area_img_desired = 2211360;
    /*-------------------------------------------------------------*/

    depth_p[0]=(double)centroid_depth;
    depth_p[1]=(double)centroid_depth;

    for (int p=0;p<2;p++)
    {
        prev_depth[p] = depth_p[p];
    }
    ofstream f1("/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/feature_executed.txt", std::ios_base::trunc);
    ofstream f2("/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_angles.txt", std::ios_base::trunc);
    ofstream f3("/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt", std::ios_base::trunc);
    ofstream f4("/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/camera velocities.txt", std::ios_base::trunc);
    ofstream f5("/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/feature_error.txt", std::ios_base::trunc);
    ofstream f6("/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/dataset.txt", std::ios_base::trunc);

    /*---------------------------------------iteration starts here--------------------------------------*/
    ros::Rate rate(180);
    int serial_no = 0;
    while(ros::ok())
    {

        UU.at<double>(0,0) = x_point; //Obtaining from img_proc_callback
        UU.at<double>(1,0) = y_point; //Obtaining from img_proc_callback
        for(int i = 0; i < 2; i++)
        {
            u_c[i]=UU.at<double>(i,0);
        }
        m[0] = (u_c[0]-320)/(531.15/640);    //(u-u0)/px
        m[1] = (u_c[1]-240)/(531.15/480);    //(v-v0)/py
        /*------Computing image jacobian--------*/

//                depth_p[0]=0.01;
//                depth_p[1]=0.01;

//        compute_image_jacobian(m, depth_p, L);
//        area_interaction_matrix( m, points_corner, depth_p, area_img_manual, L);

        cout << "image coordinates "<< m[0] << "\t" << m[1] << "\t" << points_corner[0] << "\t" << points_corner[1]<< "\t" << points_corner[2]<< "\t" << points_corner[3]<< "\t" << points_corner[4] << "\t" << points_corner[5]<< "\t" << points_corner[6]<< "\t" << points_corner[7] << "\t" << endl;

        cout << "pixel coordinates "<< u_c[0] << "\t" << u_c[1] << "\t" << points_corner_pix[0] << "\t" << points_corner_pix[1]<< "\t" << points_corner_pix[2]<< "\t" << points_corner_pix[3]<< "\t" << points_corner_pix[4] << "\t" << points_corner_pix[5]<< "\t" << points_corner_pix[6]<< "\t" << points_corner_pix[7] << "\t" << endl;

        area_interaction_matrix( u_c, points_corner_pix, depth_p, area_img_manual_pix, L);
        area_interaction_matrix( m, points_corner, depth_p, area_img_manual, L);


        cout << "area in pixel " << area_img_manual_pix << endl;
        cout << "area in image " << area_img_manual << endl;


        f6 << serial_no << "\t" << x_point << "\t" << y_point << "\t" << points_corner[0] << "\t" << points_corner[1]<< "\t" << points_corner[2]<< "\t" << points_corner[3]<< "\t" << points_corner[4] << "\t" << points_corner[5]<< "\t" << points_corner[6]<< "\t" << points_corner[7] << "\t" << area_img_current << endl;


        //m[2] = 0.5*area_img_current;
        m[2] = area_img_current;

        m_t[0] = (u_t[0]-320)/(531.15/640);    //(u-u0)/px
        m_t[1] = (u_t[1]-240)/(531.15/480);    //(v-v0)/py
        //m_t[2] =  0.5*area_img_desired;
        m_t[2] =  area_img_desired;

        f1 << m[0] << "\t"<< m[1] << "\t"<< m[2] << "\t" << endl;  //saving features to file

        cout << "current feature "<< m[0] << "\t"<< m[1] << "\t"<< m[2] << endl;
        cout << "desired feature "<< m_t[0] << "\t"<< m_t[1] << "\t"<< m_t[2] << endl;

        double mean_error = 0;
        for(int i = 0; i < 2; i++)
        {
            Error_p.at<double>(i,0) = m[i]-m_t[i];
            mean_error = mean_error + Error_p.at<double>(i,0)*Error_p.at<double>(i,0);
        }

        cout << "mean_error" << mean_error << endl;

        //Error_p.at<double>(2,0) = abs(300*(area_img_current - area_img_desired)); //100
        Error_p.at<double>(2,0) = (area_img_current - area_img_desired); //100

        f5 << Error_p.at<double>(0,0) << "\t"<< Error_p.at<double>(1,0) << "\t"<< Error_p.at<double>(2,0)<< "\t" << mean_error << endl;  //saving features to file


        Error_p_area.at<double>(0,0) = 1000*(area_img_current - area_img_desired);
        cout << "Error_p = "<< Error_p << endl;
//        cout << "area_Error = "<< Error_p_area << endl;

        for (int i=0;i<6;i++)
        {
        L_area.at<double>(0,i) = L.at<double>(2,i);
        }
//        ROS_INFO_STREAM("area_interaction " << endl << L_area);
        cv::invert(L, Linv, cv::DECOMP_SVD);
        cv::invert(L_area, L_area_inv, cv::DECOMP_SVD);

        /*--------Find camera velocity----------*/

/*---------------------------------------------------------*/

        double n1 = pow(Error_p.at<double>(0,0),2);
        double n2 = pow(Error_p.at<double>(1,0),2);
        double n3 = pow(Error_p.at<double>(2,0),2);

        double n = sqrt(n1+n2+n3);

        double e1_norm = Error_p.at<double>(0,0)/n;
        double e2_norm = Error_p.at<double>(1,0)/n;
        double e3_norm = Error_p.at<double>(2,0)/n;

//        Error_p.at<double>(0,0)=e1_norm;
//        Error_p.at<double>(1,0)=e2_norm;
//        Error_p.at<double>(2,0)=e3_norm;

/*---------------------------------------------------------*/

        V = -eta * Linv * Error_p;
//        V = -eta_area * L_area_inv * Error_p_area;

//        ROS_INFO_STREAM("camera_velocity " << endl << V);
//        ROS_INFO_STREAM("L_inv " << endl << Linv);

        cout << "Camera Velocity in Cam frame" << V << endl;

        V1.at<double>(0,0)=V.at<double>(0,0);
        V1.at<double>(1,0)=V.at<double>(1,0);
        V1.at<double>(2,0)=V.at<double>(2,0);
        V2.at<double>(0,0)=V.at<double>(5,0);
        V2.at<double>(1,0)=V.at<double>(4,0);
        V2.at<double>(2,0)=V.at<double>(3,0);

//                        V1.at<double>(0,0)=0.0;
//                        V1.at<double>(1,0)=0.0;
//                        V1.at<double>(2,0)=0.01;
//                        V2.at<double>(0,0)=0.0;
//                        V2.at<double>(1,0)=0.0;
//                        V2.at<double>(2,0)=0.0;

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
//        for(std::size_t i = 0; i < joint_names.size(); ++i)
//        {
//            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
//        }

        kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                     reference_point_position,
                                     jacobian);

//        ROS_INFO_STREAM("Moveit Jacobian: " << endl << jacobian);

        for (int i =0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                Jm.at<double>(i,j) = jacobian(i,j);
            }
        }

        /*---------------------------geometric jacobian------------------------------------*/

        double th[6]={joint_values[0], joint_values[1], joint_values[2], joint_values[3],joint_values[4], joint_values[5]};
        f2 << th[0]<< "\t"<< th[1] << "\t" << th[2] << "\t" << th[3] << "\t" << th[4] << "\t" << th[5] << "\t" << endl;  //saving joint angles to file
        compute_new_geo_jacobian(th,Jg);
//        ROS_INFO_STREAM("Geometric Jacobian: " << endl << Jg);

        /*-----------------------------------------------------------------*/
        new_Jacob = mul_Jacob*Jm; //for moveit jacobian
//        new_Jacob = mul_Jacob*Jg; //for geometric jacobian

        /*-----------------------------------------------------------------*/

        cv::invert(new_Jacob, Jginv, cv::DECOMP_SVD);

        /*---------Obtaining joint velocities-----------*/
        VV.at<double>(0,0)=V11.at<double>(0,0);
        VV.at<double>(1,0)=V11.at<double>(1,0);
        VV.at<double>(2,0)=V11.at<double>(2,0);
        VV.at<double>(3,0)=V22.at<double>(0,0);
        VV.at<double>(4,0)=V22.at<double>(1,0);
        VV.at<double>(5,0)=V22.at<double>(2,0);

        f4 << VV.at<double>(0,0) << "\t"<< VV.at<double>(1,0) << "\t"<< VV.at<double>(2,0) << "\t"<< VV.at<double>(3,0) << "\t"<< VV.at<double>(4,0) << "\t"<< VV.at<double>(5,0) << "\t" << endl;  //saving camera velocities to file

        //        VV.at<double>(0,0)=V1.at<double>(0,0);
        //        VV.at<double>(1,0)=V1.at<double>(1,0);
        //        VV.at<double>(2,0)=V1.at<double>(2,0);
        //        VV.at<double>(3,0)=V2.at<double>(0,0);
        //        VV.at<double>(4,0)=V2.at<double>(1,0);
        //        VV.at<double>(5,0)=V2.at<double>(2,0);

        Theta_dot = Jginv * VV;

        cout<< "joint_velocity = " <<" "<< (Theta_dot.at<double>(0))*180/3.14<<" "<< (Theta_dot.at<double>(1))*180/3.14<<" "<< (Theta_dot.at<double>(2))*180/3.14<<" "<< (Theta_dot.at<double>(3))*180/3.14<<" "<< (Theta_dot.at<double>(4))*180/3.14<<" "<< (Theta_dot.at<double>(5))*180/3.14<<endl;

        control_speed::velocity_control velocity;

        double vel_limit0 = abs(Theta_dot.at<double>(0))*180/3.14;
        double vel_limit1 = abs(Theta_dot.at<double>(1))*180/3.14;
        double vel_limit2 = abs(Theta_dot.at<double>(2))*180/3.14;
        double vel_limit3 = abs(Theta_dot.at<double>(3))*180/3.14;
        double vel_limit4 = abs(Theta_dot.at<double>(4))*180/3.14;
        double vel_limit5 = abs(Theta_dot.at<double>(5))*180/3.14;

        if (vel_limit0 < 8 && vel_limit1 < 8 && vel_limit2 < 8 && vel_limit3 < 8 && vel_limit4 < 8 && vel_limit5 < 8 )

        {
        velocity.vel0.data = Theta_dot.at<double>(0);
        velocity.vel1.data = Theta_dot.at<double>(1);
        velocity.vel2.data = Theta_dot.at<double>(2);
        velocity.vel3.data = Theta_dot.at<double>(5);
        velocity.vel4.data = Theta_dot.at<double>(4);
        velocity.vel5.data = Theta_dot.at<double>(3);
        Theta_dot_prev = Theta_dot;
//        cout<<"Theta_dot_prev"<<Theta_dot_prev<<cout;
        }
        else
        {
            velocity.vel0.data = 0;
            velocity.vel1.data = 0;
            velocity.vel2.data = 0;
            velocity.vel3.data = 0;
            velocity.vel4.data = 0;
            velocity.vel5.data = 0;
//            velocity.vel0.data = Theta_dot_prev.at<double>(0);
//            velocity.vel1.data = Theta_dot_prev.at<double>(1);
//            velocity.vel2.data = Theta_dot_prev.at<double>(2);
//            velocity.vel3.data = Theta_dot_prev.at<double>(5);
//            velocity.vel4.data = Theta_dot_prev.at<double>(4);
//            velocity.vel5.data = Theta_dot_prev.at<double>(3);

        }


        f3 << Theta_dot.at<double>(0) << "\t"<< Theta_dot.at<double>(1) << "\t"<< Theta_dot.at<double>(2) << "\t"<< Theta_dot.at<double>(3) << "\t"<< Theta_dot.at<double>(4) << "\t"<< Theta_dot.at<double>(5) << "\t" << endl;  //saving joint velocities velocities to file

        /*--------sending velocity to control_speed node-------*/

        pb.publish(velocity);
        ros::spinOnce();

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

        //        loop_rate.sleep();
        rate.sleep();
        cout << "sqrt_error" << sqrt(mean_error) << endl;
        if (sqrt(mean_error)<8)
            break;
    }

    return 0;
    f1.close();
    f2.close();
    f3.close();
    f4.close();
    f5.close();
    f6.close();
}


