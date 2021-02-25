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


double x_point, y_point, area_img_current,area_img_current_prev,area_img_desired,area_img_current_temp;
float centroid_depth;
double points_corner[8],points_corner_desired[8];

void img_proc_callback(const new_detection::points_four_image_data  msg)
{
    x_point = 0;
    y_point = 0;
    centroid_depth = 0.0;
    points_corner[0] = 0;
    points_corner[1] = 0;
    points_corner[2] = 0;
    points_corner[3] = 0;
    points_corner[4] = 0;
    points_corner[5] = 0;
    points_corner[6] = 0;
    points_corner[7] = 0;
    int i;
    for (i=0;i<100;i++)
    {
        x_point = x_point + msg.detected_point_x.data;
        y_point = y_point+ msg.detected_point_y.data;
        centroid_depth = centroid_depth + msg.detected_point_depth.data;
        points_corner[0] = points_corner[0]+(msg.detected_point_0.data-320)/(531.15/640);
        points_corner[1] = points_corner[1]+(msg.detected_point_1.data-240)/(531.15/480);
        points_corner[2] = points_corner[2]+(msg.detected_point_2.data-320)/(531.15/640);
        points_corner[3] = points_corner[3]+(msg.detected_point_3.data-240)/(531.15/480);
        points_corner[4] = points_corner[4]+(msg.detected_point_4.data-320)/(531.15/640);
        points_corner[5] = points_corner[5]+(msg.detected_point_5.data-240)/(531.15/480);
        points_corner[6] = points_corner[6]+(msg.detected_point_6.data-320)/(531.15/640);
        points_corner[7] = points_corner[7]+(msg.detected_point_7.data-240)/(531.15/480);

    }
    x_point = x_point/(i);
    y_point = y_point/(i);
    centroid_depth = centroid_depth/(i);
    points_corner[0]=points_corner[0]/(i);
    points_corner[1]=points_corner[1]/(i);
    points_corner[2]=points_corner[2]/(i);
    points_corner[3]=points_corner[3]/(i);
    points_corner[4]=points_corner[4]/(i);
    points_corner[5]=points_corner[5]/(i);
    points_corner[6]=points_corner[6]/(i);
    points_corner[7]=points_corner[7]/(i);
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

    /*--------for secondary task---------*/
    cv::Mat Jsec = cv::Mat(6,6, CV_64F, 0.0);
    cv::Mat Jsect = cv::Mat(6,6, CV_64F, 0.0);
    cv::Mat Jsec_converted = cv::Mat(1,6, CV_64F, 0.0);
    cv::Mat In = cv::Mat(6,6, CV_64F, 0.0);
    cv::Mat Vsec = cv::Mat(6,1, CV_64F, 0.0);
    cv::Mat sec_term = cv::Mat(1,6, CV_64F, 0.0);
    cv::Mat sec_term_inv = cv::Mat(6,1, CV_64F, 0.0);
    cv::Mat sec_task = cv::Mat(6,1, CV_64F, 0.0);

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
    double eta = 10000; //60000;
    double depth_p[3]={0,0,0},prev_depth[3], u_t[2];
    double m[3]={0,0,0};
    double m_t[3]={0,0,0};

    /*----------------------------------------------getting image -----------------------------------------*/

    ros::Subscriber img_proc = nh.subscribe< new_detection::points_four_image_data>("/object_points", 10, img_proc_callback);
    sleep(1);
    ros::spinOnce();


    /*-------------------Desired points---------------------------*/
    //    u_t[0] = 320;
    //    u_t[1] = 240;
    //    area_img_desired = 17805;

    //    u_t[0] = 252;
    //    u_t[1] = 168;
    ////    area_img_desired = 13724;

    //    area_img_desired = 15000;

    //    points_corner_desired[0]=  (172.05-320)/(531.15/640);
    //    points_corner_desired[1] = (201.27-240)/(531.15/480);
    //    points_corner_desired[2] = (169.81-320)/(531.15/640);
    //    points_corner_desired[3] = (145.27-240)/(531.15/480);
    //    points_corner_desired[4] = (332.13-320)/(531.15/640);
    //    points_corner_desired[5] = (137.82-240)/(531.15/480);
    //    points_corner_desired[6] = (335.80-320)/(531.15/640);
    //    points_corner_desired[7] = (191.65-240)/(531.15/480);


    //    u_t[0] = 250;
    //    u_t[1] = 80;
    ////    area_img_desired = 13724;

    //    area_img_desired = 22721;

    //    u_t[0] = 320;
    //    u_t[1] = 240;
    //    //    area_img_desired = 13724;

    //    area_img_desired = 19107;

    //    points_corner_desired[0]=  (430.98-320)/(531.15/640);
    //    points_corner_desired[1] = (279.49-240)/(531.15/480);
    //    points_corner_desired[2] = (211.86-320)/(531.15/640);
    //    points_corner_desired[3] = (276.97-240)/(531.15/480);
    //    points_corner_desired[4] = (212.77-320)/(531.15/640);
    //    points_corner_desired[5] = (197.90-240)/(531.15/480);
    //    points_corner_desired[6] = (431.89-320)/(531.15/640);
    //    points_corner_desired[7] = (200.42-240)/(531.15/480);

    //    u_t[0] = 320;
    //    u_t[1] = 240;
    //    area_img_desired = 15000;

    //    points_corner_desired[0]=  (430.89-320)/(531.15/640);
    //    points_corner_desired[1] = (279.53-240)/(531.15/480);
    //    points_corner_desired[2] = (246.59-320)/(531.15/640);
    //    points_corner_desired[3] = (271.86-240)/(531.15/480);
    //    points_corner_desired[4] = (249.68-320)/(531.15/640);
    //    points_corner_desired[5] = (197.57-240)/(531.15/480);
    //    points_corner_desired[6] = (433.99-320)/(531.15/640);
    //    points_corner_desired[7] = (205.25-240)/(531.15/480);


    u_t[0] = 320;
    u_t[1] = 240;
    area_img_desired = 3000;

    points_corner_desired[0]=  (295.54-320)/(531.15/640);
    points_corner_desired[1] = (269.57-240)/(531.15/480);
    points_corner_desired[2] = (285.17-320)/(531.15/640);
    points_corner_desired[3] = (225.49-240)/(531.15/480);
    points_corner_desired[4] = (346.21-320)/(531.15/640);
    points_corner_desired[5] = (211.13-240)/(531.15/480);
    points_corner_desired[6] = (356.58-320)/(531.15/640);
    points_corner_desired[7] = (255.22-240)/(531.15/480);

    float centroid_depth_des = 0.471;

    /*-------------------------------------------------------------*/

    depth_p[0]=(double)centroid_depth;
    depth_p[1]=(double)centroid_depth;
    depth_p[2]=(double)centroid_depth;

    for (int p=0;p<3;p++)
    {
        prev_depth[p] = depth_p[p];
    }

    string  files_location= "/home/mithun/exp_ibvs_area_sectask/";

    ofstream f1((files_location+"feature_executed.txt").c_str(),std::ios_base::trunc);
    ofstream f2((files_location+"joint_angles.txt").c_str(), std::ios_base::trunc);
    ofstream f3((files_location+"joint_velocities.txt").c_str(), std::ios_base::trunc);
    ofstream f4((files_location+"camera velocities.txt").c_str(), std::ios_base::trunc);
    ofstream f5((files_location+"feature_error.txt").c_str(), std::ios_base::trunc);
    ofstream f6((files_location+"dataset.txt").c_str(), std::ios_base::trunc);

    /*---------------------------------------iteration starts here--------------------------------------*/
    ros::Rate rate(180);
    int serial_no = 0;
    //    area_img_current_prev = 18825;
    while(ros::ok())
    {
        ros::spinOnce();
        //current points
        m[0] = (x_point-320)/(531.15/640);    //(u-u0)/px
        m[1] = (y_point-240)/(531.15/480);    //(v-v0)/py50
        double xc[5]={points_corner[0],points_corner[2],points_corner[4],points_corner[6],points_corner[0]};
        double yc[5]={points_corner[1],points_corner[3],points_corner[5],points_corner[7],points_corner[1]};

        double area_current = (xc[0] * yc[1] - xc[1] * yc[0] + xc[1] * yc[2] - xc[2] * yc[1] + xc[2] * yc[3] - xc[3] * yc[2] + xc[3] * yc[4] - xc[4] * yc[3] + xc[4] * yc[5] - xc[5] * yc[4])*0.5;
        m[2] = area_current;

        //desired points
        m_t[0] = (u_t[0]-320)/(531.15/640);    //(u-u0)/px
        m_t[1] = (u_t[1]-240)/(531.15/480);    //(v-v0)/py
        m_t[2] =  area_img_desired;

        //depth
        //float current_depth=centroid_depth; //for current
        float current_depth=centroid_depth_des;  //for desired
        depth_p[0]=current_depth;
        depth_p[1]=current_depth;
        depth_p[2]=current_depth;

        /*------Computing image jacobian--------*/

        new_area_L_matrix(m_t, points_corner_desired, depth_p, area_img_current_temp, L);//desired
//        new_area_L_matrix(m, points_corner, depth_p, area_img_current_temp, L);//current

        //to remove fluctuation in area
        double area_bound;
        area_bound = area_img_current_prev - area_current;

        if (area_current < 10)
        {
            m[2] = area_img_current_prev;
        }
        else
        {
            m[2] = area_current;
            area_img_current_prev = area_current;
        }


        //writing current points to file in pixel coordinates
        f6 << serial_no << "\t" << x_point << "\t" << y_point << "\t" << points_corner[0] << "\t" << points_corner[1]<< "\t" << points_corner[2]<< "\t" << points_corner[3]<< "\t" << points_corner[4] << "\t" << points_corner[5]<< "\t" << points_corner[6]<< "\t" << points_corner[7] << "\t" << area_img_current << endl;

        //writing current features to file
        f1 << m[0] << "\t"<< m[1] << "\t"<< m[2] << "\t" << endl;

        //calculating error
        double mean_error = 0;
        for(int i = 0; i < 3; i++)
        {
            Error_p.at<double>(i,0) = (m[i]-m_t[i]);
        }

        //mean error x^2 + y^2 + A^2
        for(int i = 0; i < 3; i++)
        {
            mean_error = mean_error + Error_p.at<double>(i,0)*Error_p.at<double>(i,0);
        }

        //writing error to file
        f5 << Error_p.at<double>(0,0) << "\t"<< Error_p.at<double>(1,0) << "\t"<< Error_p.at<double>(2,0)<< "\t" << sqrt(mean_error) << endl;  //saving features to file

        cout << " error_p " << Error_p.at<double>(0,0) << "\t"<< Error_p.at<double>(1,0) << "\t"<< Error_p.at<double>(2,0)<< "\t" << sqrt(mean_error) << endl;  //saving features to file

        cv::invert(L, Linv, cv::DECOMP_SVD);

        /*-----------------norm error---------------*/

        double n1 = pow(Error_p.at<double>(0,0),2);
        double n2 = pow(Error_p.at<double>(1,0),2);
        double n3 = pow(Error_p.at<double>(2,0),2);

        double n = sqrt(n1+n2+n3);

        double e1_norm = Error_p.at<double>(0,0)/n;
        double e2_norm = Error_p.at<double>(1,0)/n;
        double e3_norm = Error_p.at<double>(2,0)/n;

        Error_p.at<double>(0,0)=e1_norm;
        Error_p.at<double>(1,0)=e2_norm;
        Error_p.at<double>(2,0)=e3_norm;

        /*------------------ibvs controller------------------*/

        V = -eta * Linv * Error_p;

        cout << " " << endl;
        cout << "---------------------------------------------------------------------" << endl;
        cout << "current feature " << m[0] << "\t" << m[1] << "\t" << m[2] << endl;
        cout << "desired feature " << m_t[0] << "\t" << m_t[1] << "\t" << m_t[2] << endl;
        cout << "error_p_norm " << Error_p.at<double>(0,0) << " " << Error_p.at<double>(1,0) << " " << Error_p.at<double>(2,0) << " " <<  endl;
        cout << "sqrt error " << sqrt(mean_error) << endl;
        cout << "camera velocity" << V << endl;
        cout << "---------------------------------------------------------------------" << endl;
        cout << " " << endl;

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

//        cout << "Rot " << Rot << endl;

//        cout << "V1" << V1 << endl;

//        cout << "V2" << V2 << endl;

        V11=Rot*V1;
        V22=Rot*V2;

        /*-------------obtaining matrix to specify camera velocity interms of ee velocity. (multiplying J with  mul_Jacob transformation matrix)-------------------*/
        matrix_for_ee_to_camera(vector_ec,New_Rot,mul_Jacob);


        /*-------Obtaining robot Jacobian------------*/

        /*------moveit jacobian-------------*/

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

//        cout << "Jg " << Jm  << endl;
//        cout << "mul_Jacob " << mul_Jacob  << endl;

//        cout << "Final Jacobian " << new_Jacob  << endl;
        /*-----------------------------------------------------------------*/

        cv::invert(new_Jacob, Jginv, cv::DECOMP_SVD);

        /*---------Obtaining joint velocities-----------*/
        VV.at<double>(0,0)=V11.at<double>(0,0);
        VV.at<double>(1,0)=V11.at<double>(1,0);
        VV.at<double>(2,0)=V11.at<double>(2,0);
        VV.at<double>(3,0)=V22.at<double>(0,0);
        VV.at<double>(4,0)=V22.at<double>(1,0);
        VV.at<double>(5,0)=V22.at<double>(2,0);

//        cout << "base vel " << VV << endl;
        f4 << VV.at<double>(0,0) << "\t"<< VV.at<double>(1,0) << "\t"<< VV.at<double>(2,0) << "\t"<< VV.at<double>(3,0) << "\t"<< VV.at<double>(4,0) << "\t"<< VV.at<double>(5,0) << "\t" << endl;  //saving camera velocities to file

        Theta_dot = Jginv * VV;

        /*-----------------for secondary task----------------------------*/
        //Jsec.at<double>(4,4) = 1.0;

        Jsec.at<double>(3,3) = 1.0;
        Jsec.at<double>(4,4) = 1.0;
        Jsec.at<double>(5,5) = 1.0;

        In.at<double>(0,0) = 1.0;
        In.at<double>(1,1) = 1.0;
        In.at<double>(2,2) = 1.0;
        In.at<double>(3,3) = 1.0;
        In.at<double>(4,4) = 1.0;
        In.at<double>(5,5) = 1.0;

        sec_term = Jsec*(In-(Jginv*new_Jacob));

        cv::invert(sec_term, sec_term_inv, cv::DECOMP_SVD);

        //Vsec.at<double>(0,0) = 0;

        Vsec.at<double>(3,0) = 0;
        Vsec.at<double>(4,0) = 0;
        Vsec.at<double>(5,0) = 0;

        //Vsec.at<double>(0,0) = abs(Error_p.at<double>(0,0))/100;

        //positive velocty to second joint
        //Vsec.at<double>(0,0) = abs(Theta_dot.at<double>(1,0));

        //Vsec.at<double>(0,0) = abs(Error_p.at<double>(0,0))/100000;
        cout << " " << endl;
        cout<< " Secondary task = " << Vsec.at<double>(0,0) << endl;
        cout << " " << endl;

        sec_task = (sec_term_inv * (Vsec - (Jsec*Jginv*VV)));

        Theta_dot = (Jginv * VV) + (In-(Jginv*new_Jacob))*sec_task;

//                Theta_dot = (Jginv * VV) + sec_task;
        /*--------------------------------------------------------------------*/

        cout<< "joint_velocity = " <<" "<< (Theta_dot.at<double>(0))*180/3.14<<" "<< (Theta_dot.at<double>(1))*180/3.14<<" "<< (Theta_dot.at<double>(2))*180/3.14<<" "<< (Theta_dot.at<double>(3))*180/3.14<<" "<< (Theta_dot.at<double>(4))*180/3.14<<" "<< (Theta_dot.at<double>(5))*180/3.14<<endl;
        cout << " " << endl;

        cout<< "joint_velocity (rad)= " <<" "<< (Theta_dot.at<double>(0))<<" "<< (Theta_dot.at<double>(1))<<" "<< (Theta_dot.at<double>(2))<<" "<< (Theta_dot.at<double>(3))<<" "<< (Theta_dot.at<double>(4))<<" "<< (Theta_dot.at<double>(5))<<endl;

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

            //            velocity.vel0.data = Theta_dot.at<double>(0);
            //            velocity.vel1.data = Theta_dot.at<double>(1);
            //            velocity.vel2.data = Theta_dot.at<double>(2);
            //            velocity.vel3.data = Theta_dot.at<double>(5);
            //            velocity.vel4.data = Theta_dot.at<double>(4);
            //            velocity.vel5.data = 0;

            //        velocity.vel0.data = Theta_dot.at<double>(0);
            //        velocity.vel1.data = Theta_dot.at<double>(1);
            //        velocity.vel2.data = Theta_dot.at<double>(2);
            //        velocity.vel3.data = Theta_dot.at<double>(3);
            //        velocity.vel4.data = Theta_dot.at<double>(4);
            //        velocity.vel5.data = Theta_dot.at<double>(5);
            Theta_dot_prev = Theta_dot;
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

        //writing joint vel to file
        f3 << Theta_dot.at<double>(0) << "\t"<< Theta_dot.at<double>(1) << "\t"<< Theta_dot.at<double>(2) << "\t"<< Theta_dot.at<double>(3) << "\t"<< Theta_dot.at<double>(4) << "\t"<< Theta_dot.at<double>(5) << "\t" << endl;  //saving joint velocities velocities to file

        /*--------sending velocity to control_speed node-------*/

        pb.publish(velocity);
        ros::spinOnce();

        if (isnan(centroid_depth))
        {

            depth_p[0]=prev_depth[0];
            depth_p[1]=prev_depth[1];
            depth_p[2]=prev_depth[2];

        }
        else

        {
            depth_p[0]=(double)centroid_depth;
            depth_p[1]=(double)centroid_depth;
            depth_p[2]=(double)centroid_depth;

        }

        for (int p=0;p<3;p++)
        {
            prev_depth[p] = depth_p[p];
        }

        rate.sleep();

        if (sqrt(mean_error)<10)
        {
            break;
        }
    }

    return 0;
    f1.close();
    f2.close();
    f3.close();
    f4.close();
    f5.close();
    f6.close();
}


