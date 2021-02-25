
// RRT Planning in image space for vision-based control
// Written by Deepak Raina (EMP ID: 1445893)

/* INPUT:
 * desired and current features (s,sd),
 * current robot configuration (q,dq),
 * number of random images (n_img) taken from workspace.
*/

/* OUTPUT:
 * trajectory in image space
 * trajectory in configuration space
*/

#include <iostream>
#include<cstdlib>
#include<sstream>
#include<string>
#include<cmath>
#include<vector>
#include<fstream>

//#include "gen_new_node.h"
#include "my_funcs.h"

#include <stdlib.h>
#include <gnuplot_ci.h>
#include <ur5_control/UR5Goal.h>


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

using namespace Eigen;
using namespace std;
using namespace cv;
using namespace gnuplot_ci;

#define DEG2RAD(x) M_PI*x/180.0

//image callback
double x_point, y_point;
float centroid_depth;
double points_corner[8];


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
    for (i=0;i<500;i++)
    {

        //    x_point = msg.detected_point_x.data;
        //    y_point = msg.detected_point_y.data;
        //    centroid_depth = msg.detected_point_depth.data;
        ////    area_img_current = msg.area.data;
        //    points_corner[0]=  (msg.detected_point_0.data-320)/(531.15/640);
        //    points_corner[1] = (msg.detected_point_1.data-240)/(531.15/480);
        //    points_corner[2] = (msg.detected_point_2.data-320)/(531.15/640);
        //    points_corner[3] = (msg.detected_point_3.data-240)/(531.15/480);
        //    points_corner[4] = (msg.detected_point_4.data-320)/(531.15/640);
        //    points_corner[5] = (msg.detected_point_5.data-240)/(531.15/480);
        //    points_corner[6] = (msg.detected_point_6.data-320)/(531.15/640);
        //    points_corner[7] = (msg.detected_point_7.data-240)/(531.15/480);

        x_point = x_point + msg.detected_point_x.data;
        y_point = y_point+ msg.detected_point_y.data;
        centroid_depth = centroid_depth + msg.detected_point_depth.data;
        //    area_img_current = msg.area.data;
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

int main (int argc, char** argv)
{
    ros::init(argc,argv,"rrt_planning");

    ros::NodeHandle nh, mh;
    cv::startWindowThread();

    moveit::planning_interface::MoveGroup arm_group("manipulator");
    moveit::planning_interface::MoveGroup::Plan my_plan;

    arm_group.setPlanningTime(25);// set the maximum planning time

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    ros::Subscriber img_proc = nh.subscribe< new_detection::points_four_image_data>("/object_points", 10, img_proc_callback);
    sleep(1);

    ofstream f1("feature_executed.txt", std::ios_base::trunc);
    ofstream f2("joint_angles_executed.txt", std::ios_base::trunc);

    //reading text files of Joint Trajectory
    int rows=38, cols=6;
    string filename="tree2back.txt";
    cv::Mat traj = cv::Mat(rows, cols, CV_64F, 0.0);
    read_text_file(filename,rows,cols,traj);
    cout << "traj " << traj << endl;
    //    getchar();

    double q[6]={0,0,0,0,0,0};
    double new_node_tree2[6]={0,0,0,0,0,0};

    int k=rows-3;
    int aa=0;
    while(ros::ok())
    {

        //just to remove initial qin which comes zero zero
        if (aa=0){
            ros::spinOnce();
            std::vector<double> current_angles;
            current_angles = arm_group.getCurrentJointValues();

            std::vector<double> joint_values;

            kinematic_state->setJointGroupPositions(joint_model_group,current_angles);

            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            for(std::size_t i = 0; i < joint_names.size(); ++i)
            {
                //                    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }

            q[0]=joint_values[0];
            q[1]=joint_values[1];
            q[2]=joint_values[2];
            q[3]=joint_values[3];
            q[4]=joint_values[4];
            q[5]=joint_values[5];

            cout << "qin00 " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5]  << endl;
            aa=1;
        }
        else
        {
            /*-----------------------move robot to qnew--------------------------*/
            cout << "-------------------------------k="<<k<<"-------------------------" << endl;
            new_node_tree2[0]=traj.at<double>(k,0);
            new_node_tree2[1]=traj.at<double>(k,1);
            new_node_tree2[2]=traj.at<double>(k,2);
            new_node_tree2[3]=traj.at<double>(k,3);
            new_node_tree2[4]=traj.at<double>(k,4);
            new_node_tree2[5]=traj.at<double>(k,5);

            vector<double> goal;

            goal.push_back(new_node_tree2[0]);
            goal.push_back(new_node_tree2[1]);
            goal.push_back(new_node_tree2[2]);
            goal.push_back(new_node_tree2[5]);
            goal.push_back(new_node_tree2[4]);
            goal.push_back(new_node_tree2[3]);

            cout << "goal [" << goal[0] << "\t" << goal[1] << "\t" << goal[2] << "\t" << goal[3] << "\t" << goal[4] << "\t" << goal[5] << " ]" << endl;

//            getchar();
            for (int i=0; i<5; i++)
            {
                arm_group.setJointValueTarget(goal);
                //arm_group.move();
                arm_group.asyncMove();
                sleep(1);
            }
            //                    sleep(2);
            //getchar();

            /*-----------------------take new image--------------------------*/

            ros::spinOnce();

            cout<< "Snew (pixel) " << x_point << " " << y_point << endl;

            double newx=x_point;
            double newy=y_point;
            double centroid[2];
            //                    centroid[0]= (x_point-320)/(531.15/640);
            //                    centroid[1]= (y_point-240)/(531.15/480);

            centroid[0]= (newx-320)/(531.15/640);
            centroid[1]= (newy-240)/(531.15/480);


            //for new area calculation
            double x[5]={points_corner[0],points_corner[2],points_corner[4],points_corner[6],points_corner[0]};
            double y[5]={points_corner[1],points_corner[3],points_corner[5],points_corner[7],points_corner[1]};

            double area_new = (x[0] * y[1] - x[1] * y[0] + x[1] * y[2] - x[2] * y[1] + x[2] * y[3] - x[3] * y[2] + x[3] * y[4] - x[4] * y[3] + x[4] * y[5] - x[5] * y[4])*0.5;

            cout << "Snew (image) " << centroid[0] << " " << centroid[1] <<  " " << area_new << endl;

            f1 << centroid[0] << "\t" << centroid[1] <<  "\t" << area_new << endl;

            ros::spinOnce();

            std::vector<double> current_angles;
            current_angles = arm_group.getCurrentJointValues();

            std::vector<double> joint_values;

            kinematic_state->setJointGroupPositions(joint_model_group,current_angles);

            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            for(std::size_t i = 0; i < joint_names.size(); ++i)
            {
                //                    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }

            q[0]=joint_values[0];
            q[1]=joint_values[1];
            q[2]=joint_values[2];
            q[3]=joint_values[3];
            q[4]=joint_values[4];
            q[5]=joint_values[5];

            cout << "q reached " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5]  << endl;
            f2 << q[0] << "\t" << q[1] << "\t" << q[2] << "\t" << q[5] << "\t" << q[4] << "\t" << q[3]  << endl;
            cout << "------------------------------------------------------------" << endl;

        }

        k--;
        if (k<0)
            break;
    }

    f1.close();
    f2.close();

    cout << "Trajectory executed" << endl;
    cout << "Plotting" << endl;

    //Plot of Tree1 in image space
    GP_handle G1("/usr/bin/", "X (m)", "Y (m)");
    G1.gnuplot_cmd("set terminal wxt");
    G1.gnuplot_cmd("set xrange [-320:320]");
    G1.gnuplot_cmd("set yrange [-240:240]");
    G1.gnuplot_cmd("plot 'tree1back.txt'  u 1:2 w p pt 0 t 'Planned Trajectory', 'feature_executed.txt'  u 1:2 w p pt 0 t 'Executed Trajectory'");
    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
    G1.gnuplot_cmd("set output 'traj_compare.eps'");
    G1.gnuplot_cmd("replot");

    cout << "Press enter to save plots" << endl;
    getchar();

    return 0;
}

