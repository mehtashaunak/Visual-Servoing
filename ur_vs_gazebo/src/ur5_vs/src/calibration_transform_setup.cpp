#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <kdl/frames.hpp>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#define PI 3.14159265
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibrated_tf_publisher");
    ros::NodeHandle n;
    //    ros::Rate r(100);

    tf::TransformBroadcaster T_cl_ee;


    //    // ensenso rotated 45 with mount at eef for gripper design 1
    //    tf::Matrix3x3 R_ee_cl(0.0903809,  0.190623, 0.977494,
    //                          -0.690086, -0.695695, 0.199475,
    //                           0.718062, -0.692584, 0.0686686);

    // in iit
    //    sensor to rbt
    //     0.0711087   0.219396   0.973041  0.0396188
    //     -0.997351 0.00064995  0.0727387  0.0114124
    //     0.0153261  -0.975636   0.218861   0.160689
    //             0          0          0          1
    //    rbt to sensor
    //     0.0711087  -0.997351  0.0153261 0.00610222
    //      0.219396 0.00064994  -0.975636   0.148074
    //      0.973041  0.0727386   0.218861 -0.0745494
    //             0          0          0          1
    // in japan
    //    sensor to rbt
    //     0.0756697   0.217902   0.973033  0.0373014
    //     -0.997109 0.00977505   0.075353 0.00166302
    //    0.00690796  -0.975922   0.218012    0.15264
    //             0          0          0          1
    //    rbt to sensor
    //      0.0756697   -0.997109  0.00690768 -0.00221878
    //       0.217902  0.00977512   -0.975922     0.14082
    //       0.973033   0.0753528    0.218012  -0.0696987
    //              0           0           0           1
    // ensenso 0 degrees without mount at eef for gripper design 2
    //    tf::Matrix3x3 R_ee_cl(0.0756697,   0.217902,   0.973033,
    //                          -0.997109, 0.00977505,   0.075353,
    //                         0.00690796,  -0.975922,   0.218012);
    //

    // ========================================
    //tf::Matrix3x3 R_ee_cl(0.00161802,0.258345,0.966051,-0.999994,-0.00277869,0.002418,0.00330902,-0.966049,0.258339);

    //tf::Matrix3x3 R_ee_cl(-0.0241,-0.7090,-0.7047,0.999,-0.0284,-0.0055,-0.0160,-0.7045,0.7094);
    //tf::Matrix3x3 R_ee_cl(0.0182029,-0.0124297,0.999757, -0.706979 , -0.707223, 0.00407954,0.707 , -0.706881 , -0.021661);

    //tf::Matrix3x3 R_ee_cl(-0.0113449,-0.0296784,0.999495, -0.682252 , -0.730524, -0.0294357 ,0.731029 , -0.682241 , -0.0119604);
    tf::Matrix3x3 R_ee_cl(-0.0159867, -0.194668, 0.980739, -0.999775, 0.0168417, -0.0129541, -0.0139955, -0.980725, -0.194893   );


    // =============================calibration new matrix -16 -feb===========

    //-0.0113449 -0.0296784   0.999495  0.0692818
    // -0.682252  -0.730524 -0.0294357  0.0468184
    //  0.731029  -0.682241 -0.0119604   0.048408
    //         0          0          0          1

    // ========================================
    while(n.ok())
    {
        // ensenso rotated 45 with mount at eef for gripper design 1
        //        T_cl_ee.sendTransform(
        //                    tf::StampedTransform(
        //                        tf::Transform(R_ee_cl, tf::Vector3 (0.0432301, 0.115773, 0.122649)),
        //                        ros::Time::now(), "/ee_link", "/camera_link"));

        // ensenso 0 degrees without mount at eef for gripper design 2
        //        T_cl_ee.sendTransform(
        //                  tf::StampedTransform(
        //                       tf::Transform(R_ee_cl, tf::Vector3 (0.0590367,0.0557851,0.0944287)),
        //                     ros::Time::now(), "/ee_link", "/camera_link"));



        // tf::Transform(R_ee_cl, tf::Vector3 (0.080541,0.0494511,0.0631135)), old one

        //T_cl_ee.sendTransform( tf::StampedTransform( tf::Transform(R_ee_cl, tf::Vector3 (0.0692818,0.0468184, 0.048408)),
        T_cl_ee.sendTransform( tf::StampedTransform(tf::Transform(R_ee_cl, tf::Vector3 (-0.00277013, 0.0131105, 0.067404)),

                                                    ros::Time::now(), "/ee_link", "/camera_link"));

        //        r.sleep();
        usleep(1);
    }



    return 0;

}

