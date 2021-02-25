#include <iostream>
#include<opencv2/opencv.hpp>
//#include"gen_new_node.h"
using namespace std;
using namespace cv;

void compute_new_geo_jacobian(double th[], cv::Mat &J)
{
    double d1,a2,a3,d4,d5,d6;
        d1=0.0895; /*base*/
        a2=0.425; /*upper arm*/
        a3=0.3923; /*lower arm*/
        d4=0.1092; /*wrist 2*/
        d5=0.0947; /*wrist 3*/
        d6=0.0823; /*tool mounting bracket*/

        double th1, th2, th3, th4, th5, th6;
        th1=th[0];
        th2=th[1];
        th3=th[2];
        th4=th[3];
        th5=th[4];
        th6=th[5];
//        th1=-0.4238;
//        th2=-2.2241;
//        th3=1.7404;
//        th4=1.6614;
//        th5=-1.7116;
//        th6=1.05697;


    //Row1
    double j11, j12, j13, j14, j15, j16;

    j11=d5*sin(th2 + th3 + th4)*sin(th1) - d4*cos(th1) - a2*cos(th2)*sin(th1) - d6*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) - a3*cos(th2)*cos(th3)*sin(th1) + a3*sin(th1)*sin(th2)*sin(th3);

    j12=-cos(th1)*(a3*sin(th2 + th3) - d5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) + a2*sin(th2) + d6*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4)));

    j13=-cos(th1)*(a3*sin(th2 + th3) + d5*cos(th2 + th3 + th4) + d6*sin(th2 + th3 + th4)*sin(th5));

    j14=-cos(th1)*(d5*cos(th2 + th3 + th4) + d6*sin(th2 + th3 + th4)*sin(th5));

    j15=d6*sin(th1)*sin(th5) - d6*cos(th1)*cos(th2)*cos(th5)*sin(th3)*sin(th4) - d6*cos(th1)*cos(th3)*cos(th5)*sin(th2)*sin(th4) - d6*cos(th1)*cos(th4)*cos(th5)*sin(th2)*sin(th3) + d6*cos(th1)*cos(th2)*cos(th3)*cos(th4)*cos(th5);

    j16=0;

    int i=0;
    J.at<double>(i,0) = j11;
    J.at<double>(i,1) = j12;
    J.at<double>(i,2) = j13;
    J.at<double>(i,3) = j14;
    J.at<double>(i,4) = j15;
    J.at<double>(i,5) = j16;

    //Row2
    double j21, j22, j23, j24, j25, j26;

    j21=a2*cos(th1)*cos(th2) - d4*sin(th1) - d6*cos(th5)*sin(th1) - d5*sin(th2 + th3 + th4)*cos(th1) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3) + d6*cos(th2 + th3 + th4)*cos(th1)*sin(th5);

    j22=-sin(th1)*(a3*sin(th2 + th3) - d5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) + a2*sin(th2) + d6*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4)));

    j23=-sin(th1)*(a3*sin(th2 + th3) + d5*cos(th2 + th3 + th4) + d6*sin(th2 + th3 + th4)*sin(th5));

    j24=-sin(th1)*(d5*cos(th2 + th3 + th4) + d6*sin(th2 + th3 + th4)*sin(th5));

    j25=d6*cos(th2)*cos(th3)*cos(th4)*cos(th5)*sin(th1) - d6*cos(th1)*sin(th5) - d6*cos(th2)*cos(th5)*sin(th1)*sin(th3)*sin(th4) - d6*cos(th3)*cos(th5)*sin(th1)*sin(th2)*sin(th4) - d6*cos(th4)*cos(th5)*sin(th1)*sin(th2)*sin(th3);

    j26=0;

    //int i=1;
    J.at<double>(i+1,0) = j21;
    J.at<double>(i+1,1) = j22;
    J.at<double>(i+1,2) = j23;
    J.at<double>(i+1,3) = j24;
    J.at<double>(i+1,4) = j25;
    J.at<double>(i+1,5) = j26;

    //Row3
    double j31, j32, j33, j34, j35, j36;

    j31=0;

    j32=(d6*sin(th2 + th3 + th4 - th5))/2 - a3*cos(th2 + th3) - a2*cos(th2) - (d6*sin(th2 + th3 + th4 + th5))/2 + d5*sin(th2 + th3 + th4);

    j33=(d6*sin(th2 + th3 + th4 - th5))/2 - a3*cos(th2 + th3) - (d6*sin(th2 + th3 + th4 + th5))/2 + d5*sin(th2 + th3 + th4);

    j34=(d6*sin(th2 + th3 + th4 - th5))/2 - (d6*sin(th2 + th3 + th4 + th5))/2 + d5*sin(th2 + th3 + th4);

    j35=-d6*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4 + th5)/2);

    j36=0;

    //int i=2;
    J.at<double>(i+2,0) = j31;
    J.at<double>(i+2,1) = j32;
    J.at<double>(i+2,2) = j33;
    J.at<double>(i+2,3) = j34;
    J.at<double>(i+2,4) = j35;
    J.at<double>(i+2,5) = j36;


    //Row4
    double j41, j42, j43, j44, j45, j46;

    j41=0;

    j42=-sin(th1);

    j43=-sin(th1);

    j44=-sin(th1);

    j45=-sin(th2 + th3 + th4)*cos(th1);

    j46=cos(th2 + th3 + th4)*cos(th1)*sin(th5) - cos(th5)*sin(th1);

 //   int i=3;
    J.at<double>(i+3,0) = j41;
    J.at<double>(i+3,1) = j42;
    J.at<double>(i+3,2) = j43;
    J.at<double>(i+3,3) = j44;
    J.at<double>(i+3,4) = j45;
    J.at<double>(i+3,5) = j46;

    //Row5
    double j51, j52, j53, j54, j55, j56;

    j51=0;

    j52=cos(th1);

    j53=cos(th1);

    j54=cos(th1);

    j55=-sin(th2 + th3 + th4)*sin(th1);

    j56=cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5);

//    int i=4;
    J.at<double>(i+4,0) = j51;
    J.at<double>(i+4,1) = j52;
    J.at<double>(i+4,2) = j53;
    J.at<double>(i+4,3) = j54;
    J.at<double>(i+4,4) = j55;
    J.at<double>(i+4,5) = j56;

    //Row6
    double j61, j62, j63, j64, j65, j66;

    j61=1;

    j62=0;

    j63=0;

    j64=0;

    j65=-cos(th2 + th3 + th4);

    j66=-sin(th2 + th3 + th4)*sin(th5);

//    int i=5;
    J.at<double>(i+5,0) = j61;
    J.at<double>(i+5,1) = j62;
    J.at<double>(i+5,2) = j63;
    J.at<double>(i+5,3) = j64;
    J.at<double>(i+5,4) = j65;
    J.at<double>(i+5,5) = j66;


//    cout << j11 << "\t" << j12 << "\t" << j13 << "\t" << j14 << "\t" << j15 << "\t" << j16 <<  endl;

//    cout << j21 << "\t" << j22 << "\t" << j23 << "\t" << j24 << "\t" << j25 << "\t" << j26 <<  endl;

//    cout << j31 << "\t" << j32 << "\t" << j33 << "\t" << j34 << "\t" << j35 << "\t" << j36 <<  endl;

//    cout << j41 << "\t" << j42 << "\t" << j43 <<  "\t" << j44 << "\t" << j45 << "\t" << j46 <<  endl;

//    cout << j51 << "\t" << j52 << "\t" << j53 << "\t" << j54 <<"\t" << j55 << "\t" << j56 <<  endl;

//    cout << j61 << "\t" << j62 << "\t" << j63 << "\t" << j64 << "\t" << j65 << "\t" << j66 <<  endl;

}

