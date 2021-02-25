#include <iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void matrix_for_ee_to_camera( cv::Mat vector_ec,cv::Mat New_Rot,cv::Mat &mul_Jacob)
{

    cv::Mat S_aec = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat S_aec_mul = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat tNew_Rot = cv::Mat(3, 3, CV_64F, 0.0);

    S_aec.at<double>(0,1) =  - vector_ec.at<double>(0,2);
    S_aec.at<double>(0,2) =    vector_ec.at<double>(0,1);
    S_aec.at<double>(1,0) =    vector_ec.at<double>(0,2);
    S_aec.at<double>(1,2) =  - vector_ec.at<double>(0,0);
    S_aec.at<double>(2,0) =  - vector_ec.at<double>(0,1);
    S_aec.at<double>(2,1) =    vector_ec.at<double>(0,0);



    cv::transpose(New_Rot,tNew_Rot);

    S_aec_mul = -S_aec*tNew_Rot;

    mul_Jacob.at<double>(0,0) =  1;
    mul_Jacob.at<double>(1,1) =  1;
    mul_Jacob.at<double>(2,2) =  1;

    mul_Jacob.at<double>(0,3) =  S_aec_mul.at<double>(0,0); mul_Jacob.at<double>(0,4) = S_aec_mul.at<double>(0,1); mul_Jacob.at<double>(0,5) = S_aec_mul.at<double>(0,2);
    mul_Jacob.at<double>(1,3) =  S_aec_mul.at<double>(1,0); mul_Jacob.at<double>(1,4) = S_aec_mul.at<double>(1,1); mul_Jacob.at<double>(1,5) = S_aec_mul.at<double>(1,2);
    mul_Jacob.at<double>(2,3) =  S_aec_mul.at<double>(2,0); mul_Jacob.at<double>(2,4) = S_aec_mul.at<double>(2,1); mul_Jacob.at<double>(2,5) = S_aec_mul.at<double>(2,2);

    mul_Jacob.at<double>(3,3) = tNew_Rot.at<double>(0,0); mul_Jacob.at<double>(3,4)=tNew_Rot.at<double>(0,1); mul_Jacob.at<double>(3,5)=tNew_Rot.at<double>(0,2);
    mul_Jacob.at<double>(4,3) = tNew_Rot.at<double>(1,0); mul_Jacob.at<double>(4,4)=tNew_Rot.at<double>(1,1); mul_Jacob.at<double>(4,5)=tNew_Rot.at<double>(1,2);
    mul_Jacob.at<double>(5,3) = tNew_Rot.at<double>(2,0); mul_Jacob.at<double>(5,4)=tNew_Rot.at<double>(2,1); mul_Jacob.at<double>(5,5)=tNew_Rot.at<double>(2,2);

}

