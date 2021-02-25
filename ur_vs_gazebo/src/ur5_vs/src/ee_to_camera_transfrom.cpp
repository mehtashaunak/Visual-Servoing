#include <iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void ee_to_camera_transform(cv::Mat &R, cv::Mat &vector_ec){

    int i=0;

            R.at<double>(i,0) = 0.0182029;
            R.at<double>(i,1) =-0.0124297;
            R.at<double>(i,2) = 0.999757;

            R.at<double>(i+1,0) = -0.706979;
            R.at<double>(i+1,1) = -0.707223;
            R.at<double>(i+1,2) = 0.00407954;

            R.at<double>(i+2,0) = 0.707;
            R.at<double>(i+2,1) = -0.706881;
            R.at<double>(i+2,2) = -0.021661;

            vector_ec.at<double>(0,0)=0.080541;
            vector_ec.at<double>(0,1)=0.0494511;
            vector_ec.at<double>(0,2)=0.0631135;

}



