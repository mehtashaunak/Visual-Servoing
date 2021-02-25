#include <iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void compute_image_jacobian_four_points(double u[], double z[], cv::Mat &L)
{
//    if(z == 0)
//        z = z + 0.001;

    double lambda=531.15; //531.15
    for (int i = 0; i < 8 ; i = i+2)
    {
//    int i=0;
        L.at<double>(i,0) = -lambda / z[0];
        L.at<double>(i,1) = 0.0;
        L.at<double>(i,2) = u[i] / z[0];
        L.at<double>(i,3) = (u[i] * u[i+1]) / lambda;
        L.at<double>(i,4) = -((lambda * lambda) + (u[i] * u[i])) / lambda;
        L.at<double>(i,5) = u[i+1];

        L.at<double>(i+1,0) = 0;
        L.at<double>(i+1,1) = -lambda / z[0];
        L.at<double>(i+1,2) = u[i+1] / z[0];
        L.at<double>(i+1,3) = -((lambda * lambda) + (u[i+1] * u[i+1])) / lambda;
        L.at<double>(i+1,4) = (-u[i] * u[i+1]) / lambda;
        L.at<double>(i+1,5) = -u[i];
//    L.at<double>(i,0) = -lambda / z[0];
//    L.at<double>(i,1) = 0.0;
//    L.at<double>(i,2) = u[i] / z[0];
//    L.at<double>(i,3) = u[i+1];
//    L.at<double>(i,4) = -((lambda * lambda) + (u[i] * u[i])) / lambda;
//    L.at<double>(i,5) = (u[i] * u[i+1]) / lambda;


//    L.at<double>(i+1,0) = 0;
//    L.at<double>(i+1,1) = -lambda / z[0];
//    L.at<double>(i+1,2) = u[i+1] / z[0];
//    L.at<double>(i+1,3) = -u[i];
//    L.at<double>(i+1,4) = (-u[i] * u[i+1]) / lambda;
//    L.at<double>(i+1,5) = -((lambda * lambda) + (u[i+1] * u[i+1])) / lambda;

    }
}
