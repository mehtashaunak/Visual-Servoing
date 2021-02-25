#include <iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void compute_image_jacobian_area(double u[], double z[], cv::Mat &L)
{
    int i=0;
    if(z[i] == 0)
        z[i] = z[i] + 0.001;
        double lambda=1;//531.15;

   // for (int i = 0; i < 1 ; i = i+2) {

        // Calculate interaction matrix for area
  //      cv::Mat P1 = cv::Mat(3, 1, CV_64F, 0.0);
 //       cv::Mat P2 = cv::Mat(3, 1, CV_64F, 0.0);
   //     cv::Mat P3 = cv::Mat(3, 1, CV_64F, 0.0);
double P1[3], P2[3], P3[3],vector1[3], vector2[3], normal[3];
        P1[0] = u[i];
        P1[1] = u[i+1];
        P1[2] = z[i];
        P2[0] = u[i+2];
        P2[1] = u[i+3];
        P2[2] = z[i+1];
        P3[0] = u[i+4];
        P3[1] = u[i+5];
        P3[2] = z[i+2];
   //     cout << " P1:"<< P1[0] <<" " << P1[1] << " "<<P1[2] << endl;
     //   cout << "P2: "<< P2[0] <<" "<< P2[1] <<" " <<P2[2] << endl;
       // cout << P3[0] <<" "<< P3[1]<<" " <<P3[2] << endl;

        vector1[0] = P1[0] - P2[0];
        vector1[1] = P1[1] - P2[1];
        vector1[2] = P1[2] - P2[2];
        vector2[0] = P1[0] - P3[0];
        vector2[1] = P1[1] - P3[1];
        vector2[2] = P1[2] - P3[2];
       // double Product[3];
             //Cross product formula
        normal[0] = (vector1[1] * vector2[2]) - (vector1[2] * vector2[1]);
        normal[1] = (vector1[2] * vector2[0]) - (vector1[0] * vector2[2]);
        normal[2] = (vector1[0] * vector2[1]) - (vector1[1] * vector2[0]);

  //      cout<< "normal:" << normal[0] << "  "<<normal[1] << "  " << normal[2] << endl;
       // double *Normal = CrossProduct(vector1,vector2);
     //   CrossProduct(vector1, vector2, Product);
   //     double d=normal[0]* (-P1[0]) + normal[1]* (-P1[1]) +normal[2]* (-P1[2]);
   //     double a=normal[0] * (-1/normal[2]);
   //     double b=normal[1]*(-1/normal[2]);
   //     double c=d*(-1/normal[2]);
   //     double A=-a/c;
    //    double B=-b/c;
     //   double C=1/c;

        double A= normal[0];
        double B=  normal[1];
        double C= normal[2];

  //      cout << "A:" << A <<"B:" << B << "C:" << C <<endl;

        // calculate moments using opencv function
        std::vector<cv::Point> points(4);
        cv::Moments mom;
        points[0].x = u[0];
        points[0].y = u[1];
        points[1].x = u[2];
        points[1].y = u[3];
        points[2].x = u[4];
        points[2].y = u[5];
        points[3].x = u[6];
        points[3].y = u[7];
        mom = cv::moments(points, false);

        // calculate moments using mithuns matlab code
        double sum1=0;
        double sum2=0;
        double sum3=0;
        double sum4=0;
        double sum5=0;
        double sum6=0;
        int n = 4;
        double x[5] = {u[0], u[2], u[4], u[6], u[0]};
        double y[5] = {u[1], u[3], u[5], u[7], u[1]};
   //  %-----------  calculation---------------------------%
         for (int j=0; j < n; j++){
             sum1 = sum1 + ( x[j] * y[j+1] - x[j+1] * y[j]);    // % a calculation
             sum2 = sum2 + (( x[j]*y[j+1] - x[j+1]*y[j] ) * ( x[j] + x[j+1] ) );  //% alp_10 calc
             sum3 = sum3 + (( x[j] *y[j+1] - x[j+1] * y[j] ) * ( y[j] + y[j+1] ) );   //% alp_01 calc
             sum4 = sum4 + (( x[j]*y[j+1] - x[j+1]*y[j] ) * ( (2*x[j]*y[j]) + (x[j]*y[j+1]) + (x[j+1]*y[j]) + (2*x[j+1]*y[j+1] )) );   //% alp_11 calc
             sum5 = sum5 + (( x[j]*y[j+1] - x[j+1]*y[j] ) * ( x[j] * x[j] + (x[j]*x[j+1]) + x[j+1] * x[j+1] ) ); // /% alp_20 calc
             sum6 = sum6 + (( x[j]*y[j+1] - x[j+1]*y[j] ) * ( y[j] * y[j] + (y[j]*y[j+1]) + y[j+1] * y[j+1] ) );  // % alp_02 calc
         }

        double  alp_00 = sum1/2;
        double  a = alp_00;
   //      cout<<"area"<<a<<endl;
      //   getchar();
        double  alp_10 = ( 1/(6*a) ) * sum2 ;
        double  alp_01 = ( 1/(6*a) ) * sum3 ;
       double  alp_11 = ( 1/(24*a) ) * sum4 ;
       double  alp_20 = ( 1/(12*a) ) * sum5 ;
       double  alp_02 = ( 1/(12*a) ) * sum6 ;

        double  mu_11 = alp_11 - alp_10*alp_01;
        double mu_20 = alp_20 - alp_10 * alp_10;
        double mu_02 = alp_02 - alp_01 *alp_01;
        // ******************************
 /*       double n11 = mom.mu11/mom.m00;
        double n20 = mom.m20;
        double n02 = mom.m02;
        double Xg=mom.m10/mom.m00;
        double Yg=mom.m01/mom.m00;
        cout<< "Moments: " << mom.m00 <<" " <<n11 << " " << mom.nu11<<endl;*/

        double Xg = alp_10;
        double Yg = alp_01;
        double Zg=z[i];
//        double centroid[0]=Xg;
//        double centroid[1]=Yg;

        double n11 = mu_11;
        double n20 = mu_20;
        double n02 = mu_02;
  //      cout <<"Xg and Yg"<< Xg << " " << Yg << endl;
  //      cout << "centroid" << centroid[0]  << " " << centroid[1] << endl;rosrun control_speed new_test_speed

    //    cout << "area" <<a <<endl;

        double Xg_vz = ( Xg/Zg ) + 4 *( A * n20 + B * n11 );
        double Yg_vz = (Yg/Zg ) + 4 *( A * n11 + B * n02 );
        double Xg_wx = ( Xg * Yg ) + 4 * n11;
        double Yg_wy = -Xg_wx;
        double Xg_wy = -( 1 + Xg * Xg + 4 * n20 );
        double Yg_wx = -( 1 + Yg * Yg + 4 * n02 );

    //    getchar();

  /*      L.at<double>(i,0) = (-lambda / z);
        L.at<double>(i,1) = 0.0;
        L.at<double>(i,2) =  (centroid[i] / z);
        L.at<double>(i,3) =  (centroid[i]*centroid[i+1]/lambda);
        L.at<double>(i,4) =  (-(lambda*lambda+centroid[i]*centroid[i])/lambda);
        L.at<double>(i,5) =  (centroid[i+1]);

        L.at<double>(i+1,0) =  0;
        L.at<double>(i+1,1) =  (-lambda/z);
        L.at<double>(i+1,2) =  (centroid[i+1]/z);
        L.at<double>(i+1,3) =  (-(lambda*lambda+centroid[i+1]*centroid[i+1])/lambda);
        L.at<double>(i+1,4) =  (-centroid[i]*centroid[i+1]/lambda);
        L.at<double>(i+1,5) =  (-centroid[i]); */

        L.at<double>(i,0) = (-lambda / Zg);
        L.at<double>(i,1) = 0.0;
        L.at<double>(i,2) =  Xg_vz;
        L.at<double>(i,3) =  Xg_wx;
        L.at<double>(i,4) =  Xg_wy;
        L.at<double>(i,5) =  Yg;

        L.at<double>(i+1,0) =  0;
        L.at<double>(i+1,1) =  (-lambda/Zg);
        L.at<double>(i+1,2) =  Yg_vz;
        L.at<double>(i+1,3) = Yg_wx;
        L.at<double>(i+1,4) = Yg_wy;
        L.at<double>(i+1,5) = -Xg;



        L.at<double>(i+2,0) = -a * A;
        L.at<double>(i+2,1) = -a * B;
        L.at<double>(i+2,2) = a * ( (3 / Zg) - C );
  //      L.at<double>(i+2,3) = 3 * area * centroid[i+1];
   //     L.at<double>(i+2,4) = - 3 * area * centroid[i];
        L.at<double>(i+2,3) = 3 * a * Yg;
        L.at<double>(i+2,4) = - 3 * a * Xg;
        L.at<double>(i+2,5) = 0;
 //  }
}

