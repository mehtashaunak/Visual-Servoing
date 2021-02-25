
#include <iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void new_area_L_matrix(double centroid[],double u[], double z[],double &area, cv::Mat &L)
{
    int i=0;
    double lambda=531.15;//531.15;
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

    vector1[0] = P1[0] - P2[0];
    vector1[1] = P1[1] - P2[1];
    vector1[2] = P1[2] - P2[2];
    vector2[0] = P1[0] - P3[0];
    vector2[1] = P1[1] - P3[1];
    vector2[2] = P1[2] - P3[2];

    //-------Cross product formula
    normal[0] = (vector1[1] * vector2[2]) - (vector1[2] * vector2[1]);
    normal[1] = (vector1[2] * vector2[0]) - (vector1[0] * vector2[2]);
    normal[2] = (vector1[0] * vector2[1]) - (vector1[1] * vector2[0]);

    double A = normal[0];
    double B = normal[1];
    double C = normal[2];
    double D = (normal[0]* (-P1[0])) + (normal[1] * (-P1[1])) + ( normal[2] * (-P1[2]));


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
    area = alp_00; //0.008

    double  alp_10 = ( 1/(6*area) )  * sum2;
    double  alp_01 = ( 1/(6*area) )  * sum3;
    double  alp_11 = ( 1/(24*area) ) * sum4;
    double  alp_20 = ( 1/(12*area) ) * sum5;
    double  alp_02 = ( 1/(12*area) ) * sum6;

    double mu_11 = alp_11 - alp_10 * alp_01;
    double mu_20 = alp_20 - alp_10 * alp_10;
    double mu_02 = alp_02 - alp_01 * alp_01;




    double Xg = alp_10;
    double Yg = alp_01;
    double Zg = z[i];

//    double an = 0.488 * sqrt((22721*0.01)/(area*0.01));
//           Xg = an*Xg;
//           Yg = an*Yg;

            Zg =1/(A*Xg + B*Yg + C);

    //        double n11 = mu_11;
    //        double n20 = mu_20;
    //        double n02 = mu_02;


//    double n11 = mu_11/alp_00;
//    double n20 = mu_20/alp_00;
//    double n02 = mu_02/alp_00;

    double n11 = mu_11;
    double n20 = mu_20;
    double n02 = mu_02;

//    cout <<"Xg and Yg"<< Xg << " " << Yg << endl;

    double Xg_vz = ( Xg/Zg ) + 4 *( A * n20 + B * n11 );
    double Yg_vz = (Yg/Zg ) + 4 *( A * n11 + B * n02 );
    double Xg_wx = ( Xg * Yg ) + 4 * n11;
    double Yg_wy = -Xg_wx;
//    double Xg_wy = -( 1 + Xg * Xg + 4 * n20 );
//    double Yg_wx = ( 1 + Yg * Yg + 4 * n02 );

    double Xg_wy = -((lambda * lambda)+ Xg * Xg + 4 * n20 );
    double Yg_wx = ( (lambda * lambda) + Yg * Yg + 4 * n02 );

    double eps1 = (Xg * Yg) + 4 * n11;
    double eps2 = (Xg * Xg) + 4 * n20;
    double eps3 = (Yg * Yg) + 4 * n02;


//    double eps11 = 4*n11 - ((Xg*Yg)/2);
//    double eps22 = eps11;
//    double eps12 = 4*n20 - ((Xg*Xg)/2);
//    double eps21 = 4*n02 - ((Yg*Yg)/2);
//    double eps31 = 3*Yg/2;
//    double eps32 = 3*Xg/2;

           double eps11 = (4*n11) - ((Xg*Yg)/(2*lambda));
           double eps22 = eps11;
           double eps12 = ((4*n20*lambda * lambda) - ((Xg*Xg)/2))/lambda;
           double eps21 = ((4*n02*lambda * lambda) - ((Yg*Yg)/2))/lambda;
           double eps31 = 3*Yg/2;
           double eps32 = 3*Xg/2;


//    L.at<double>(i,0) = (-1/Zg);
//    L.at<double>(i,1) = 0.0;
//    L.at<double>(i,2) =  Xg_vz;
//    L.at<double>(i,3) =  Xg_wx;
//    L.at<double>(i,4) =  Xg_wy;
//    L.at<double>(i,5) =  Yg;

//    L.at<double>(i+1,0) =  0;
//    L.at<double>(i+1,1) =  (-1/Zg);
//    L.at<double>(i+1,2) =  Yg_vz;
//    L.at<double>(i+1,3) = -Yg_wx;
//    L.at<double>(i+1,4) = Yg_wy;
//    L.at<double>(i+1,5) = -Xg;
/*--------------working one--xy-----------------*/
    L.at<double>(i,0) = (-lambda/Zg);
    L.at<double>(i,1) = 0.0;
    L.at<double>(i,2) =  Xg_vz;
    L.at<double>(i,3) =  Xg_wx/lambda;
    L.at<double>(i,4) =  Xg_wy/lambda;
    L.at<double>(i,5) =  Yg;

    L.at<double>(i+1,0) =  0;
    L.at<double>(i+1,1) =  (-lambda/Zg);
    L.at<double>(i+1,2) =  Yg_vz;
    L.at<double>(i+1,3) = Yg_wx/lambda;
    L.at<double>(i+1,4) = Yg_wy/lambda;
    L.at<double>(i+1,5) = -Xg;

    L.at<double>(i+2,0) = - area*A;
    L.at<double>(i+2,1) = -area*B;
    L.at<double>(i+2,2) = area*((3/Zg)-C);
    L.at<double>(i+2,3) = 3 * area * Yg;
    L.at<double>(i+2,4) = - 3 * area * Xg;
    L.at<double>(i+2,5) = 0;

//    L.at<double>(i,0) = 0;
//    L.at<double>(i,1) = 0;
//    L.at<double>(i,2) = 0;
//    L.at<double>(i,3) = 0;
//    L.at<double>(i,4) = 0;
//    L.at<double>(i,5) = 0;

//    L.at<double>(i+1,0) = 0;
//    L.at<double>(i+1,1) = 0;
//    L.at<double>(i+1,2) = 0;
//    L.at<double>(i+1,3) = 0;
//    L.at<double>(i+1,4) = 0;
//    L.at<double>(i+1,5) = 0;

//    L.at<double>(i+2,0) = 0;
//    L.at<double>(i+2,1) = 0;
//    L.at<double>(i+2,2) = 2 * alp_00 * Zg;
//    L.at<double>(i+2,3) = 3 * alp_00 * Yg;
//    L.at<double>(i+2,4) = - 3 * alp_00 * Xg;
//    L.at<double>(i+2,5) = 0;

//    L.at<double>(i+2,0) = 0;
//    L.at<double>(i+2,1) = 0;
//    L.at<double>(i+2,2) = 1 * an * 1/Zg;
////    L.at<double>(i+2,3) = 3 * an * Xg;
////    L.at<double>(i+2,4) = - 3 * an * Yg;
//    L.at<double>(i+2,3) = 1 * an * xn;
//    L.at<double>(i+2,4) = - 1 * an * yn;
//    L.at<double>(i+2,5) = 0;

/*------------------------------------------------*/

//    L.at<double>(i,0) =  -1;
//    L.at<double>(i,1) =   0;
//    L.at<double>(i,2) =   0;
//    L.at<double>(i,3) =   an*eps11;
//    L.at<double>(i,4) =  -an*(1+eps12);
//    L.at<double>(i,5) =   Yg;
//////    L.at<double>(i,5) =   0;

//    L.at<double>(i+1,0) =  0;
//    L.at<double>(i+1,1) = -1;
//    L.at<double>(i+1,2) =  0;
//    L.at<double>(i+1,3) =  an*(1+eps21);
//    L.at<double>(i+1,4) =  -an*eps22;
//    L.at<double>(i+1,5) =  -Xg;
////    L.at<double>(i+1,5) =  0;


//            L.at<double>(i,0) = -lambda / z[i];
//            L.at<double>(i,1) = 0.0;
//            L.at<double>(i,2) = centroid[i] / z[i];
//            L.at<double>(i,3) = (centroid[i] * centroid[i+1]) / lambda;
//            L.at<double>(i,4) = -((lambda * lambda) + (centroid[i] * centroid[i])) / lambda;
//            L.at<double>(i,5) = centroid[i+1];

//            L.at<double>(i+1,0) = 0;
//            L.at<double>(i+1,1) = -lambda / z[i];
//            L.at<double>(i+1,2) = centroid[i+1] / z[i];
//            L.at<double>(i+1,3) = -((lambda * lambda) + (centroid[i+1] * centroid[i+1])) / lambda;
//            L.at<double>(i+1,4) = (-centroid[i] * centroid[i+1]) / lambda;
//            L.at<double>(i+1,5) = -centroid[i];


//    L.at<double>(i+2,0) =  0;
//    L.at<double>(i+2,1) =  0;
//    L.at<double>(i+2,2) =  -1;
//    L.at<double>(i+2,3) = -an*eps31;
//    L.at<double>(i+2,4) =  an*eps32;
//    L.at<double>(i+2,5) = 0;

//            L.at<double>(i+2,0) =  0;
//            L.at<double>(i+2,1) =  0;
//            L.at<double>(i+2,2) =  0;
//            L.at<double>(i+2,3) =  0;
//            L.at<double>(i+2,4) = 0;
//            L.at<double>(i+2,5) = 0;


//    L.at<double>(i,0) =  -C;
//    L.at<double>(i,1) =   0;
//    L.at<double>(i,2) =   C*Xg;
//    L.at<double>(i,3) =   eps1;
//    L.at<double>(i,4) =  -(1+eps2);
//    L.at<double>(i,5) =   Yg;

//    L.at<double>(i+1,0) =  0;
//    L.at<double>(i+1,1) = -C;
//    L.at<double>(i+1,2) =  C*Yg;
//    L.at<double>(i+1,3) =  1+eps3;
//    L.at<double>(i+1,4) =  -eps1;
//    L.at<double>(i+1,5) =  -Xg;

//    L.at<double>(i+2,0) =  0;
//    L.at<double>(i+2,1) =  0;
//    L.at<double>(i+2,2) =  2*area*C;
//    L.at<double>(i+2,3) =  3*area*Yg;
//    L.at<double>(i+2,4) = -3*area*Xg;
//    L.at<double>(i+2,5) = 0;

}



