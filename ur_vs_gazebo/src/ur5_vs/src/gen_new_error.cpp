#include<iostream>
#include<cstdlib>
#include<stdio.h>
#include<cmath>
#include<vector>
#include<algorithm>
#include<ctime>
//#include"gen_new_node.h"
#include "my_funcs.h"
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace std;

//void gen_new_node(int i, int bias, int n, int n_img, double sd[], double points_corner[], double q[], double dq[], cv::Mat &FP, std::vector< std::vector<double> > tree1, std::vector< std::vector<double> > tree2, double new_node_tree1[], double new_node_tree2[]){
void gen_new_error(int i, int bias, int n, double sd[], std::vector< std::vector<double> > tree1, double err[]){

    //Set parameters
    double delta = 50;
    double beta=1000000;

//    double lambda= 500000; //10000; //100000// 0.1; /*convergence rate*/
    double amax=3.14; //2; //3.14; /*upper bound on acceleration limit*/

    double nx, ny, na, norm, rn1, rn2, rn3;

    if (i%10>=bias){
        srand(time(NULL)*i);

        rn1 = (rand()%(300000+300000 + 1) -300000)/1000.0; //Random number between -320 & 320
        rn2 = (rand()%(200000+200000 + 1) -200000)/1000.0; //Random number between -240 & 240
        rn3 = (rand()%(2000000 - 1600000 + 1) + 1600000)/100.0; //Random number between 0 & 20000
        cout << "Random Feature " << rn1 <<  " " << rn2 << " " << rn3 << endl;
        //getchar();
    }
    else{
        rn1=sd[0];
        rn2=sd[1];
        rn3=sd[2];
        cout << "Biased Feature " << rn1 <<  " " << rn2 << " " << rn3 << endl;
    }

    double r[3]={rn1,rn2,rn3};

    //finding nearest node to the random node generated
    std::vector<double> d_frm_ran(n);

    for (int i=0; i<=n-1; i=i+1){
        double t1=tree1[0][i];
        double t2=tree1[1][i];
        double t3=tree1[2][i];
        double t[3]={t1, t2, t3};

        nx=pow(t[0]-r[0],2);
        ny=pow(t[1]-r[1],2);
        na=pow(t[2]-r[2],2);
        //norm = sqrt(nx+ny); //excluding area
        norm = sqrt(nx+ny+na); //including area
        d_frm_ran[i]=norm;
    }
    //    for (int i = 0; i < d_frm_ran.size(); i++) {
    //            cout << "d_frm_ran " << d_frm_ran[i] << endl;
    //        }
    double min_pos = distance(d_frm_ran.begin(),min_element(d_frm_ran.begin(),d_frm_ran.end()));
    double temp=*min_element(d_frm_ran.begin(),d_frm_ran.end());
    double ix=min_pos; //parent node index

    //position and velocity of parent feature vector
    double sp[3]={tree1[0][ix], tree1[1][ix], tree1[2][ix]};
//    double dsp[3]={tree1[3][ix], tree1[4][ix], tree1[5][ix]};

    //using random image
    double e1, e2, e3;
    e1=sp[0]-r[0];
    e2=sp[1]-r[1];
    e3=sp[2]-r[2];

    err[0]=e1;
    err[1]=e2;
    err[2]=e3;

    cout << "Parent Node index " << ix << endl;

    cout << "Random Node RRT "<< r[0] << " " << r[1] << " " << r[2] << endl;

    cout << "Parent Feature Vec " << sp[0] << " " << sp[1] << " " << sp[2] << endl;

//    cout << "Parent Feature Velocity " << dsp[0] << " " << dsp[1] << " " << dsp[2] << endl;

    cout << "Error " << e1 << " " << e2 << " " << e3 << endl;

}
