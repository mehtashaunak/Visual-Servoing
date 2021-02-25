
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
void gen_new_error_DC(int i, int bias, int n, double sd[], int &ix, std::vector< std::vector<double> > tree1, double err[], int &dc_violated){

    double nx, ny, na, norm, rn1, rn2, rn3;
    int max, min;

    if (i%10>=bias){
        srand(time(NULL)*i);

        //        rn1 = (rand()%(300000+300000 + 1) -300000)/1000.0; //Random number between -320 & 320
        //        rn2 = (rand()%(200000+200000 + 1) -200000)/1000.0; //Random number between -240 & 240
        //        rn3 = (rand()%(2000000 - 1600000 + 1) + 1600000)/100.0; //Random number between 0 & 20000

        //x
        //        max=250;
        //        min=-50;
        max=320;
        min=-320;
        rn1 = (rand()%(max*1000-min*1000+1)+min*1000)/1000.0; //Random number between -320 & 320
        //y
        //        max=50;
        //        min=-150;
        max=240;
        min=-240;
        rn2 = (rand()%(max*1000-min*1000+1)+min*1000)/1000.0; //Random number between -240 & 240
        //area
        //        max=16000;
        //        min=15000;
        max=3000;
        min=2000;
        rn3 = (rand()%(max*1000-min*1000+1)+min*1000)/1000.0; //Random number between 0 & 20000
        rn3 = sd[2];
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
        //        norm = sqrt(nx+ny); //excluding area
        norm = sqrt(nx+ny+na); //including area

        //Direction criteria

        double dpx, dpy, dpa, dp_norm, drx, dry, dra, dr_norm, dp[3],dr[3], dpr[3];

        drx=r[0]-t[0];
        dry=r[1]-t[1];
        dra=r[2]-t[2];
        dr_norm=sqrt(pow(drx,2)+pow(dry,2)+pow(dra,2));
        dr[0]=drx/dr_norm;
        dr[1]=dry/dr_norm;
        dr[2]=dra/dr_norm;
        //        cout << "drxya " << drx << " " << dry << " " << dra << endl;
        //        cout << "dr " << dr[0] << " " << dr[1] << " " << dr[2] << endl;

        if (i==0){
            dpx=tree1[0][i]-0;
            dpy=tree1[1][i]-0;
            dpa=tree1[2][i]-0;
            dp_norm=sqrt(pow(dpx,2)+pow(dpy,2)+pow(dpa,2));
            dp[0]=dpx/dp_norm;
            dp[1]=dpy/dp_norm;
            dp[2]=dpa/dp_norm;
            //            cout << "dpxya " << dpx << " " << dpy << " " << dpa << endl;
            //            cout << "dp " << dp[0] << " " << dp[1] << " " << dp[2] << endl;

        }
        else {
            dpx=tree1[0][i]-tree1[0][tree1[6][i-1]];
            dpy=tree1[1][i]-tree1[1][tree1[6][i-1]];
            dpa=tree1[2][i]-tree1[2][tree1[6][i-1]];
            dp_norm=sqrt(pow(dpx,2)+pow(dpy,2)+pow(dpa,2));
            dp[0]=dpx/dp_norm;
            dp[1]=dpy/dp_norm;
            dp[2]=dpa/dp_norm;
            //            cout << "dpxya " << dpx << " " << dpy << " " << dpa << endl;
            //            cout << "dp " << dp[0] << " " << dp[1] << " " << dp[2] << endl;
        }
        dpr[0]=abs(dr[0]-dp[0]);
        dpr[1]=abs(dr[1]-dp[1]);
        dpr[2]=abs(dr[2]-dp[2]);
        //        cout << "dpr " << dpr[0] << " " << dpr[1] << " " << dpr[2] << endl;

        double dc_limit = sin(M_PI/10);
        //        cout << "DC limit = sin(M_PI/10) = " <<  dc_limit << endl;

        if ((dpr[0] < dc_limit) && (dpr[1] < dc_limit) && (dpr[2] < dc_limit)) //sin(M_PI/10) = 0.309
        {
            //            cout << "DC satisfied" << endl;
            d_frm_ran[i]=norm;
        }
        else{
            //            cout << "DC not satisfied" << endl;
            d_frm_ran[i]=1000000000;
        }

        //        d_frm_ran[i]=norm;
    }
//    for (int i = 0; i < d_frm_ran.size(); i++) {
//        cout << "d_frm_ran " << d_frm_ran[i] << endl;
//    }
    double min_pos = distance(d_frm_ran.begin(),min_element(d_frm_ran.begin(),d_frm_ran.end()));
    double temp=*min_element(d_frm_ran.begin(),d_frm_ran.end());
    //    cout << "d_min" << temp << endl;
    ix=min_pos; //parent node index;
    //    getchar();

    if (temp == 1000000000){
        dc_violated = 1; //all nodes violated direction criteria
        cout << "DC not satisfied" << endl;

    }
    else{
        dc_violated = 0;
    }
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

