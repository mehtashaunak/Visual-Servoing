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
void gen_new_node_centroid(int i, int bias, int n, int n_img, double sd[], double q[], double dq[], cv::Mat &FP, std::vector< std::vector<double> > tree1, std::vector< std::vector<double> > tree2, double new_node_tree1[], double new_node_tree2[], int &obstacle, cv::Mat Rot, cv::Mat mul_Jacob,float centroid_depth,double centroid_img[]){

    //Set parameters
    double delta = 0.1;
    double beta=2;
    double lambda= 1; // 0.1; /*convergence rate*/
    double amax=3.14; //2; //3.14; /*upper bound on acceleration limit*/

    double nx, ny, norm, rn1, rn2;

    if (i%10>=bias){
        srand(time(NULL)*i);

        rn1 = (rand()%(320000+320000 + 1) -320000)/1000.0; //Random number between -320 & 320
        rn2 = (rand()%(240000+240000 + 1) -240000)/1000.0; //Random number between -240 & 240
        cout << "Random Feature " << rn1 <<  " " << rn2 << endl;
        //getchar();
    }
    else{
        rn1=sd[0];
        rn2=sd[1];
        cout << "Biased Feature " << rn1 <<  " " << rn2 << endl;
    }

    double r[2]={rn1,rn2};

    //finding nearest node to the random node generated
    std::vector<double> d_frm_ran(n);

    for (int i=0; i<=n-1; i=i+1){
        double t1=tree1[0][i];
        double t2=tree1[1][i];
        double t[2]={t1, t2};

        nx=pow(t[0]-r[0],2);
        ny=pow(t[1]-r[1],2);
        norm = sqrt(nx+ny); //excluding area
        d_frm_ran[i]=norm;
    }
    //    for (int i = 0; i < d_frm_ran.size(); i++) {
    //            cout << "d_frm_ran " << d_frm_ran[i] << endl;
    //        }
    double min_pos = distance(d_frm_ran.begin(),min_element(d_frm_ran.begin(),d_frm_ran.end()));
    double temp=*min_element(d_frm_ran.begin(),d_frm_ran.end());
    int ix=min_pos;

    //position and velocity of parent feature vector
    double sp[2]={tree1[0][ix], tree1[1][ix]};
    double dsp[2]={tree1[2][ix], tree1[3][ix]};

    //    // Find image nearest to random image from workspace set of images
    //    std::vector<double> d_frm_ran_2(n_img);

    //    for (int i=0; i<n_img; i=i+1){
    ////        double r2x_pix=FP.at<double>(i,1);
    ////        double r2y_pix=FP.at<double>(i,2);
    //        double r2x_pix=FP.at<double>(i,1);
    //        double r2y_pix=FP.at<double>(i,2);
    //        double r2x = (r2x_pix-320)/(531.15/640);
    //        double r2y = (r2y_pix-240)/(531.15/480);
    //        double ra=0.01*FP.at<double>(i,12);
    //        double r1=r[0];
    //        double r2=r[1];
    //        double r3=r[2];
    //        d_frm_ran_2[i]=sqrt(pow(r2x-r1,2)+pow(r2y-r2,2)+pow(ra-r3,2));
    //        //        cout << "d " << d_frm_ran_2[i] << endl;
    //    }
    //    double min_pos2 = distance(d_frm_ran_2.begin(),min_element(d_frm_ran_2.begin(),d_frm_ran_2.end()));
    //    double min_temp2=*min_element(d_frm_ran_2.begin(),d_frm_ran_2.end());
    //    int ix2=min_pos2;
    //    obstacle=ix2;

    //    //error between random and parent feature vector
    //    double rnew1, rnew2, rnew3, e1, e2, e3, r_ws[3];

    //    r_ws[0]=FP.at<double>(ix2,1);
    //    r_ws[1]=FP.at<double>(ix2,2);
    //    r_ws[2]=0.01*FP.at<double>(ix2,12);

    //    rnew1 = (r_ws[0]-320)/(531.15/640);    //(u-u0)/px //pixel domain to image plane coordinates
    //    rnew2 = (r_ws[1]-240)/(531.15/480);    //(v-v0)/px

    ////    rnew1 = r_ws[0];    //(u-u0)/px //pixel domain to image plane coordinates
    ////    rnew2 = r_ws[1];    //(v-v0)/px
    //    rnew3 = r_ws[2];

    //    e1=sp[0]-rnew1;
    //    e2=sp[1]-rnew2;
    //    e3=sp[2]-rnew3;

    //using random image
    double e1, e2;
    e1=sp[0]-r[0];
    e2=sp[1]-r[1];

    //e1=r[0]-sp[0];
    //e2=r[1]-sp[1];

    //acceleration
    double temp1, temp2;
//    temp1=-lambda*e1 - dsp[0];
//    temp2=-lambda*e2 - dsp[1];
    temp1=-lambda*e1 - dsp[0];
    temp2=-lambda*e2 - dsp[1];

    double facc1, facc2;
    facc1=min(abs(temp1),amax);
    facc2=min(abs(temp2),amax);

    double n1, n2;
    n1=pow(temp1,2);
    n2=pow(temp2,2);

    //    n1=pow(facc1,2);
    //    n2=pow(facc2,2);

    double normv = sqrt(n1+n2);

    //feature velocity
    double dsnew1, dsnew2;
    dsnew1 = dsp[0] + beta*(temp1/normv)*facc1;
    dsnew2 = dsp[1] + beta*(temp2/normv)*facc2;

    //dsnew1=0.5;
    //dsnew2=0.5;

    //New feature position from RRT
    double snew1, snew2;
    snew1 =  sp[0]+dsnew1*delta;
    snew2 =  sp[1]+dsnew2*delta;

    //    // Find image nearest to new image from workspace set of images
    //    std::vector<double> d_frm_ran_3(n_img);

    //    for (int i=0; i<n_img; i=i+1){
    ////        double r2x_pix=FP.at<double>(i,1);
    ////        double r2y_pix=FP.at<double>(i,2);
    //        double r2x_pix=FP.at<double>(i,1);
    //        double r2y_pix=FP.at<double>(i,2);
    //        double r2x = (r2x_pix-320)/(531.15/640);
    //        double r2y = (r2y_pix-240)/(531.15/480);
    //        double ra=FP.at<double>(i,12);
    //        double r1=snew1;
    //        double r2=snew2;
    //        double r3=snew3;
    //        d_frm_ran_3[i]=sqrt(pow(r2x-r1,2)+pow(r2y-r2,2)+pow(ra-r3,2));
    ////                cout << "d " << d_frm_ran_3[i] << endl;
    //    }
    //    double min_pos3 = distance(d_frm_ran_3.begin(),min_element(d_frm_ran_3.begin(),d_frm_ran_3.end()));
    //    double min_temp3=*min_element(d_frm_ran_3.begin(),d_frm_ran_3.end());
    //    int ix3=min_pos3;
    //    obstacle=ix3;

    //    //Pick up image from WS
    //    double snew1_ws_pix, snew2_ws_pix, snew3_ws_pix, snew1_ws, snew2_ws, snew3_ws;

    //    snew1_ws_pix=FP.at<double>(ix3,1);
    //    snew2_ws_pix=FP.at<double>(ix3,2);
    //    snew3_ws_pix =FP.at<double>(ix3,12);

    //    snew1_ws = (snew1_ws_pix-320)/(531.15/640);    //(u-u0)/px //pixel domain to image plane coordinates
    //    snew2_ws = (snew2_ws_pix-240)/(531.15/480);    //(v-v0)/px
    //    snew3_ws = snew3_ws_pix;

    //Output
    cout << "Parent Node index " << ix << endl;

    cout << "Random Node RRT "<< r[0] << " " << r[1] << endl;

    //    cout << "WS image index " << ix2 << endl;

    //    cout << "Random Node WS "<< rnew1 << " " << rnew2 << " " << rnew3 << endl;

    cout << "Parent Feature Vec " << sp[0] << " " << sp[1] << endl;

    cout << "Error " << e1 << " " << e2 << endl;

    //    cout << "Normalized Error " << e1/e_norm << " " << e2/e_norm << " " << e3/e_norm << endl;

    cout << "temp " << temp1 << " " << temp2 << endl;

    cout << "facc " << facc1 << " " << facc2 << endl;

    cout << "n " << n1 << " " << n2 << endl;

    cout << "Parent Feature Velocity " << dsnew1 << " " << dsnew2 << endl;

    cout << "New Feature Vec RRT " << snew1 << " " << snew2 << endl;

    //    cout << "New pick-up Feature Vec WS " << snew1_ws << " " << snew2_ws << " " << snew3_ws << endl;

    //getchar();

    //  cout << "temp2 " << min_temp2<< endl;
    //  cout << "ix2 " << ix2 << endl;
    //  getchar();

    //    double u_c[2], snew_new1, snew_new2;

    //    snew_new1=FP.at<double>(ix2,1);
    //    snew_new2=FP.at<double>(ix2,2);

    //    snew_new1= (u_c[0]-320)/(531.15/640);    //(u-u0)/px //pixel domain to image plane coordinates
    //    snew_new2= (u_c[1]-240)/(531.15/480);    //(v-v0)/py

    //    cout << "New feature after explore " << snew_new1 << " " << snew_new2 << endl;
    //    getchar();

    new_node_tree1[0]=snew1;
    new_node_tree1[1]=snew2;
    //    new_node_tree1[0]=snew1_ws;
    //    new_node_tree1[1]=snew2_ws;
    //    new_node_tree1[2]=snew3_ws;
    new_node_tree1[2]=dsnew1;
    new_node_tree1[3]=dsnew2;
    new_node_tree1[4]=ix+1;

    //            double q[6]={0,0,0,0,0,0};
    //            double dq[6]={0,0,0,0,0,0};
    //    double q[6], dq[6];

    q[0]=tree2[0][ix];
    q[1]=tree2[1][ix];
    q[2]=tree2[2][ix];
    q[3]=tree2[3][ix];
    q[4]=tree2[4][ix];
    q[5]=tree2[5][ix];

    dq[0]=tree2[6][ix];
    dq[1]=tree2[7][ix];
    dq[2]=tree2[8][ix];
    dq[3]=tree2[9][ix];
    dq[4]=tree2[10][ix];
    dq[5]=tree2[11][ix];

    //    cv::Mat q = cv::Mat(6, 1, CV_64F, 0.0);
    //    cv::Mat dq = cv::Mat(6, 1, CV_64F, 0.0);
    cv::Mat dsnew = cv::Mat(2, 1, CV_64F, 0.0);

    cv::Mat J = cv::Mat(6, 6, CV_64F, 0.0);
    cv::Mat Jinv = cv::Mat(6, 6, CV_64F, 0.0);
    cv::Mat L = cv::Mat(2, 6, CV_64F, 0.0);
    cv::Mat Linv = cv::Mat(6, 2, CV_64F, 0.0);
    cv::Mat Rbe = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat Rec = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat Rbc = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat Rcb = cv::Mat(3, 3, CV_64F, 0.0);
    cv::Mat V_bf = cv::Mat(6, 1, CV_64F, 0.0);
    cv::Mat V_bf1 = cv::Mat(3, 1, CV_64F, 0.0);

    cv::Mat V_cf = cv::Mat(6, 1, CV_64F, 0.0);
    cv::Mat V_cf1 = cv::Mat(3, 1, CV_64F, 0.0);

    cv::Mat V_cf_lin = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V_cf_ang = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V_bf_lin = cv::Mat(3, 1, CV_64F, 0.0);
    cv::Mat V_bf_ang = cv::Mat(3, 1, CV_64F, 0.0);

    cv::Mat vector_ec = cv::Mat(1, 3, CV_64F, 0.0);
    cv::Mat Jec = cv::Mat(6, 6, CV_64F, 0.0);
    cv::Mat J1 = cv::Mat(6, 6, CV_64F, 0.0);
    cv::Mat J1inv = cv::Mat(6, 6, CV_64F, 0.0);

    cv::Mat UC = cv::Mat(2, 1, CV_64F, 0.0);
    cv::Mat UD = cv::Mat(2, 1, CV_64F, 0.0);
    cv::Mat err = cv::Mat(2, 1, CV_64F, 0.0);
    cv::Mat q_old = cv::Mat(6, 1, CV_64F, 0.0);
    cv::Mat qnew = cv::Mat(6, 1, CV_64F, 0.0);
    cv::Mat dqnew = cv::Mat(6, 1, CV_64F, 0.0);

    // q_old is same as q[] array but variable changed to cv::mat to do matrix addition
    q_old.at<double>(0,0)=tree2[0][ix];
    q_old.at<double>(1,0)=tree2[1][ix];
    q_old.at<double>(2,0)=tree2[2][ix];
    q_old.at<double>(3,0)=tree2[3][ix];
    q_old.at<double>(4,0)=tree2[4][ix];
    q_old.at<double>(5,0)=tree2[5][ix];

    //    dq.at<double>(0,0)=tree2[6][ix];
    //    dq.at<double>(1,0)=tree2[7][ix];
    //    dq.at<double>(2,0)=tree2[8][ix];
    //    dq.at<double>(3,0)=tree2[9][ix];
    //    dq.at<double>(4,0)=tree2[10][ix];
    //    dq.at<double>(5,0)=tree2[11][ix];

    //Centroid of image
    double centroid[2];

        centroid[0]=sp[0]; //cuurent
        centroid[1]=sp[1];

//    centroid[0]=sd[0]; //desired
//    centroid[1]=sd[1];

    //Depth of centroid
    double depth_p[2];
    double depth_centroid = (double)centroid_depth; //0.62; //0.553; //fixed
    depth_p[0]=depth_centroid;
    depth_p[1]=depth_centroid;

//    cout<<"depth"<<depth_p[0]<<depth_p[1]<<endl;

    //Image Jacobian
    compute_image_jacobian(centroid,depth_p,L);

    //Image Jacobain inverse
    cv::invert(L, Linv, cv::DECOMP_SVD);

    //Robot Jacobian
    compute_new_geo_jacobian(q,J);

    //Jacobian Inverse
    cv::invert(J, Jinv, cv::DECOMP_SVD);

//    Compute Rotation Matrix from base to end-effector
    base_to_ee_transform(q,Rbe);

//    Compute Rotation Matrix from end-effector to camera
    ee_to_camera_transform(Rec,vector_ec);

//    Compute Rotation Matrix from base to camera
    Rbc = Rbe*Rec;

//    Compute Rotation Matrix from camera to base (transpose of above)
    cv::transpose(Rbc, Rcb);

    //Print on terminal
    //        cout << "Image Jacobian" << L << endl;
    //        cout << "Inverse Image Jacobian" << Linv << endl;
    //        cout << "Geo Jacobian" << J << endl;
    //        cout << "Inverse Geo Jacobian" << Jinv << endl;
    //        cout << "Rot base to ee" << Rbe << endl;
    //        cout << "Rot ee to camera" << Rec << endl;

//    cout << "Rot camera to base" << Rot << endl;

    //    /*.....................Paper formulation............................*/
    //    //Velocity in camera frame
    //    V_cf1.at<double>(0,0)=dsnew1;
    //    V_cf1.at<double>(1,0)=dsnew2;

    //    //Velocity in base frame
    //    V_bf1 = Rcb*V_cf1;

    //    //Finding J1
    //    J1 = Jinv*Linv;

    //    //New joint velocities
    //    dqnew = J1*V_bf1;
    //    //    cout << "Joint Velocity " << dqnew << endl;

    //    //new joint position
    //    qnew = q_old + dqnew*delta;
    //    //    cout << "Joint Position " << qnew << endl;
    //    //    getchar();

    /*.....................Baisc formulation............................*/
    //Velocity in camera frame
    dsnew.at<double>(0,0)=dsnew1;
    dsnew.at<double>(1,0)=dsnew2;

//    err.at<double>(0,0)=sp[0]-sd[0];
//    err.at<double>(1,0)=sp[1]-sd[1];

    err.at<double>(0,0)=sp[0]-r[0];
    err.at<double>(1,0)=sp[1]-r[1];

//    err.at<double>(0,0)=centroid_img[0]-sd[0];
//    err.at<double>(1,0)=centroid_img[1]-sd[1];

        V_cf = Linv*dsnew; // paper

    cout << "Vcf (paper)" << V_cf << endl;

//        V_cf = -lambda*Linv*err; //ibvs

//    cout << "Vcf (ibvs)" << V_cf << endl;

    //Velocity in base frame

    V_cf_lin.at<double>(0,0)=V_cf.at<double>(0,0);
    V_cf_lin.at<double>(1,0)=V_cf.at<double>(1,0);
    V_cf_lin.at<double>(2,0)=V_cf.at<double>(2,0);
    V_cf_ang.at<double>(0,0)=V_cf.at<double>(5,0);
    V_cf_ang.at<double>(1,0)=V_cf.at<double>(4,0);
    V_cf_ang.at<double>(2,0)=V_cf.at<double>(3,0);

//    V_bf_lin = Rcb*V_cf_lin; //linear velcoity in base frame
//    V_bf_ang = Rcb*V_cf_ang; //angular velcoity in base frame

    V_bf_lin = Rot*V_cf_lin; //linear velcoity in base frame
    V_bf_ang = Rot*V_cf_ang; //angular velcoity in base frame

    V_bf.at<double>(0,0)=V_bf_lin.at<double>(0,0);
    V_bf.at<double>(1,0)=V_bf_lin.at<double>(1,0);
    V_bf.at<double>(2,0)=V_bf_lin.at<double>(2,0);
    V_bf.at<double>(3,0)=V_bf_ang.at<double>(0,0);
    V_bf.at<double>(4,0)=V_bf_ang.at<double>(1,0);
    V_bf.at<double>(5,0)=V_bf_ang.at<double>(2,0);

    //Include Jacobian from ee to camera
//    cout << "vector_ec " << vector_ec << endl;
//    cout << "Rec " << Rec << endl;

    Jacob_ee_to_camera(vector_ec,Rec,Jec);

//    cout << "Jec " << Jec << endl;
//    getchar();

//    J1 = Jec*J;
    J1 = mul_Jacob*J;

//        cout << "mul_Jacob " << mul_Jacob << endl;

    cv::invert(J1, J1inv, cv::DECOMP_SVD);

    //New joint velocities
    dqnew = J1inv*V_bf;
    cout << "Joint Velocity " << dqnew << endl;

    //new joint position
    qnew = q_old + dqnew*delta;

    cout << "Joint Position old " << q_old << endl;
    cout << " " << endl;

    cout << "Joint Position new" << qnew << endl;
    cout << " " << endl;
    //        getchar();

    //move_robot and get image
    //get_new_image(qnew);

    /*.....................General formulation............................*/
    //    double eta = 0.1;

    //    //Current iamge
    //    UC.at<double>(0,0)=sp[0];
    //    UC.at<double>(1,0)=sp[1];
    //    UC.at<double>(2,0)=sp[2];

    //    //Desired image
    //    UD.at<double>(0,0)=sd[0];
    //    UD.at<double>(1,0)=sd[1];
    //    UD.at<double>(2,0)=sd[2];

    //    //Error
    //    e = UD-UC;
    //    //    cout << "UC" << UC << endl;
    //    //    cout << "UD" << UD << endl;
    //    //    cout << "error" << e << endl;


    //    V_cf = -eta*Linv*e; //velcoity in camera frame

    //    //    cout << "Vel in camera frame" << V_cf << endl;

    //    V_cf_lin.at<double>(0,0)=V_cf.at<double>(0,0);
    //    V_cf_lin.at<double>(1,0)=V_cf.at<double>(1,0);
    //    V_cf_lin.at<double>(2,0)=V_cf.at<double>(2,0);
    //    V_cf_ang.at<double>(0,0)=V_cf.at<double>(5,0);
    //    V_cf_ang.at<double>(1,0)=V_cf.at<double>(4,0);
    //    V_cf_ang.at<double>(2,0)=V_cf.at<double>(3,0);

    //    V_bf_lin = Rcb*V_cf_lin; //linear velcoity in base frame
    //    V_bf_ang = Rcb*V_cf_ang; //angular velcoity in base frame

    //    V_bf.at<double>(0,0)=V_bf_lin.at<double>(0,0);
    //    V_bf.at<double>(1,0)=V_bf_lin.at<double>(1,0);
    //    V_bf.at<double>(2,0)=V_bf_lin.at<double>(2,0);
    //    V_bf.at<double>(3,0)=V_bf_ang.at<double>(0,0);
    //    V_bf.at<double>(4,0)=V_bf_ang.at<double>(1,0);
    //    V_bf.at<double>(5,0)=V_bf_ang.at<double>(2,0);

    //    //    cout << "Vel in base frame" << V_bf << endl;

    //    dqnew = Jinv*V_bf; //new joint velocity

    //    //    cout << "Joint Velocity " << dqnew << endl;

    //    //    getchar();

    //    qnew = q_old + dqnew*delta; //new joint position

    //    //    cout << "Joint position" << qnew << endl;

    ////    getchar();

    //Output

    new_node_tree2[0]=qnew.at<double>(0,0);
    new_node_tree2[1]=qnew.at<double>(1,0);
    new_node_tree2[2]=qnew.at<double>(2,0);
    new_node_tree2[3]=qnew.at<double>(3,0);
    new_node_tree2[4]=qnew.at<double>(4,0);
    new_node_tree2[5]=qnew.at<double>(5,0);
    new_node_tree2[6]=dqnew.at<double>(0,0);
    new_node_tree2[7]=dqnew.at<double>(1,0);
    new_node_tree2[8]=dqnew.at<double>(2,0);
    new_node_tree2[9]=dqnew.at<double>(3,0);
    new_node_tree2[10]=dqnew.at<double>(4,0);
    new_node_tree2[11]=dqnew.at<double>(5,0);
    new_node_tree2[12]=ix+1;

    //return new_node;

}

