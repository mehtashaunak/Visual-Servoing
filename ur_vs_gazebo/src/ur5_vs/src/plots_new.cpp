#include <iostream>
#include <gnuplot_ci.h>
#include <stdlib.h>

/*--------plot-------*/

using namespace gnuplot_ci;
using namespace std;

//void plots(){

//    //Plot of Tree1 in image space
//    GP_handle G1("/usr/bin/", "X (m)", "Y (m)");
//    G1.gnuplot_cmd("set terminal wxt");
//    G1.gnuplot_cmd("plot 'start.txt'  u 1:2 w p pt 7 ps 2 t 'Start feature', 'goal.txt'  u 1:2 w p pt 7 ps 2 t 'Goal feature'");
//    G1.gnuplot_cmd("replot 'tree1.txt'  u 1:2 w p pt 0 t 'RRT Tree in image space', 'tree1back.txt'  u 1:2 w p pt 0 t 'Image Trajectory'");
//    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
//    G1.gnuplot_cmd("set output 'tree1combined.eps'");
//    G1.gnuplot_cmd("replot");

//    cout << "Press enter to save plots" << endl;
//    getchar();
//}

//int main(int argc, char** argv)
//{

//    //Plot of Joint velocity
//    GP_handle G1("/usr/bin/", "X (m)", "Y (m)");
//    G1.gnuplot_cmd("set terminal wxt");
//    G1.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 1 w l t 'dth1'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 2 w l t 'dth2'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 3 w l t 'dth3'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 4 w l t 'dth4'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 5 w l t 'dth5'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 6 w l t 'dth6'");
//    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
//    G1.gnuplot_cmd("set output '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/Joint_velocities.eps'");
//    G1.gnuplot_cmd("replot");


//    //Plot of Error
//    GP_handle G2("/usr/bin/", "X (m)", "Y (m)");
//    G2.gnuplot_cmd("set terminal wxt");
//    G2.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/feature_error.txt'  u 4 w l t 'norm error'");
//    G2.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
//    G2.gnuplot_cmd("set output '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/feature_error.eps'");
//    G2.gnuplot_cmd("replot");

////    //Plot of feature comaparison in image space
////    GP_handle G3("/usr/bin/", "X (m)", "Y (m)");
////    G3.gnuplot_cmd("set terminal wxt");
////    G3.gnuplot_cmd("set xrange [-320:320]");
////    G3.gnuplot_cmd("set yrange [-240:240]");
////    //G3.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/start.txt'  u 1:2 w p pt 7 ps 2 t 'Start feature', '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/goal.txt'  u 1:2 w p pt 7 ps 2 t 'Goal feature'");
////    G3.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/tree1back.txt'  u 1:2 w p pt 0 t 'Planned Trajectory', '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/feature_executed.txt'  u 1:2 w p pt 0 t 'Executed Trajectory'");
////    G3.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
////    G3.gnuplot_cmd("set output '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/tree1compare.eps'");
////    G3.gnuplot_cmd("replot");

////    //Plot feature Trajectory in image space
////    GP_handle G2("/usr/bin/", "X (m)", "Y (m)");
////    G2.gnuplot_cmd("set terminal wxt");
////    //G2.gnuplot_cmd("plot 'start.txt'  u 1:2 w p pt 7 ps 2 t 'Start feature', 'goal.txt'  u 1:2 w p pt 7 ps 2 t 'Goal feature'");
////    G2.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/tree1back.txt'  u 1:2 w l t 'Planned Trajectory', '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/feature_executed.txt'  u 1:2 w l t 'Executed Trajectory'");
////    G2.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
////    G2.gnuplot_cmd("set output '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/plots/trajcompare.eps'");
////    G2.gnuplot_cmd("replot");

////    //Plot of Tree1 in image space
////    GP_handle G3("/usr/bin/", "X (m)", "Y (m)");
////    G3.gnuplot_cmd("set terminal wxt");
////    G3.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/start.txt'  u 1:2 w p pt 7 ps 2 t 'Start feature', '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/goal.txt'  u 1:2 w p pt 7 ps 2 t 'Goal feature'");
////    G3.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/tree1.txt'  u 1:2 w p pt 0 t 'RRT Tree in image space', '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/tree1back.txt'  u 1:2 w p pt 0 t 'Image Trajectory'");
////    G3.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
////    G3.gnuplot_cmd("set output '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/plots/tree1combined.eps'");
////    G3.gnuplot_cmd("replot");

//    cout << "Press enter to save plots" << endl;
//    getchar();
//        return 0;
//}

//int main(int argc, char** argv)
//{

//    //Plot of camera velocity
//    GP_handle G1("/usr/bin/", "X (m)", "Y (m)");
//    G1.gnuplot_cmd("set terminal wxt");
//    G1.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 1 w l t 'dth1'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 2 w l t 'dth2'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 3 w l t 'dth3'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 4 w l t 'dth4'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 5 w l t 'dth5'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/joint_velocities.txt'  u 6 w l t 'dth6'");
//    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
//    G1.gnuplot_cmd("set output '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/Joint_velocities.eps'");
//    G1.gnuplot_cmd("replot");

//    //Plot of camera velocity
//    GP_handle G1("/usr/bin/", "X (m)", "Y (m)");
//    G1.gnuplot_cmd("set terminal wxt");
//    G1.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/joint_velocities.txt'  u 1 w l t 'dth1'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/joint_velocities.txt'  u 2 w l t 'dth2'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/joint_velocities.txt'  u 3 w l t 'dth3'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/joint_velocities.txt'  u 4 w l t 'dth4'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/joint_velocities.txt'  u 5 w l t 'dth5'");
//    G1.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/joint_velocities.txt'  u 6 w l t 'dth6'");
//    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
//    G1.gnuplot_cmd("set output '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/Joint_velocities.eps'");
//    G1.gnuplot_cmd("replot");

//    //Plot of Error
//    GP_handle G2("/usr/bin/", "X (m)", "Y (m)");
//    G2.gnuplot_cmd("set terminal wxt");
//    G2.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/feature_error.txt'  u 4 w l t 'norm error'");
////    G2.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/feature_error.txt'  u 2 w l t 'dth2'");
////    G2.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area_simple_ibvs/exp_1/feature_error.txt'  u 3 w l t 'dth3'");
//    G2.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
//    G2.gnuplot_cmd("set output '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/feature_error.eps'");
//    G2.gnuplot_cmd("replot");

//    //Plot of feature comaparison in image space
//    GP_handle G3("/usr/bin/", "X (m)", "Y (m)");
//    G3.gnuplot_cmd("set terminal wxt");
//    G3.gnuplot_cmd("set xrange [-320:320]");
//    G3.gnuplot_cmd("set yrange [-240:240]");
//    //G3.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/start.txt'  u 1:2 w p pt 7 ps 2 t 'Start feature', '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/goal.txt'  u 1:2 w p pt 7 ps 2 t 'Goal feature'");
//    G3.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/tree1back.txt'  u 1:2 w p pt 0 t 'Planned Trajectory', '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/feature_executed.txt'  u 1:2 w p pt 0 t 'Executed Trajectory'");
//    G3.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
//    G3.gnuplot_cmd("set output '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/tree1compare.eps'");
//    G3.gnuplot_cmd("replot");

////    //Plot feature Trajectory in image space
////    GP_handle G2("/usr/bin/", "X (m)", "Y (m)");
////    G2.gnuplot_cmd("set terminal wxt");
////    //G2.gnuplot_cmd("plot 'start.txt'  u 1:2 w p pt 7 ps 2 t 'Start feature', 'goal.txt'  u 1:2 w p pt 7 ps 2 t 'Goal feature'");
////    G2.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/tree1back.txt'  u 1:2 w l t 'Planned Trajectory', '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/feature_executed.txt'  u 1:2 w l t 'Executed Trajectory'");
////    G2.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
////    G2.gnuplot_cmd("set output '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/plots/trajcompare.eps'");
////    G2.gnuplot_cmd("replot");

////    //Plot of Tree1 in image space
////    GP_handle G3("/usr/bin/", "X (m)", "Y (m)");
////    G3.gnuplot_cmd("set terminal wxt");
////    G3.gnuplot_cmd("plot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/start.txt'  u 1:2 w p pt 7 ps 2 t 'Start feature', '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/goal.txt'  u 1:2 w p pt 7 ps 2 t 'Goal feature'");
////    G3.gnuplot_cmd("replot '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/tree1.txt'  u 1:2 w p pt 0 t 'RRT Tree in image space', '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/tree1back.txt'  u 1:2 w p pt 0 t 'Image Trajectory'");
////    G3.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
////    G3.gnuplot_cmd("set output '/home/mithun/visual_servoing_ws/src/ur5_vs/src/experiments_data_area/exp_1/plots/tree1combined.eps'");
////    G3.gnuplot_cmd("replot");

//    cout << "Press enter to save plots" << endl;
//    getchar();
//        return 0;
//}
double xb[3] = {0,0,0};
double dx[3] = {1, 1, 1};

//int main(){

//    //Plot of Tree1 in image space
//    GP_handle G1("/usr/bin/", "X (m)", "Y (m)", "Area (m sq.)");
//    G1.gnuplot_cmd("set terminal wxt");
//    G1.gnuplot_cmd("set border");
//    G1.gnuplot_cmd("set ticslevel 0");
//    G1.gnuplot_cmd("set xrange [-320:320]");
//    G1.gnuplot_cmd("set yrange [-240:240]");
//    G1.gnuplot_cmd("set zrange [16000:10000]");
//    G1.gnuplot_cmd("splot '/home/mithun/start.txt'  u 1:2:3 w p pt 7 ps 2 t 'Start feature', '/home/mithun/goal.txt'  u 1:2:3 w p pt 7 ps 2 t 'Goal feature'");
//    G1.gnuplot_cmd("replot '/home/mithun/tree1.txt'  u 1:2:3 w p pt 0 t 'RRT Tree in image space'");
//    G1.gnuplot_cmd("replot '/home/mithun/tree1back.txt'  u 1:2:3 w p pt 0 t 'Planned Trajectory'");
//    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
//    G1.gnuplot_cmd("set output 'tree1area.eps'");
//    G1.gnuplot_cmd("replot");
//    G1.draw3dcoordAxis(xb, dx, true,2);
////    G1.draw3dcoordAxis(xp, xe1[0], xe1[1], xe1[2], true,2);
////    G1.draw3dcoordAxis(xp, xe2[0], xe2[1], xe2[2], true,3);

//    cout << "Press enter to save plots" << endl;
//    getchar();
//}

int main(){

    //Plot of Tree1 in image space
    GP_handle G1("/usr/bin/", "x-coordinate", "y-coordinate", "area");
    G1.gnuplot_cmd("set terminal wxt");
    G1.gnuplot_cmd("set border");
    G1.gnuplot_cmd("set ticslevel 0");
    G1.gnuplot_cmd("set xrange [-320:320]");
    G1.gnuplot_cmd("set yrange [-240:240]");
    G1.gnuplot_cmd("set zrange [16000:10000]");
    G1.gnuplot_cmd("splot '/home/mithun/start.txt'  u 1:2:3 w p pt 7 ps 2 t 'Start feature', '/home/mithun/goal.txt'  u 1:2:3 w p pt 7 ps 2 t 'Goal feature'");
    G1.gnuplot_cmd("replot '/home/mithun/tree1back.txt'  u 1:2:3 w p pt 0 t 'Planned Trajectory', '/home/mithun/feature_executed.txt'  u 1:2:3 w p pt 0 t 'Executed Trajectory'");
    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
    G1.gnuplot_cmd("set output '/home/mithun/trajcompare.eps'");
    G1.gnuplot_cmd("replot");
    G1.draw3dcoordAxis(xb, dx, true,3);
//    G1.draw3dcoordAxis(xp, xe1[0], xe1[1], xe1[2], true,2);
//    G1.draw3dcoordAxis(xp, xe2[0], xe2[1], xe2[2], true,3);

    cout << "Press enter to save plots" << endl;
    getchar();
}
//int main(){


//    //Plot of Tree1 in image space
//    GP_handle G1("/usr/bin/", "X (m)", "Y (m)");
//    G1.gnuplot_cmd("set terminal wxt");
//    G1.gnuplot_cmd("plot 'start.txt'  u 1:2 w p pt 7 ps 2 t 'Start feature', 'goal.txt'  u 1:2 w p pt 7 ps 2 t 'Goal feature'");
//    G1.gnuplot_cmd("plot 'tree1.txt'  u 1:2 w p pt 0 t 'RRT Tree in image space'");
//    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
//    G1.gnuplot_cmd("set output 'tree1temp.eps'");
//    G1.gnuplot_cmd("replot");


//    //Plot of Tree1 in image space
//    GP_handle G1("/usr/bin/", "X (m)", "Y (m)");
//    G1.gnuplot_cmd("set terminal wxt");
//    G1.gnuplot_cmd("plot 'start.txt'  u 1:2 w p pt 7 ps 2 t 'Start feature', 'goal.txt'  u 1:2 w p pt 7 ps 2 t 'Goal feature'");
//    G1.gnuplot_cmd("replot 'tree1.txt'  u 1:2 w p pt 0 t 'RRT Tree in image space', 'tree1back.txt'  u 1:2 w p pt 0 t 'Image Trajectory'");
//    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
//    G1.gnuplot_cmd("set output 'tree1combined.eps'");
//    G1.gnuplot_cmd("replot");

//    //Plot of Tree1 in image space
//    GP_handle G1("/usr/bin/", "X (m)", "Y (m)");
//    G1.gnuplot_cmd("set terminal wxt");
//    G1.gnuplot_cmd("plot 'start.txt'  u 1:2 w p pt 7 ps 2 t 'Start feature', 'goal.txt'  u 1:2 w p pt 7 ps 2 t 'Goal feature'");
//    G1.gnuplot_cmd("replot 'tree1back.txt'  u 1:2 w l t 'Planned Trajectory', 'feature_executed.txt'  u 1:2 w l t 'Executed Trajectory'");
//    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,16'");
//    G1.gnuplot_cmd("set output 'trajcomparenewcontroller.eps'");
//    G1.gnuplot_cmd("replot");

//    cout << "Press enter to save plots" << endl;
//    getchar();
//}



