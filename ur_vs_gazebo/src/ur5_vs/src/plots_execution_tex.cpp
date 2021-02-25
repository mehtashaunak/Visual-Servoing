#include <iostream>
#include <gnuplot_ci.h>
#include <stdlib.h>

/*--------plot-------*/

using namespace gnuplot_ci;
using namespace std;

double xb[3] = {0,0,0};
double dx[3] = {1, 1, 1};

int main(int argc, char** argv)
{
    //Plot of Joint velocity
    GP_handle G1("/usr/bin/", "Time (s)", "Joint velocity (rad/s)");
    G1.gnuplot_cmd("set terminal wxt");
    G1.gnuplot_cmd("plot '/home/mithun/exp0/joint_velocities.txt'  u 1 w l t '{/Symbol q}_1'");
    G1.gnuplot_cmd("replot '/home/mithun/exp0/joint_velocities.txt'  u 2 w l t '{/Symbol q}_2'");
    G1.gnuplot_cmd("replot '/home/mithun/exp0/joint_velocities.txt'  u 3 w l t '{/Symbol q}_3'");
    G1.gnuplot_cmd("replot '/home/mithun/exp0/joint_velocities.txt'  u 6 w l t '{/Symbol q}_4'");
    G1.gnuplot_cmd("replot '/home/mithun/exp0/joint_velocities.txt'  u 5 w l t '{/Symbol q}_5'");
    G1.gnuplot_cmd("replot '/home/mithun/exp0/joint_velocities.txt'  u 4 w l t '{/Symbol q}_6'");
    G1.gnuplot_cmd("set terminal epslatex");
    G1.gnuplot_cmd("set output '/home/mithun/exp0/joint_velocities.tex'");
    G1.gnuplot_cmd("replot");

    //Plot of Joint angles
    GP_handle G12("/usr/bin/", "Time (s)", "Joint angles (rad)");
    G12.gnuplot_cmd("set terminal wxt");
    G12.gnuplot_cmd("set yrange [-3.14:3.14]");
    G12.gnuplot_cmd("plot '/home/mithun/exp0/joint_angles.txt'  u 1 w l t '{/Symbol q}_1'");
    G12.gnuplot_cmd("replot '/home/mithun/exp0/joint_angles.txt'  u 2 w l t '{/Symbol q}_2'");
    G12.gnuplot_cmd("replot '/home/mithun/exp0/joint_angles.txt'  u 3 w l t '{/Symbol q}_3'");
    G12.gnuplot_cmd("replot '/home/mithun/exp0/joint_angles.txt'  u 6 w l t '{/Symbol q}_4'");
    G12.gnuplot_cmd("replot '/home/mithun/exp0/joint_angles.txt'  u 5 w l t '{/Symbol q}_5'");
    G12.gnuplot_cmd("replot '/home/mithun/exp0/joint_angles.txt'  u 4 w l t '{/Symbol q}_6'");
    G12.gnuplot_cmd("set terminal epslatex");
    G12.gnuplot_cmd("set output '/home/mithun/exp0/joint_angles.tex'");
    G12.gnuplot_cmd("replot");

    //Plot of camera velocity
    GP_handle G2("/usr/bin/", "Time (s)", "Camera velocity");
    G2.gnuplot_cmd("set terminal wxt");
    G2.gnuplot_cmd("plot '/home/mithun/exp0/camera velocities.txt'  u 1 w l t 'V_x'");
    G2.gnuplot_cmd("replot '/home/mithun/exp0/camera velocities.txt'  u 2 w l t 'V_y'");
    G2.gnuplot_cmd("replot '/home/mithun/exp0/camera velocities.txt'  u 3 w l t 'V_z'");
    G2.gnuplot_cmd("replot '/home/mithun/exp0/camera velocities.txt'  u 4 w l t '{/Symbol w}_x'");
    G2.gnuplot_cmd("replot '/home/mithun/exp0/camera velocities.txt'  u 5 w l t '{/Symbol w}_y'");
    G2.gnuplot_cmd("replot '/home/mithun/exp0/camera velocities.txt'  u 6 w l t '{/Symbol w}_z'");
    G2.gnuplot_cmd("set terminal epslatex");
    G2.gnuplot_cmd("set output '/home/mithun/exp0/camera_velocities.tex'");
    G2.gnuplot_cmd("replot");

    //Plot of Error in x
    GP_handle G31("/usr/bin/", "Time (s)", "Error");
    G31.gnuplot_cmd("set terminal wxt");
    G31.gnuplot_cmd("plot '/home/mithun/exp0/feature_error.txt'  u 1 w l t 'e_x'");
    G31.gnuplot_cmd("set terminal epslatex");
    G31.gnuplot_cmd("set output '/home/mithun/exp0/feature_error_x.tex'");
    G31.gnuplot_cmd("replot");

    //Plot of Error in y
    GP_handle G32("/usr/bin/", "Time (s)", "Error");
    G32.gnuplot_cmd("set terminal wxt");
    G32.gnuplot_cmd("plot '/home/mithun/exp0/feature_error.txt'  u 2 w l t 'e_y'");
    G32.gnuplot_cmd("set terminal epslatex");
    G32.gnuplot_cmd("set output '/home/mithun/exp0/feature_error_y.tex'");
    G32.gnuplot_cmd("replot");

    //Plot of Error in area
    GP_handle G33("/usr/bin/", "Time (s)", "Error");
    G33.gnuplot_cmd("set terminal wxt");
    G33.gnuplot_cmd("plot '/home/mithun/exp0/feature_error.txt'  u 3 w l t 'e_a");
    G33.gnuplot_cmd("set terminal epslatex");
    G33.gnuplot_cmd("set output '/home/mithun/exp0/feature_error_area.tex'");
    G33.gnuplot_cmd("replot");

    //Plot of Error in mean
    GP_handle G34("/usr/bin/", "Time (s)", "Error");
    G34.gnuplot_cmd("set terminal wxt");
    G34.gnuplot_cmd("plot '/home/mithun/exp0/feature_error.txt'  u 4 w l t 'error'");
    G34.gnuplot_cmd("set terminal epslatex");
    G34.gnuplot_cmd("set output '/home/mithun/exp0/feature_error_norm.tex'");
    G34.gnuplot_cmd("replot");

    //Plot of Error in x y a
    GP_handle G35("/usr/bin/", "Time (s)", "Error");
    G35.gnuplot_cmd("set terminal wxt");
    G35.gnuplot_cmd("plot '/home/mithun/exp0/feature_error.txt'  u 1 w l t 'e_x'");
    G35.gnuplot_cmd("replot '/home/mithun/exp0/feature_error.txt'  u 2 w l t 'e_y'");
    G35.gnuplot_cmd("replot '/home/mithun/exp0/feature_error.txt'  u 3 w l t 'e_a'");
    G35.gnuplot_cmd("set terminal epslatex");
    G35.gnuplot_cmd("set output '/home/mithun/exp0/feature_error_all.tex'");
    G35.gnuplot_cmd("replot");

    //Plot of Tree1 in image space
    GP_handle G4("/usr/bin/", "x-coordinate (pixels)", "y-coordinate (pixels)", "area (pixels^2)");
    G4.gnuplot_cmd("set terminal wxt");
    G4.gnuplot_cmd("set xlabel offset character 0, -1, 0");
    G4.gnuplot_cmd("set ylabel offset character 1.5, 0, 0");
    G4.gnuplot_cmd("set zlabel offset character 1, 0, 0 rotate");
    G4.gnuplot_cmd("set border");
    G4.gnuplot_cmd("set ticslevel 0");
    G4.gnuplot_cmd("set xrange [-320:320]");
    G4.gnuplot_cmd("set yrange [-320:320]");
    G4.gnuplot_cmd("set zrange [1300:3200]");
    G4.gnuplot_cmd("set xtics 320");
    G4.gnuplot_cmd("set ytics 240");
    G4.gnuplot_cmd("set ztics 500");
    G4.gnuplot_cmd("splot '/home/mithun/exp0/start.txt'  u 1:2:3 w p pt 7 ps 2 t 'Start feature', '/home/mithun/exp0/goal.txt'  u 1:2:3 w p pt 7 ps 2 t 'Goal feature'");
    G4.gnuplot_cmd("replot '/home/mithun/exp0/tree1back.txt'  u 1:2:3 w l lw 1 t 's_d', '/home/mithun/exp0/feature_executed.txt'  u 1:2:3 w l lw 1 t 's'");
    G4.gnuplot_cmd("set terminal epslatex");
    G4.gnuplot_cmd("set output '/home/mithun/exp0/trajcompare.tex'");
    G4.draw3dcoordAxis(xb, dx, true,3);

    cout << "Press enter to save plots" << endl;
    getchar();
    return 0;
}

