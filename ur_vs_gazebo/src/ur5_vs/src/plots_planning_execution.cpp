
#include <iostream>
#include <sstream>
#include <string>
#include <gnuplot_ci.h>
#include <stdlib.h>

/*--------plot-------*/

using namespace gnuplot_ci;
using namespace std;
double xb[3] = {0,0,0};
double dx[3] = {1, 1, 1};

int main(int argc, char** argv)
{
    //plot tree1 RRT
    GP_handle G4("/usr/bin/", "x-coordinate (pixel)", "y-coordinate (pixel)", "area (pixel^2)");
    G4.gnuplot_cmd("set terminal wxt");
    G4.gnuplot_cmd("set xlabel offset character 0, -1, 0");
    G4.gnuplot_cmd("set ylabel offset character 2, 0, 0");
    G4.gnuplot_cmd("set zlabel offset character 0.5, 0, 0 rotate");
    G4.gnuplot_cmd("set border");
    G4.gnuplot_cmd("set ticslevel 0");
    G4.gnuplot_cmd("set xrange [-320:320]");
    G4.gnuplot_cmd("set yrange [-320:320]");
    G4.gnuplot_cmd("set zrange [1300:3200]");
    G4.gnuplot_cmd("set xtics 320");
    G4.gnuplot_cmd("set ytics 240");
    G4.gnuplot_cmd("set ztics 500");
    G4.gnuplot_cmd("splot '/home/mithun/exp0_sectask/start.txt'  u 1:2:3 w p pt 7 ps 2 t 'Start feature', '/home/mithun/exp0_sectask/goal.txt'  u 1:2:3 w p pt 7 ps 2 t 'Goal feature'");
    G4.gnuplot_cmd("replot '/home/mithun/exp0_sectask/tree1.txt'  u 1:2:3 w p pt 0 t 'Tree_1'");
    G4.gnuplot_cmd("replot '/home/mithun/exp0_sectask/tree1back.txt'  u 1:2:3 w l lw 1 t 's_d'");
    G4.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 1.5 font 'Arial,22'");
    G4.gnuplot_cmd("set output '/home/mithun/exp0_sectask/tree1.eps'");
    G4.draw3dcoordAxis(xb, dx, true,2);

//    //Plot of Joint velocity
//    GP_handle G1("/usr/bin/", "Time (s)", "Joint velocity (rad/s)");
//    G1.gnuplot_cmd("set terminal wxt");
//    G1.gnuplot_cmd("plot '/home/mithun/exp0_sectask/exp0/tree2.txt'  u 6 w l t 'dth_1'");
//    G1.gnuplot_cmd("replot '/home/mithun/exp0_sectask/exp0/tree2.txt'  u 7 w l t 'dth_2'");
//    G1.gnuplot_cmd("replot '/home/mithun/exp0_sectask/exp0/tree2.txt'  u 8 w l t 'dth_3'");
//    G1.gnuplot_cmd("replot '/home/mithun/exp0_sectask/exp0/tree2.txt'  u 11 w l t 'dth_4'");
//    G1.gnuplot_cmd("replot '/home/mithun/exp0_sectask/exp0/tree2.txt'  u 10 w l t 'dth_5'");
//    G1.gnuplot_cmd("replot '/home/mithun/exp0_sectask/exp0/tree2.txt'  u 9 w l t 'dth_6'");
//    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 2 font 'Arial-Bold,18'");
//    G1.gnuplot_cmd("set output '/home/mithun/exp0_sectask/exp0/tree2jointvel.eps'");
//    G1.gnuplot_cmd("replot");

    cout << "Press enter to save plots" << endl;
    getchar();
}





