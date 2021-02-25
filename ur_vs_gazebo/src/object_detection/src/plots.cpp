#include <iostream>
#include <gnuplot_ci.h>
#include <stdlib.h>

/*--------plot-------*/

using namespace gnuplot_ci;
using namespace std;

//double xb[3] = {0,0,0};
//double dx[3] = {1, 1, 1};

int main(int argc, char** argv)
{
    //Plot of tracked features
    GP_handle G1("/usr/bin/", "x (m)", "y(m)");
    G1.gnuplot_cmd("set terminal wxt");
    G1.gnuplot_cmd("plot '/home/mithun/ur5_visual_servoing/src/object_detection/src/results/camshiftfeatures.txt' u 1:2 w l t 'camshift'");
    G1.gnuplot_cmd("replot '/home/mithun/ur5_visual_servoing/src/object_detection/src/results/kalmanfeatures.txt' u 1:2 w l t 'kalman'");
    G1.gnuplot_cmd("set terminal postscript eps enhanced color solid lw 1 font 'Arial,22'");
    G1.gnuplot_cmd("set output '/home/mithun/ur5_visual_servoing/src/object_detection/src/results/featurescompare.txt'");
    G1.gnuplot_cmd("replot");

    cout << "Press enter to save plots" << endl;
    getchar();
    return 0;
}

