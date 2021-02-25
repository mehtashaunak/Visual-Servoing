// Function for reading textfile
// Written by Deepak Raina (EMP ID: 1445893)

/* Syntax example:
 int rows=100, cols=10;
 string filename="abc.txt";
 cv::Mat PP = cv::Mat(rows, cols, CV_64F, 0.0);
 read_text_file(filename,rows,cols,PP);
 cout << "PP " << PP << endl;
*/

#include <iostream>
#include <fstream>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;

void read_text_file(string filename, int rows, int cols, cv::Mat &P){

    vector<double> numbers;

    ifstream points;
    points.open(filename.c_str());

//    cout<<"reading textfile"<<endl;
    double number;  //Variable to hold each number as it is read

    //Read number using the extraction (>>) operator
    while (points >> number) {
        //Add the number to the end of the array
//        cout<<"number"<<number<<endl;
        numbers.push_back(number);
//        cout << number;
//        getchar();
    }


    points.close();
//    cout <<  "No. of elements = " << numbers.size() << endl;
//    getchar();

    int i=0;
    for (int j=0; j<rows; j++)
    {
        for (int k=0; k<cols; k++)
        {
            P.at<double>(j,k)=numbers[i];
            i++;
        }
    }
}


