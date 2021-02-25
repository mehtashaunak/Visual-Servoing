
/******************************************
 * OpenCV Tutorial: Ball Tracking using   *
 * Kalman Filter                          *
 ******************************************/

// Module "core"
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

// Module "video"
#include <opencv2/video/video.hpp>
#include<opencv2/features2d/features2d.hpp>

// Output
#include <iostream>
#include <stdlib.h>

// Vector
#include <vector>

//ros
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include <object_detection/image_data.h>


using namespace std;
using namespace cv;

// >>>>> Color to be tracked
//#define MIN_H_BLUE 167
//#define MAX_H_BLUE 237
//#define MIN_H_BLUE 200
//#define MAX_H_BLUE 300
// <<<<< Color to be tracked
//int vmin = 167, vmax = 237, smin = 146;
//int vmin = 86, vmax = 196, smin = 116;
//int vmin = 84, vmax = 202, smin = 152;
//int vmin = 138, vmax = 242, smin = 152;
//int vmin = 205, vmax = 255, smin = 162;
//int vmin = 204, vmax = 256, smin = 216;
int vmin = 106, vmax = 225, smin = 153;

//int vmin = 255, vmax = 255, smin = 0;
//int iLowH =3; int iHighH=15; int iLowS = 88; int iHighS =100; int iLowV = 50; int iHighV = 70;

int iLastX = 0;
int iLastY = 0;
int predict_x = 0;
int predict_y = 0;

Mat image, imgTh;
bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
bool paused= false;
Mat object;
char c;
double xc, yc = 0.0;
static void onMouse( int event, int x, int y, int, void* )
{
    if( selectObject )
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);

        selection &= Rect(0, 0, image.cols, image.rows);

    }

    switch( event )
    {
    case EVENT_LBUTTONDOWN:
        origin = Point(x,y);
        selection = Rect(x,y,0,0);

        selectObject = true;
        break;
    case EVENT_LBUTTONUP:
        selectObject = false;
        if( selection.width > 0 && selection.height > 0 )
            trackObject = -1;   // Set up CAMShift properties in main() loop
        break;
    }
}

void MyFilledCircle( Mat img, Point center , int r,int g, int b)
{
    int thickness = 5;
    int lineType = 8;

    circle( img,
            center,
            4,
            Scalar( r, g, b ),
            thickness,
            lineType );
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}


void ptCloudcallBack(const sensor_msgs::PointCloud2::ConstPtr &input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud, bool &flag)
{

    pcl::fromROSMsg( *input, *ptCloud );
    flag = true;
}


int main(int argc, char ** argv)
{
    // Camera frame
    Mat frame, hsv, hue, hist, histimg = Mat::zeros(200, 320, CV_8UC3),backproj;
    Mat mask =  Mat::zeros(480, 848, CV_8UC3);
    //    Mat mask(200,320,CV_8U, Scalar(0,0,0));
    //    imshow("mask", mask);
    //    cv::waitKey(0);
    double area;
    Mat imgLines = Mat::zeros(image.size(), CV_8UC3);
    Rect trackWindow;
    int hsize = 16;
    float hranges[] = {0,180};
    const float* phranges = hranges;
    vector<Vec4i> hierarchy;
    vector<vector<Point> > contours_blue;
    Point2f  rect_points[4];
    double uu[8];

    // >>>> Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
    //cv::Mat procNoise(stateSize, 1, type)
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    //Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    ros::init(argc, argv, "object_detect");
    ros::NodeHandle handle1;
    image_transport::ImageTransport img_trans(handle1);
    bool flag_ptcloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    ros::Publisher pub = handle1.advertise<object_detection::image_data>("/object_points", 10);
    object_detection::image_data data;
    image_transport::Subscriber sub_img1 = img_trans.subscribe("/camera/rgb/image_raw",1,imageCallback);
    sleep(1);

    ros::Subscriber sub = handle1.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, boost::bind(ptCloudcallBack, _1, boost::ref(ptcloud),boost::ref(flag_ptcloud)));
    sleep(1);
    ros::spinOnce();

    //file for writing image features
    ofstream f1, f2;
    f1.open("/home/mithun/ur5_visual_servoing/src/object_detection/src/results/camshiftfeatures.txt",std::ios_base::trunc);
    f2.open("/home/mithun/ur5_visual_servoing/src/object_detection/src/results/kalmanfeatures.txt",std::ios_base::trunc);


    double ticks = 0;
    bool found = false;

    int notFoundCount = 0;
    double processedImages = 0;
    bool startProcessingImage = false;
    double time, total_time = 10.0;
    time = ros::Time::now().toSec();

    // >>>>> Main loop
    while(ros::ok())
    {
        pcl::PointXYZRGB pt;
        double precTick = ticks;
        ticks = (double) cv::getTickCount();
        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        ros::spinOnce();
        if( !paused )
        {
            // image = frame;
            if( image.empty() )
                break;
        }

        //        frame.copyTo( image );
        if(found)
        {
            cout << "inside prediction " << endl;
            //getchar();
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A

            cout << "dT:" << endl << dT << endl;

            state = kf.predict();
            //cout << "State post:" << endl << state << endl;

            cv::Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width/ 2;
            predRect.y =state.at<float>(1) - predRect.height / 2;

            cv::Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);

            cv::circle(image, center, 2, CV_RGB(255,0,0), -1);
            cv::rectangle(image, predRect, CV_RGB(255,0,0), 2);

        }

        namedWindow( "Histogram", 0 );
        namedWindow( "CamShift Demo", 0 );
        namedWindow("Control", CV_WINDOW_AUTOSIZE);
        setMouseCallback( "CamShift Demo", onMouse, 0 );

        createTrackbar( "Vmin", "Control", &vmin, 256, 0 );
        createTrackbar( "Vmax", "Control", &vmax, 256, 0 );
        createTrackbar( "Smin", "Control", &smin, 256, 0 );
        //            createTrackbar("LowH", "Control", &iLowH, 179);
        //            createTrackbar("HighH", "Control", &iHighH,179);
        //            createTrackbar("LowS", "Control", &iLowS, 255);
        //            createTrackbar("HighS", "Control", &iHighS,255);
        //            createTrackbar("LowV", "Control", &iLowV, 255);
        //            createTrackbar("HighV", "Control", &iHighV,255);


        if( !paused )
        {
            cvtColor(image, hsv, COLOR_BGR2HSV);
            vector<vector<cv::Point> > contours;
            if( trackObject )
            {
                //cout << "Before for" << endl;
                int _vmin = vmin, _vmax = vmax;
                inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),Scalar(180, 256, MAX(_vmin, _vmax)), mask);
                //                      inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), mask);

                erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
                dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
                dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
                erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

                mask.copyTo(imgTh);
                imshow("Mask",mask);
                findContours(imgTh, contours_blue, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
                vector<RotatedRect> minRect( contours_blue.size() );

                for( int i = 0; i < contours_blue.size(); i++ )
                {
                    minRect[i] = minAreaRect( Mat(contours_blue[i]) );
                    minRect[i].points( rect_points );
                }
                int j=0;
                for (int i=0;i<4;i++)
                {
                    uu[j]=rect_points[i].x;
                    uu[j+1]=rect_points[i].y;
                    j=j+2;
                }
                j=0;
                //                for (int i=0;i<4;i++)
                //                {
                //                    MyFilledCircle( image, Point(uu[j], uu[j+1]),0,0,255 );
                //                    j=j+2;
                //                }

                Moments oMoments = moments(mask);

                double dM01 = oMoments.m01;
                double dM10 = oMoments.m10;
                double dArea = oMoments.m00;

                area = dArea;

                if(dArea >10000)
                {
                    int posX = dM10 / dArea;
                    int posY = dM01/dArea;


                    if(iLastX >= 0 && iLastY >=0 && posY >=0)
                    {
                        line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255),2);

                    }

                    iLastX = posX;
                    iLastY = posY;
                }


                int ch[] = {0, 0};
                hue.create(hsv.size(), hsv.depth());
                mixChannels(&hsv, 1, &hue, 1, ch, 1);

                if( trackObject < 0 )
                {
                    // Object has been selected by user, set up CAMShift search properties once
                    Mat roi(hue, selection), maskroi(mask, selection);
                    calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
                    normalize(hist, hist, 0, 255, NORM_MINMAX);

                    trackWindow = selection;
                    trackObject = 1; // Don't set up again, unless user selects new ROI
                    startProcessingImage = true;
                    time = ros::Time::now().toSec();

                    histimg = Scalar::all(0);
                    int binW = histimg.cols / hsize;
                    Mat buf(1, hsize, CV_8UC3);
                    for( int i = 0; i < hsize; i++ )
                        buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);
                    cvtColor(buf, buf, COLOR_HSV2BGR);

                    for( int i = 0; i < hsize; i++ )
                    {
                        int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
                        rectangle( histimg, Point(i*binW,histimg.rows),
                                   Point((i+1)*binW,histimg.rows - val),
                                   Scalar(buf.at<Vec3b>(i)), -1, 8 );
                    }

                }

                cv::findContours(mask, contours, CV_RETR_EXTERNAL,
                                 CV_CHAIN_APPROX_NONE);
                cout<<"Countour size "<<contours.size()<<"\n";
                // >>>>> Detection result

                if(contours.size()!=0)
                {
                    cout << "Performing camshift" << endl;
                    calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
                    backproj &= mask;

                    cout <<"before camshift"<<endl;


                    RotatedRect trackBox = CamShift(backproj, trackWindow,
                                                    TermCriteria( TermCriteria::EPS | TermCriteria::COUNT, 10, 1 ));

                    cout << "centre from cam shift " << trackBox.size <<" "<< trackBox.center.x<< " " << trackBox.center.y << endl;
                    cout <<"after camshift"<<endl;

                    if( trackWindow.area() <= 1 )
                    {
                        int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
                        trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                                           trackWindow.x + r, trackWindow.y + r) &
                                Rect(0, 0, cols, rows);
                        ROS_INFO("out of camera scope");


                    }

                    if( backprojMode )
                        cvtColor( backproj, image, COLOR_GRAY2BGR );
                    ellipse( image, trackBox, Scalar(255,0,0), 3, CV_AA );

                }
                else
                {
                    cout << "Performing Kalman tracking" << endl;
                    for (size_t i = 0; i < contours.size(); i++)
                    {
                        cv::drawContours(image,contours, i, CV_RGB(0,255,0), 3);
                        cv::Rect initRect;
                        initRect.width = selection.width;
                        initRect.height = selection.height;
                        initRect.x = iLastX - initRect.width / 2;
                        initRect.y = iLastY - initRect.height / 2;
                        rectangle(image, initRect, CV_RGB(0,255,0),2);
                        cv::Point center;
                        center.x = iLastX;
                        center.y = iLastY;
                        cv::circle(image, center, 2, CV_RGB(20,150,20), 2);
                        stringstream sstr;
                        sstr << "(" << center.x << "," << center.y << ")";
                        cv::putText(image, sstr.str(),
                                    cv::Point(center.x + 3, center.y - 3),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
                    }

                }
            }
            else if( trackObject < 0 )
                paused = false;


            // <<<<< Detection result

            if( selectObject && selection.width > 0 && selection.height > 0 )
            {
                Mat roi(image, selection);
                bitwise_not(roi, roi);
                startProcessingImage = true;

            }


            // >>>>> Kalman Update
            if (contours.size() == 0)
            {
                notFoundCount++;
                cout << "notFoundCount:" << notFoundCount << endl;
                if( notFoundCount >= 1000 )
                {
                    found = false;
                }
                cout << "out of range" << endl;
                //                else
                //                kf.statePost = state;
            }
            else
            {
                notFoundCount = 0;

                meas.at<float>(0) = iLastX;
                meas.at<float>(1) = iLastY;
                meas.at<float>(2) = selection.width;
                meas.at<float>(3) = selection.height;
                //                meas.at<float>(0) = bBox.x + bBox.width/2;
                //                meas.at<float>(1) = bBox.y + bBox.height/2;
                //                meas.at<float>(2) = bBox.width;
                //                meas.at<float>(3) = bBox.height;


                if (!found) // First detection!
                {
                    cout << "kalman detection " << endl;
                    //getchar();
                    // >>>> Initialization
                    kf.errorCovPre.at<float>(0) = 1; // px
                    kf.errorCovPre.at<float>(7) = 1; // px
                    kf.errorCovPre.at<float>(14) = 1;
                    kf.errorCovPre.at<float>(21) = 1;
                    kf.errorCovPre.at<float>(28) = 1; // px
                    kf.errorCovPre.at<float>(35) = 1; // px

                    state.at<float>(0) = meas.at<float>(0);
                    state.at<float>(1) = meas.at<float>(1);
                    state.at<float>(2) = 0;
                    state.at<float>(3) = 0;
                    state.at<float>(4) = meas.at<float>(2);
                    state.at<float>(5) = meas.at<float>(3);

                    // <<<< Initialization

                    kf.statePost = state;

                    found = true;
                }
                else
                    kf.correct(meas); // Kalman Correction

                cout << "Measure matrix:" << endl << meas << endl;
            }
            imshow( "CamShift Demo", image );
            imshow( "Histogram", histimg );
            // <<<<< Kalman Update

            // Final result
            // cv::imshow("Tracking", mask);

            // User key
            cv::waitKey(1);

            if (contours.size()==0)
            {
                xc = state.at<float>(0);
                yc = state.at<float>(1);
                double m[2];
                m[0] = (xc-320)/(531.15/640);    //(u-u0)/px
                m[1] = (yc-240)/(531.15/480);    //(v-v0)/py50
                if (trackObject)
                    f2 << m[0] << "\t" << m[1] << endl;
            }
            else
            {
                xc = iLastX;
                yc = iLastY;
                double m[2];
                m[0] = (xc-320)/(531.15/640);    //(u-u0)/px
                m[1] = (yc-240)/(531.15/480);    //(v-v0)/py50
                if (trackObject)
                    f1 << m[0] << "\t" << m[1] << endl;
            }

        }






        //        cout<<"Test1"<<endl;
        //        predict_x=state.at<float>(0);
        //        cout<<"Test1"<<endl;
        //        predict_y=state.at<float>(1);
        //        cout<<"Test1"<<endl;
        //        cout << "centroid (pixel) " << predict_x << " " << predict_y << endl;
        //        cout<< "point cloud size: " << ptcloud->width << "height:" << ptcloud->height << endl;
        //        if((predict_x< 0 || predict_x > 640 )|| (predict_y <0 || predict_y >480))
        //        {
        //            cout << "out of range" << endl;
        //            pt.z = 0.7;
        //        }
        //        else
        //            pt = ptcloud ->at(predict_x, predict_y);


        cout << "centroid (pixel) " << xc << " " << yc << endl;
        cout<< "point cloud size: " << ptcloud->width << "height:" << ptcloud->height << endl;
        if((xc< 0 || xc > 640 )|| (yc <0 || yc >480))
        {
            cout << "out of range" << endl;
            pt.z = 0.7;
        }
        else
            //                    pt = ptcloud ->at(predict_x, predict_y);
            pt = ptcloud ->at(xc, yc);

        //        cout<<"Test1"<<endl;

        //Centroif in image coordinates
        double m[2];
        //        m[0] = (predict_x-320)/(531.15/640);    //(u-u0)/px
        //        m[1] = (predict_y-240)/(531.15/480);    //(v-v0)/py50

        m[0] = (xc-320)/(531.15/640);    //(u-u0)/px
        m[1] = (yc-240)/(531.15/480);    //(v-v0)/py50

        cout << "depth = " << (float)pt.z<<endl;
        cout << "centroid (pixel) " << xc<< " " << yc << endl;
        cout << "centroid (image) " << m[0] << " " << m[1] << endl;
        //        cout << "centroid (image_thresholding) " << iLastX << " " << iLastY << endl;

        //        data.detected_point_x.data = predict_x;
        //        data.detected_point_y.data = predict_y;

        data.detected_point_x.data = xc;
        data.detected_point_y.data = yc;
        data.detected_point_depth.data =(float)pt.z;

        //for checking number of processed images
        if (startProcessingImage)
        {
            double duration = ros::Time::now().toSec() -time ;
            cout << "Duration: " << duration << endl;
            processedImages++;
            cout << "Processed Images: " << processedImages << endl;


            if(duration>total_time)
            {
                cout << "Processed Images in "<< total_time << " sec = " << processedImages << endl;
                cout << "Processed Images in 1.0 sec = " << processedImages/total_time << endl;
                //break;
            }
        }

        pub.publish(data);
        ros::spinOnce();
        // <<<<< Main loop
    }
    f1.close();
    f2.close();

    return EXIT_SUCCESS;
}
