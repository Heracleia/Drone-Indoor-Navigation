/*
 * Ar.Drone autonomous navigation in indoor corridor
 * Created on : Dec 02, 2013
 * Author: Alexandros Lioulemes
 * OpenCV - ROS - C++ - CvBridge
 * Canny Edges - Simple Canny
 * Hough Transform - HoughLines
 * Vanishing Point - Leasrt Square Problem
 * HOG - Pedestrian detection
 * Wall centralization algorithm
 */

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <time.h>
#include <highgui.h>
#include <vector>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image window";

double x_global=0.05; // foward or back
// y>0 --> left
// y<0 --> right
double y_global=0.0;
double yaw = 0.0;
int pedestrian_flag = 0; // 0 for pedestrian not found
Point pedestrian; // position of the pedestrian in the small window (200,160)
double value ;

// Fly time
float takeoff_time=3.0;
float fly_time=15;
float land_time=3.0;
float kill_time =3.0;

// Here we create a Propotional Controller for the yaw of the ardrone
// but due to incorrect error from the unstibilaze hovering we could apply
// it correctrly.

double P_Controller(double target, double actual, double range, double gain){
	double n_target = target/range;
	double n_actual = actual/range;
	double error = abs(n_target - n_actual);
	double velocity = error*gain;
	return(velocity);
}

// Here this class handles the vanihing point and the pedestrian detection
// Also it modifies the manipulation situations of the AR.DRONE while
// capturing image from the camera and moving in the corridor.


class simplecanny{
    
	ros::NodeHandle nh_;
	ros::NodeHandle n;
	ros::Publisher pub ;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_; //image subscriber
	image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)
	std_msgs::String msg;
	
public:
    simplecanny(): it_(nh_)
    {
        image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &simplecanny::imageCb, this);
        image_pub_= it_.advertise("/arcv/Image",1);
    }
    
    ~simplecanny()
    {
        cv::destroyWindow(WINDOW);
    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        // Convert a ROS sensor_msgs/Image messages into a CvImage
        sensor_msgs::CvBridge bridge;//we need this object bridge for facilitating conversion from ros-img to opencv
        IplImage* img = bridge.imgMsgToCv(msg,"bgr8");  //image being converted from ros to opencv using cvbridge
        Mat dst, cdst,img2;
        HOGDescriptor hog;
        hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
        IplImage* out1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );   //make sure to feed the image(img) data to the parameters necessary for canny edge output
        IplImage* gray_out = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
        IplImage* canny_out = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
        IplImage* gray_out1=cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
        IplImage* img1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
        cvCvtColor(img, gray_out, CV_BGR2GRAY);
        cvSmooth(gray_out, gray_out, CV_GAUSSIAN, 9, 9);
        cvCanny( gray_out, canny_out, 50, 125, 3 );
        dst=cvarrToMat(canny_out);
        IplImage* small_img = cvCreateImage(cvSize(200,160), img->depth, img->nChannels);
        cvResize(img,small_img,CV_INTER_LINEAR);
        img2=cvarrToMat(small_img);
        vector<Vec2f> lines;
        // Find all the Hough-lines
        HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );
        vector<float> LineRho;
        vector<float> LineTheta;
        int n = 0;
        // We use this variables for measuring the mean values of the left and the right lines
        double mean_right = 0;
        int count_right =0;
        double mean_left = 0;
        int count_left =0;
        
        for( size_t i = 0; i < lines.size(); i++ ){
            float rho = lines[i][0], theta = lines[i][1];
            if (theta < 0.5) { // eliminate the orizontal lines
                continue;
            }
            if (theta > 3 && theta < 3.4) {   // eliminate vertical lines (like doors)
                continue;
            }
            LineRho.push_back(rho);
            LineTheta.push_back(theta);
            n++;
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            if(theta>1.57){ // pi/2
                count_left++;
                mean_left = mean_left+theta;
                line( dst, pt1, pt2, Scalar(255,255,0), 2, CV_AA); // (\)
            }
            else{
                count_right++;
                mean_right = mean_right+theta;
                line( dst, pt1, pt2, Scalar(255,255,0), 1, CV_AA); // (/)
            }
        }
        
        mean_left = mean_left/count_left;
        mean_right = mean_right/count_right;
        printf("Mean_left--> (\)=%g\n",mean_left);
        printf("Mean_right-->(/)=%g\n",mean_right);
        
        
        // Here we find the angle of the lines in order
        // to manipulate the AR.Drone to avoid the walls
        
        if(mean_left > 2.20 || mean_left <2.10	){
            if(mean_left > 2.20)
                y_global = 0.05;
            else
                y_global = -0.05;
        }
        else
            y_global = 0;
        
        printf("\ny_global = %g\n",y_global);
        mean_left = value;
        
        // Here we find the vanishing point
        // x = inv(AtA)Atb
        
        if(n>0){
			
            CvMat *A = cvCreateMat(n,2,CV_32FC1);
            
            // Create of matrix b [nx1]
            CvMat *b = cvCreateMat(n,1,CV_32FC1);
            
            // Populate Matrix A
            for(int j = 0; j< LineRho.size(); j++){
                cvmSet(A, j, 0, cos(LineTheta[j]));
                cvmSet(A, j, 1, sin(LineTheta[j]));
            }
            
            for(int j = 0; j<LineRho.size(); j++){
                cvmSet(b, j, 0, LineRho[j]);
            }
            
            CvMat *AT = cvCreateMat(2,n,CV_32FC1);
            
            cvTranspose(A,AT);
            
            CvMat *temp = cvCreateMat(2,2,CV_32FC1);
            CvMat *temp1 = cvCreateMat(2,2,CV_32FC1);
            CvMat *temp2 = cvCreateMat(2,n,CV_32FC1);
            CvMat *x = cvCreateMat(2,1,CV_32FC1);
            
            cvMatMul(AT,A,temp);
            cvInvert(temp,temp1,CV_LU);
            
            cvMatMul(temp1,AT,temp2);
            cvMatMul(temp2,b,x);
            
            // Print the Vanishing Point:
            printf("x:");
            for(int r = 0; r < 2; r++)
            {
                printf("%.20f\n",CV_MAT_ELEM( *x, float,  r, 0));
            }
            printf("\n");
            
            // VANISHING POINT IN THE CENTER OF OUR IMAGE
            
            double v = cvRound(CV_MAT_ELEM( *x, float,  1, 0));
            double u = cvRound(CV_MAT_ELEM( *x, float,  0, 0));
            
            if(u > 330 || u < 310){
                if(u>330) yaw =-0.1;
                else yaw = 0.1;
                
            }
            else yaw =0;
            
            printf("YAW = %g",yaw);
            int rows = dst.rows;
            int cols = dst.cols;
            
            Size s = dst.size();
            rows = s.height;
            cols = s.width;
            
            circle(dst,Point(cvRound(cols/2),cvRound(rows/2)),10,Scalar(0,255,0), 20,1, 0);
            circle(dst,Point(cvRound(CV_MAT_ELEM( *x, float,  0, 0)),cvRound(CV_MAT_ELEM( *x, float,  1, 0))),10,Scalar(255,255,0), 3, 2, 0);
        }
        
        // Here we detect the pedestrians
        
        cvCvtColor(canny_out ,gray_out1, CV_GRAY2BGR);
        imshow("detected lines", dst);
        
        vector<Rect> found, found_filtered;
        double t = (double)getTickCount();
        // run the detector with default parameters. to get a higher hit-rate
        // (and more false alarms, respectively), decrease the hitThreshold and
        // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
        hog.detectMultiScale(img2, found, 0, Size(8,8), Size(32,32), 1.05, 2);
        t = (double)getTickCount() - t;
        //printf("tdetection time = %gms\n", t*1000./cv::getTickFrequency());
        size_t i, j;
        
        for( i = 0; i < found.size(); i++ )
        {
            Rect r = found[i];
            for( j = 0; j < found.size(); j++ )
                if( j != i && (r & found[j]) == r)
                    break;
            if( j == found.size() )
                found_filtered.push_back(r);
            
            // Stop the ArDrone when sees a pedestrian
            pedestrian_flag = 1;
            //printf("\n\nWait two second\n\n");
            //sleep(2);
			
        }
        
        for( i = 0; i < found_filtered.size(); i++ )
        {
            Rect r = found_filtered[i];
            // the HOG detector returns slightly larger rectangles than the real objects.
            // so we slightly shrink the rectangles to get a nicer output.
            r.x += cvRound(r.width*0.1);
            r.width = cvRound(r.width*0.8);
            r.y += cvRound(r.height*0.07);
            r.height = cvRound(r.height*0.8);
            rectangle(img2, r.tl(), r.br(), cv::Scalar(255,255,0), 3);
            Point tl = r.tl();
            Point br = r.br();
            printf("i = %d\n",i);
            printf("Rectangle position:\n");
            printf("Top Left:(%d,%d)\n",tl.x,tl.y);
            printf("Bottom Left:(%d,%d)\n\n",br.x,br.y);
            //sleep(1);
            
            pedestrian.x = (tl.x+br.x)/2;
            pedestrian.y = (tl.y+br.y)/2;
            printf("Pedestrian:(%d,%d)\n",pedestrian.x,pedestrian.y);
            printf("pedestrian position:(%d)\n",pedestrian.x);
        }
        
        imshow("Pedestrian Detector", img2);
        
        cvWaitKey(2);
        
    }
};


geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_neg;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_up;
std_msgs::Empty emp_msg;

////////////////////////////////////////
//*************** MAIN ***************//
////////////////////////////////////////

int main(int argc, char** argv){
    
    ROS_INFO("ARdrone Test Back and Forth Starting");
    ros::init(argc, argv,"ARDrone_test");
    
    simplecanny ic;
    
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    
    ros::Publisher pub_empty_land;
    ros::Publisher pub_twist;
    ros::Publisher pub_empty_takeoff;
    ros::Publisher pub_empty_reset;
    double start_time;
    
    //hover message
    //"auto-hover" is enabled when all six components are set to zero.
    twist_msg_hover.linear.x=0.0;
    twist_msg_hover.linear.y=0.0;
    twist_msg_hover.linear.z=0.0;
    twist_msg_hover.angular.x=0.0;
    twist_msg_hover.angular.y=0.0;
    twist_msg_hover.angular.z=0.0;
    //up message
    twist_msg_up.linear.x=0.0;
    twist_msg_up.linear.y=0.0;
    twist_msg_up.linear.z=0.1;
    twist_msg_up.angular.x=0.0;
    twist_msg_up.angular.y=0.0;
    twist_msg_up.angular.z=0.0;
    //command message
    
    twist_msg.linear.x = x_global;
    // y>0 --> left
    // y<0 --> right
    twist_msg.linear.y = y_global;
    twist_msg.linear.z=0.0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y= 0.0;
    twist_msg.angular.z=yaw;
    
    twist_msg_neg.linear.x=-twist_msg.linear.x;
    twist_msg_neg.linear.y=-twist_msg.linear.y;
    twist_msg_neg.linear.z=-twist_msg.linear.z;
    twist_msg_neg.angular.x=-twist_msg.angular.x;
    twist_msg_neg.angular.y=-twist_msg.angular.y;
    twist_msg_neg.angular.z=-twist_msg.angular.z;
    
    pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
    pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
    pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
    pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */
    
    start_time =(double)ros::Time::now().toSec();
    ROS_INFO("Starting ARdrone_test loop");
    
    while (ros::ok()) {
        while ((double)ros::Time::now().toSec()< start_time+takeoff_time){ //takeoff
            
            pub_empty_takeoff.publish(emp_msg); //launches the drone
            pub_twist.publish(twist_msg_hover); //drone is flat
            ROS_INFO("Taking off");
            ros::spinOnce();
            loop_rate.sleep();
        }//while takeoff
        
        while  ((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time){
            
            pub_twist.publish(twist_msg_hover); //drone is flat
            pub_empty_land.publish(emp_msg); //lands the drone
            ROS_INFO("Landing");
            
            
            if ((double)ros::Time::now().toSec()> takeoff_time+start_time+fly_time+land_time+kill_time){
                ROS_INFO("Closing Node");
                //pub_empty_reset.publish(emp_msg); //kills the drone
                exit(0);
            }//kill node
            ros::spinOnce();
            loop_rate.sleep();
        }//while land
        
        while ( (double)ros::Time::now().toSec()> start_time+takeoff_time && (double)ros::Time::now().toSec()< start_time+takeoff_time+fly_time){
            if((double)ros::Time::now().toSec()< start_time+takeoff_time+fly_time){
                pub_twist.publish(twist_msg);
                ROS_INFO("Flying +ve");
            }//fly according to desired twist
            
            
            // Here we try to avoid pedestrian regarding his
            // position in the pedestrian detection window of
            // size (200, 160)
            
            if(pedestrian_flag==1){
                printf("Petestrian  Detection");
                if(pedestrian.x<80){ // pedestrian is on the left
                    y_global=-0.25; // ardrone go right
                    printf("\nARDRONE go right\n");
                }
                if(pedestrian.x>120){ // pedestrian is on the right
                    y_global=0.25;	// ardrone go left
                    printf("\nARDRONE go left\n");
                }
                
                if(pedestrian.x > 80 && pedestrian.x < 120){
                    if(value > 2.20){
                        y_global = 0.15;
                        printf("\nARDRONE go left\n");
                    }
                    if(value < 2.10){
                        y_global = -0.15;
                        printf("\nARDRONE go right\n");
                    }
                    printf("Avoid pedestrian.\n");
                }
                
                pedestrian_flag=0;
                
            }
            
            twist_msg.linear.x = x_global;
            // y>0 --> left
            // y<0 --> right 
            twist_msg.linear.y = y_global;
            twist_msg.linear.z=0.0;
            twist_msg.angular.x=0.0; 
            twist_msg.angular.y= 0.0;
            twist_msg.angular.z=yaw;
            printf("\nFlying time = %g\n",fly_time);
            
            ros::spinOnce();
            loop_rate.sleep();
        }
        ros::spinOnce();
        loop_rate.sleep();
        
    }//ros::ok
}
