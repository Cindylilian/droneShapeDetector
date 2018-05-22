#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "ardrone_autonomy/Navdata.h"
#include <geometry_msgs/Vector3.h>
#include <termios.h>
#include <cv.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui_c.h"
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

geometry_msgs::Twist hover;
geometry_msgs::Twist hoverUp;
geometry_msgs::Twist hoverDown;
geometry_msgs::Twist trackLeft;
geometry_msgs::Twist trackRight;
geometry_msgs::Twist trackForward;
geometry_msgs::Twist trackBackward;
geometry_msgs::Twist moveRight;
geometry_msgs::Twist moveLeft;
geometry_msgs::Twist moveForward;
geometry_msgs::Twist moveBackward;
geometry_msgs::Twist hoverStop;
geometry_msgs::Twist hoverShape;
std_msgs::Empty msg;

static struct termios initial_settings, new_settings;
static int peek_character = -1;

//data navigasi
ardrone_autonomy::Navdata msg_in;
int drone_altd;
float battery;
int rotX, rotY, rotZ, vx, vy, vz, motor1, motor2, motor3, motor4, ax, ay, az; 

//kondisi
bool showData = false;
bool baterai = false;
bool gotShape = false;

bool otomatis = false;
bool automateHeight = false;

bool isLanding = false;
bool isMoving = false;
bool isTrack = false;

int nomerObjek;

double koorXShape, koorYShape;
char command = '~';
char commandObjek = '~';

Point CoordShape;
string tipeObjek;
string gerakan;

static const char WINDOW[]="SRC Image"; 
static const char WINDOW2[]="DST Image";
static const char WINDOW3[]="Black White Image";
static const char WINDOW4[]="Grayscale Image";
static const char WINDOW5[]="Blur";


/**
 * Helper function to display text in the center of a contour
 */
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);
	gotShape = true;
	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	CoordShape.x = r.x + (r.width/ 2);
	CoordShape.y = r.y + (r.height/ 2);
	tipeObjek = label;
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}


void process(const sensor_msgs::ImageConstPtr& cam_image){
	
	cv::Mat src;
	cv::Mat gray;
	cv::Mat bw;
	cv::Mat dst;
    cv::Mat blurimg;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Point> approx;

	// kamera drone cv bridge
	cv_bridge::CvImagePtr bridge;
	bridge = cv_bridge::toCvCopy(cam_image, sensor_msgs::image_encodings::BGR8);
	gray = bridge->image;

	resize(gray,src,Size(320,240));
	
	// //kamera
	// VideoCapture capture(0);
	// capture >> src;

	cv::cvtColor(src, gray, CV_BGR2GRAY);

	// Use Canny instead of threshold to catch squares with gradient shading
	blur( gray, bw, Size(3,3) );
    blur( gray, blurimg, Size(3,3) );
	cv::Canny(gray, bw, 130, 250, 3);

	// Find contours
	cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	src.copyTo(dst);

	for (int i = 0; i < contours.size(); i++)
	{
		// Approximate contour with accuracy proportional
		// to the contour perimeter
		cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

		// Skip small or non-convex objects 
		if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
			continue;
        
        
        // Number of vertices of polygonal curve
        int vtc = approx.size();
        
        
        // Use the number of vertices
        // to determine the shape of the contour
		if (vtc == 3 && nomerObjek == 3)
		{
			setLabel(dst, "TRI", contours[i]);    // Segitiga
		}
		else if (vtc >= 4 && nomerObjek == 4)
		{
            setLabel(dst, "RECT", contours[i]); //Kotak
		}
        else if (vtc >= 5 && nomerObjek == 5)
        {
            setLabel(dst, "PENTA", contours[i]); //Segilima
        }
		else
		{
			gotShape = false; //Objek tidak ditemukan
			CoordShape.x = 999;
			CoordShape.y = 999;
			tipeObjek = "";
		}
	}

	int droneHeight = drone_altd/10; // satuan centimeter

	
	if(otomatis && !isLanding){
		if(!gotShape && isMoving){
			if (gerakan == "Maju"){
				command = '^';
			}else if (gerakan == "Mundur"){
				command = 'v';
			}else if (gerakan == "Kiri"){
				command = '<';
			}else if (gerakan == "Kanan"){
				command = '>';
			}else if (gerakan == "Naik"){
				command = '(';
			}else if (gerakan == "Turun"){
				command = ')';
			}
		}else if(gotShape && !isTrack){
			cout<<"Objek telah terdeteksi";
			isMoving = false;
			isTrack = true;
		}else if(isTrack){
			
			if(!gotShape){
				//Hover jika tidak menemui objek
                char Areas[20];
                sprintf(Areas,"Hover");
                putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
		     	command = '5';
			}
			//kotak kiri 3
			 else if (CoordShape.x>0 && CoordShape.x<110 && CoordShape.y >0 && CoordShape.y <80 ) {
                 //geser kiri jika kotak kiri atas
                 char Areas[20];
                 sprintf(Areas,"Geser Kiri");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '3';
             } else if (CoordShape.x>0 && CoordShape.x<110 && CoordShape.y >80 && CoordShape.y <160 ) {
                 //geser kiri jika kotak kiri tengah
                 char Areas[20];
                 sprintf(Areas,"Geser Kiri");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '3';
             } else if (CoordShape.x>0 && CoordShape.x<110 && CoordShape.y >160 && CoordShape.y <240 ) {
                 //geser kiri jika kotak kiri bawah
                 char Areas[20];
                 sprintf(Areas,"Geser Kiri");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '3';
                 
             }
            //kotak tengah 3
             else if (CoordShape.x>110 && CoordShape.x<220 && CoordShape.y >0 && CoordShape.y <80 ) {
                 //maju depan jika kotak tengah atas
                 char Areas[20];
                 sprintf(Areas,"Maju Depan");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '7';
             } else if (CoordShape.x>110 && CoordShape.x<220 && CoordShape.y >80 && CoordShape.y <160 ) {
                 //landing jika koordinat titik tengah objek berada di kotak tengah
                 char Areas[20];
                 sprintf(Areas,"Landing");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '6';
                 isLanding = true;
                 isMoving = false;
             } else if (CoordShape.x>110 && CoordShape.x<220 && CoordShape.y >160 && CoordShape.y <240 ) {
                 //mundur ke belakang jika kotak tengah bawah
                 char Areas[20];
                 sprintf(Areas,"Mundur Belakang");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '8';
             }
            //kotak kanan 3
             else if (CoordShape.x>220 && CoordShape.x<330 && CoordShape.y >0 && CoordShape.y <80 ) {
                 //geser kanan jika kotak kanan atas
                 char Areas[20];
                 sprintf(Areas,"Geser Kanan");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '4';
             } else if (CoordShape.x>220 && CoordShape.x<330 && CoordShape.y >80 && CoordShape.y <160 ) {
                 //geser kanan jika kotak kanan tengah
                 char Areas[20];
                 sprintf(Areas,"Geser Kanan");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '4';
             } else if (CoordShape.x>220 && CoordShape.x<330 && CoordShape.y >160 && CoordShape.y <240 ) {
                 //geser kanan jika kotak kanan bawah
                 char Areas[20];
                 sprintf(Areas,"Geser Kanan");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '4';
             } else if (CoordShape.x == 999 && CoordShape.y == 999){
                 //Hover jika tidak menemui objek
                 char Areas[20];
                 sprintf(Areas,"Hover");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '5';
                 
             } else{
                 //Hover jika tidak menemui objek
                 char Areas[20];
                 sprintf(Areas,"Hover");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '5';
             }
        }
    } else if(isLanding){
        command = '6';
        if (tipeObjek != ""){
            cout<<"Landing pada Objek: "<<tipeObjek<<endl;
        }
        isLanding = false;
    }
    
    //----------------------------vertical----------------------------------//
    line( dst, Point( 110,0 ), Point( 110,240), Scalar( 0, 255, 0),  1, 8 );
    line( dst, Point( 220,0 ), Point( 220,240), Scalar( 0, 255, 0),  1, 8 );
    //---------------------------horizontal---------------------------------//
    line( dst, Point( 0,80 ), Point( 330,80), Scalar( 0, 255, 0),  1, 8 );
    line( dst, Point( 0,160 ), Point( 330,160), Scalar( 0, 255, 0),  1, 8 );


	if(showData){
		cout<<"Height : "<<drone_altd/10<<endl;
	}

	cv::imshow(WINDOW,src);
	cv::imshow(WINDOW2,dst);
	cv::imshow(WINDOW3,bw);
    cv::imshow(WINDOW4,gray);
    cv::imshow(WINDOW5,blurimg);
	cvWaitKey(1);
}


void init_keyboard()
{
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

int kbhit()
{
    char ch;
    int nread;

    if(peek_character != -1)
        return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);

    if(nread == 1) {
        peek_character = ch;
        return 1;
    }
    return 0;
}

int readch()
{
    char ch;

    if(peek_character != -1) {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}

void dataNavigasi(const ardrone_autonomy::Navdata& msg_in)
{
    drone_altd = msg_in.altd;
    rotX = msg_in.rotX;
    rotY = msg_in.rotY;
    rotZ = msg_in.rotZ;
    vx = msg_in.vx;
    vy = msg_in.vy;
    vz = msg_in.vz;
    motor1 = msg_in.motor1;
    motor2 = msg_in.motor2;
    motor3 = msg_in.motor3;
    motor4 = msg_in.motor4;
    ax = msg_in.ax;
    ay = msg_in.ay;
    az = msg_in.az;
    battery = msg_in.batteryPercent;
}


int main(int argc, char **argv){
    ros::init(argc, argv,"shapeDetection");
    ros::NodeHandle node;
    ros::NodeHandle n;
    ros::Rate loop_rate(50);
    ros::Subscriber sub_navdata;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/bottom/image_raw",1,process);
    
    sub_navdata = node.subscribe("/ardrone/navdata",1,dataNavigasi);

    ros::Publisher pub_empty_takeoff;
    ros::Publisher pub_empty_land;
    ros::Publisher pub_empty_reset;
    ros::Publisher pub_twist;

    hoverStop.linear.x=0.0;
    hoverStop.linear.y=0.0;
    hoverStop.linear.z=0.0;
    hoverStop.angular.x=0.0;
    hoverStop.angular.y=0.0;
    hoverStop.angular.z=0.0;

    hoverUp.linear.x=0.0;
    hoverUp.linear.y=0.0;
    hoverUp.linear.z=0.1;
    hoverUp.angular.z=0.0;
    
    hoverDown.linear.x=0.0;
    hoverDown.linear.y=0.0;
    hoverDown.linear.z=-0.1;
    hoverDown.angular.z=0.0;
    
    trackLeft.linear.x=0.0;
    trackLeft.linear.y=0.02; 
    trackLeft.linear.z=0.0;
    trackLeft.angular.z=0.0;
    
    trackRight.linear.x=0.0;
    trackRight.linear.y=-0.02;
    trackRight.linear.z=0.0;
    trackRight.angular.z=0.0;
    
    trackForward.linear.x=0.02;
    trackForward.linear.y=0.0;
    trackForward.linear.z=0.0;
    trackForward.angular.z=0.0;
    
    trackBackward.linear.x=-0.02;
    trackBackward.linear.y=-0.0;
    trackBackward.linear.z=0.0;
    trackBackward.angular.z=0.0;
    
    moveRight.linear.x=0.0;
    moveRight.linear.y=-0.03;
    moveRight.linear.z=0.0;
    moveRight.angular.z=0.0;
    
    moveLeft.linear.x=0.0;
    moveLeft.linear.y=0.03;
    moveLeft.linear.z=0.0;
    moveLeft.angular.z=0.0;
    
    moveForward.linear.x=0.03;
    moveForward.linear.y=0.0;
    moveForward.linear.z=0.0;
    moveForward.angular.z=0.0;
    
    moveBackward.linear.x=-0.03;
    moveBackward.linear.y=0.0;
    moveBackward.linear.z=0.0;
    moveBackward.angular.z=0.0;
    pub_twist         = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);/* Message queue length is just 1 */
    pub_empty_land    = node.advertise<std_msgs::Empty>("/ardrone/land", -1);
    pub_empty_reset   = node.advertise<std_msgs::Empty>("/ardrone/reset", -1);

    cout<<"Ready to Fly!"<<endl;
    cout<<"Map keyboard:\n"
     <<"t = Take off\n"
     	<<"3 = Pilih Objek segitiga\n"
     	<<"4 = Pilih Objek kotak\n"
     	<<"5 = Pilih Objek penta\n"
        <<"u = Up\n"
        <<"j = Down\n"
        <<"w = Maju\n"
        <<"s = Mundur\n"
        <<"a = Geser Kiri\n"
        <<"d = Geser Kanan\n"
        <<"l = Landing\n"
        <<"space = Stop\n"
        <<"m = Battery\n"
        <<"n = Show Navigation Data\n";
	cv::namedWindow(WINDOW);
	cv::namedWindow(WINDOW2);
	cv::namedWindow(WINDOW3);
    cv::namedWindow(WINDOW4);
    cv::namedWindow(WINDOW5);

	cv::destroyWindow(WINDOW);
	cv::destroyWindow(WINDOW2);
	cv::destroyWindow(WINDOW3);
    cv::destroyWindow(WINDOW4);
    cv::destroyWindow(WINDOW5);
while (ros::ok())
{

    init_keyboard();
    if(kbhit())
        {
        command=readch();
        switch(command)
        {
        case 'u':
            cout<<"Altitude+"<<endl;
            otomatis = true;
	        gerakan = "Naik";
        	gotShape = false;
        	isTrack = false;
        	isMoving = true;
            command = '~';
            break;
        case 'j':
            cout<<"Altitude-"<<endl;
            otomatis = true;
	        gerakan = "Turun";
        	gotShape = false;
        	isTrack = false;
        	isMoving = true;
            command = '~';
            break;    
        case '3':
        	cout<<"Objek segitiga akan dideteksi"<<endl;
			nomerObjek = 3;
			command = '~';
			break;
		case '4':
			cout<<"Objek kotak akan dideteksi"<<endl;
			nomerObjek = 4;
			command = '~';
			break;
		case '5':
			cout<<"Objek penta akan dideteksi";
			nomerObjek = 5;
			command = '~';
			break;    
        case 't':
        	pub_empty_takeoff.publish(msg); //launches the drone
            pub_twist.publish(hoverStop);
            cout<<"Take Off"<<endl;
            command = '~';
            break;
        case 'w':
        	cout<<"Gerakan Maju"<<endl;
            otomatis = true;
	            gerakan = "Maju";
        		gotShape = false;
        		isTrack = false;
        		isMoving = true;
            break;  
        case 's':
        cout<<"Gerakan Mundur"<<endl;
            otomatis = true;
	            gerakan = "Mundur";
        		gotShape = false;
        		isTrack = false;
        		isMoving = true;
            break;    
        case 'a':
        cout<<"Gerakan Geser Kiri"<<endl;
            otomatis = true;
	            gerakan = "Kiri";
        		gotShape = false;
        		isTrack = false;
        		isMoving = true;
            break; 
        case 'd':
        cout<<"Gerakan Geser kanan"<<endl;
            otomatis = true;
	            gerakan = "Kanan";
        		gotShape = false;
        		isTrack = false;
        		isMoving = true;
        	command = '~';
            break;                   
            case 'l':
                pub_twist.publish(hoverStop); //drone is flat
                pub_empty_land.publish(msg); //lands the drone
                command = '~';
                otomatis = false;
                exit(0);
                break;
        case ' ':
            cout<<"Stop"<<endl;
            pub_twist.publish(hoverStop);
            otomatis = false;
            command = '~';
            break;
        case 'n':
            if(showData){
            	showData = false;
            }else{
            	cout<<"Show Data"<<endl;
            	showData = true;
            }
            
            command = '~';
            break;   
        case 'm': 
            if(baterai)baterai=false;
            else if(!baterai)baterai=true;
            cout<<"Battery: "<<battery<<endl;
            command = '~';
            break;
        }

    }
    else if(otomatis){
        switch(command)
        {
    	case '(':
            pub_twist.publish(hoverUp); //drone fly up
            command = '~';
            break;
        case ')':
            pub_twist.publish(hoverDown); //drone fly down
            command = '~';
            break; 
        case '3':
            pub_twist.publish(trackLeft); //drone fly to left
            command = '~';
            break;
        case '4':
            pub_twist.publish(trackRight); //drone fly to right
            command = '~';
            break;            
        case '5':
            pub_twist.publish(hoverStop); //drone stop
            command = '~';
            break;
        case '6':
        	cout<<"Landing"<<endl;
        	pub_twist.publish(hoverStop); //drone is flat
            pub_empty_land.publish(msg); //lands the drone
            command = '~';
            otomatis = false;
            exit(0);
            break;
        case '7':
            pub_twist.publish(trackForward); //drone fly forward
            command = '~';
            break;
        case '8':
            pub_twist.publish(trackBackward); //drone fly backward
            command = '~';
            break;
        case '>':
            pub_twist.publish(moveRight); //drone fly to right
            command = '~';
            break;
        case '<':
            pub_twist.publish(moveLeft); //drone fly to left
            command = '~';
            break;
        case 'v':
            pub_twist.publish(moveBackward); //drone  move backward
            command = '~';
            break;  
        case '^':
            pub_twist.publish(moveForward); //drone move forward
            command = '~';
            break;                    
        }  
    }

    ros::spinOnce();
    loop_rate.sleep();
}

	return 0;
}
