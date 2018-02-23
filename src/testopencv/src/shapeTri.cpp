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
geometry_msgs::Twist hoverLeft;
geometry_msgs::Twist hoverRight;
geometry_msgs::Twist hoverForward;
geometry_msgs::Twist hoverBackward;
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

bool doneGeserKiri = false;
bool doneGeserKanan = false;
bool doneMajuKiri = false;
bool doneMajuKanan = false;

bool isLanding = false;
bool isTrack = false;
bool isZigZag = false;
bool isLandXD = false;

unsigned int konturAtas, konturBawah;
unsigned int konturObjek=0;
float konturPersen;
int waktu, waktuKiri, waktuKanan, waktuMajuKiri, waktuMajuKanan;
int counterPuterKiri = 0;
int counterPuterKanan = 0;
int zigzagCounter;
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

 int iLowH = 0;
 int iHighH = 88;

 int iLowS = 98; 
 int iHighS = 255;

 int iLowV = 0;
 int iHighV = 255;



/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

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
	// cout<<"Found a "<<label<<" Shape !\n"<<"Koordinat X Shape : "<< CoordShape.x <<"\nKoordinat Y Shape"<< CoordShape.y <<endl;
	cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}


void process(const sensor_msgs::ImageConstPtr& cam_image){
	

	//cv::Mat src = cv::imread("polygon.png");
	cv::Mat src;
	cv::Mat gray;
	cv::Mat bw;
	cv::Mat dst;
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

		if (approx.size() == 3 && nomerObjek == 3)//if (approx.size() == 3 && nomerObjek == 3)
		{
					setLabel(dst, "TRI", contours[i]);    // Triangles
					}
					else if (approx.size() >= 4 && approx.size() <= 6)
					{
						// Number of vertices of polygonal curve
						int vtc = approx.size();

						// Get the cosines of all corners
						std::vector<double> cos;
						for (int j = 2; j < vtc+1; j++)
							cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

						// Sort ascending the cosine values
						std::sort(cos.begin(), cos.end());

						// Get the lowest and the highest cosine
						double mincos = cos.front();
						double maxcos = cos.back();

						// Use the degrees obtained above and the number of vertices
						// to determine the shape of the contour
						if (vtc == 4 && nomerObjek == 4){//if (vtc == 4 && nomerObjek == 4){
							setLabel(dst, "RECT", contours[i]);
						}
						else if (vtc == 5 && nomerObjek == 5){//else if (vtc == 5 && nomerObjek == 5){
							setLabel(dst, "PENTA", contours[i]);
						}else{
							gotShape = false;
							CoordShape.x = 999;
							CoordShape.y = 999;
							tipeObjek = "";
						}
					}
					else
					{
						gotShape = false;
						CoordShape.x = 999;
						CoordShape.y = 999;
						tipeObjek = "";
		}
	}

	int droneHeight = drone_altd/10; // satuan centimeter

	
	if(otomatis && !isLanding){
		if(!gotShape && isZigZag){
			if(droneHeight<135 && !automateHeight){
				command = '1';
			}else if(droneHeight>135 && !automateHeight){
				command = '5';
				automateHeight = true;
			}else if(automateHeight){
				if (gerakan == "Maju"){
					command = '7';
				}else if (gerakan == "Mundur"){
					command = '8';
				}else if (gerakan == "Kiri"){
					command = '4';
				}else if (gerakan == "Kanan"){
					command = '3';
				}
			}	
		}else if(gotShape && !isTrack){
			command = '5';
			isZigZag = false;
			isTrack = true;
		}else if(isTrack){
			//kotak kiri 3 
			if (CoordShape.x>0 && CoordShape.x<110 && CoordShape.y >0 && CoordShape.y <80 ) {
                //geser kanan jika kotak kiri atas
                char Areas[20];
                sprintf(Areas,"Geser Kanan");
                putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
		         command = '3';
		     } else if (CoordShape.x>0 && CoordShape.x<110 && CoordShape.y >80 && CoordShape.y <160 ) {
                 //geser kanan jika kotak kiri atas
                 char Areas[20];
                 sprintf(Areas,"Geser Kanan");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
		         command = '3';
		     } else if (CoordShape.x>0 && CoordShape.x<110 && CoordShape.y >160 && CoordShape.y <240 ) {
                 //geser kanan jika kotak kiri atas
                 char Areas[20];
                 sprintf(Areas,"Geser Kanan");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
		         command = '3';
		     } 
		     //kotak tengah 3
		     else if (CoordShape.x>110 && CoordShape.x<220 && CoordShape.y >0 && CoordShape.y <80 ) {
                 //geser kanan jika kotak kiri atas
                 char Areas[20];
                 sprintf(Areas,"Maju Depan");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
		         command = '7';
		     } else if (CoordShape.x>110 && CoordShape.x<220 && CoordShape.y >80 && CoordShape.y <160 ) {
                //landing jika koordinat titik tengah objek berada ditengah grid
                char Areas[20];
                sprintf(Areas,"Landing");
                putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
				command = '5';
				isLanding = true;
				isTrack = false;
		     } else if (CoordShape.x>110 && CoordShape.x<220 && CoordShape.y >160 && CoordShape.y <240 ) {
                 //mundur ke bawah jika dibawah tengah
                 char Areas[20];
                 sprintf(Areas,"Mundur Bawah");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
		         command = '8';
		     }
		     //kotak kanan 3
		     else if (CoordShape.x>220 && CoordShape.x<330 && CoordShape.y >0 && CoordShape.y <80 ) {
                 //geser kiri jika kotak kiri atas
                 char Areas[20];
                 sprintf(Areas,"Geser Kiri");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '4';
		     } else if (CoordShape.x>220 && CoordShape.x<330 && CoordShape.y >80 && CoordShape.y <160 ) {
                 //geser kiri jika kotak kiri tengah
                 char Areas[20];
                 sprintf(Areas,"Geser Kiri");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '4';
		     } else if (CoordShape.x>220 && CoordShape.x<330 && CoordShape.y >160 && CoordShape.y <240 ) {
                 //geser kiri jika kotak kiri tengah
                 char Areas[20];
                 sprintf(Areas,"Geser Kiri");
                 putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
                 command = '4';
		     }else if (CoordShape.x == 999 && CoordShape.y == 999){
                //Hover jika tidak menemui objek
                char Areas[20];
                sprintf(Areas,"Hover");
                putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
		     	command = '5';
		     }else{
		     	//Hover jika tidak menemui objek
                char Areas[20];
                sprintf(Areas,"Hover");
                putText(dst,Areas,Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),2);
		     	command = '5';
		     }
		}
	}else if(isLanding){
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
		cout<<"Height : "<<drone_altd/10<<"cm \tPosisi VX : "<<vx<<" \t Posisi VY: "<<vy<<" \t Posisi VZ: "<<vz<<" \t Posisi RotX: "<<rotX<<" \t Posisi RotY: "<<rotY<<" \t Posisi RotZ: "<<rotZ<<endl;
	}

	cv::imshow(WINDOW,src);
	cv::imshow(WINDOW2,dst);
	cv::imshow(WINDOW3,bw);
	// cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
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


    //hover message
    hover.linear.x=0.0;
    hover.linear.y=0.0;
    hover.linear.z=0.0;
    hover.angular.z=0.0;

    hoverStop.linear.x=0.0;
    hoverStop.linear.y=0.0;
    hoverStop.linear.z=0.0;
    hoverStop.angular.x=0.0;
	hoverStop.angular.y=0.0;
	hoverStop.angular.z=0.0;

    hoverUp.linear.x=0.0;
	hoverUp.linear.y=0.0;
	hoverUp.linear.z=0.05;
	hoverUp.angular.z=0.0;

	hoverDown.linear.x=0.0;
	hoverDown.linear.y=0.0;
	hoverDown.linear.z=-0.05;
	hoverDown.angular.z=0.0;

	hoverLeft.linear.x=0.0;
    hoverLeft.linear.y=0.05;
    hoverLeft.linear.z=0.0;
	hoverLeft.angular.z=0.0;

	hoverRight.linear.x=0.0;
    hoverRight.linear.y=-0.05;
    hoverRight.linear.z=0.0;
	hoverRight.angular.z=0.0;

	hoverForward.linear.x=0.05;
	hoverForward.linear.y=0.0;
	hoverForward.linear.z=0.0;
	hoverForward.angular.z=0.0;

	hoverBackward.linear.x=-0.05;
	hoverBackward.linear.y=-0.0;
	hoverBackward.linear.z=0.0;
	hoverBackward.angular.z=0.0;

	
	zigzagCounter = 0;
	

    pub_twist         = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);/* Message queue length is just 1 */
    pub_empty_land    = node.advertise<std_msgs::Empty>("/ardrone/land", -1);
    pub_empty_reset   = node.advertise<std_msgs::Empty>("/ardrone/reset", -1);

    cout<<"Ready to Fly!"<<endl;
    cout<<"Map keyboard:\n"
     <<"t = Take off\n"
     	<<"a = Pilih Objek segitiga\n"
     	<<"b = Pilih Objek kotak\n"
     	<<"c = Pilih Objek penta\n"
        <<"1 = Maju\n"
        <<"2 = Mundur\n"
        <<"3 = Geser Kiri\n"
        <<"4 = Geser Kanan\n"
        <<"l = Landing\n"
        <<"space = Stop\n"
        <<"m = Battery\n"
        <<"n = Show Navigation Data\n";
	cv::namedWindow(WINDOW);
	cv::namedWindow(WINDOW2);
	cv::namedWindow(WINDOW3);
	// cv::namedWindow("Thresholded Image");

	cv::destroyWindow(WINDOW);
	cv::destroyWindow(WINDOW2);
	cv::destroyWindow(WINDOW3);
	// cv::destroyWindow("Thresholded Image");
while (ros::ok())
{
	
	
    init_keyboard();
    if(kbhit())
        {
        command=readch();
        switch(command)
        {
        case 'a':
			nomerObjek = 3;
			cout<<"Objek segitiga akan dideteksi";
			command = '~';
			break;
		case 'b':
			nomerObjek = 4;
			cout<<"Objek kotak akan dideteksi";
			command = '~';
			break;
		case 'c':
			nomerObjek = 5;
			cout<<"Objek penta akan dideteksi";
			command = '~';
			break;    
        case 't':
        	pub_empty_takeoff.publish(msg); //launches the drone
            pub_twist.publish(hover); //drone fly up
            cout<<"Take Off"<<endl;
            command = '~';
            break;
        case '1':
            otomatis = !otomatis;
        	if(otomatis){
        		pub_empty_takeoff.publish(msg); //launches the drone
	            pub_twist.publish(hover); //drone fly up
	            gerakan = "Maju";
        		automateHeight = false;
        		gotShape = false;
        		isTrack = false;
        	}else{
        		isZigZag = false;
        		pub_twist.publish(hoverStop);
        	}
        	command = '~';
            break;  
        case '2':
            otomatis = !otomatis;
        	if(otomatis){
        		pub_empty_takeoff.publish(msg); //launches the drone
	            pub_twist.publish(hover); //drone fly up
	            gerakan = "Mundur";
        		automateHeight = false;
        		gotShape = false;
        		isTrack = false;
        	}else{
        		isZigZag = false;
        		pub_twist.publish(hoverStop);
        	}
        	command = '~';
            break;    
        case '3':
            otomatis = !otomatis;
        	if(otomatis){
        		pub_empty_takeoff.publish(msg); //launches the drone
	            pub_twist.publish(hover); //drone fly up
	            gerakan = "Kiri";
        		automateHeight = false;
        		gotShape = false;
        		isTrack = false;
        	}else{
        		isZigZag = false;
        		pub_twist.publish(hoverStop);
        	}
        	command = '~';
            break; 
        case '4':
            otomatis = !otomatis;
        	if(otomatis){
        		pub_empty_takeoff.publish(msg); //launches the drone
	            pub_twist.publish(hover); //drone fly up
	            gerakan = "Kanan";
        		automateHeight = false;
        		gotShape = false;
        		isTrack = false;
        	}else{
        		isZigZag = false;
        		pub_twist.publish(hoverStop);
        	}
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
    	case '1':
            pub_twist.publish(hoverUp); //drone fly up
            // cout<<"Fly UP"<<endl;
            command = '~';
            break;
        case '2':
            pub_twist.publish(hoverDown); //drone fly down
            // cout<<"Fly Down"<<endl;
            command = '~';
            break; 
        case '3':
            pub_twist.publish(hoverLeft); //drone fly to left
            // cout<<"Fly Left"<<endl;
            command = '~';
            break;
        case '4':
            pub_twist.publish(hoverRight); //drone fly to right
            // cout<<"Fly Right"<<endl;
            command = '~';
            break;
        case '5':
        	// cout<<"Stop"<<endl;
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
            pub_twist.publish(hoverForward); //drone fly forward
            command = '~';
            break;
        case '8':
            pub_twist.publish(hoverBackward); //drone fly backward
            command = '~';
            break;              
        }  
    }

    ros::spinOnce();
    loop_rate.sleep();
}//ros::ok

	return 0;
}
