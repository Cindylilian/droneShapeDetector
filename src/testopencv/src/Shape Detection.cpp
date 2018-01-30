#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

geometry_msgs::Twist twist_msg;
geometry_msgs::Twist hover;
std_msgs::Empty msg;

ardrone_autonomy::Navdata msg_in;
int drone_altd;
float battery;

bool otomatis = false;
bool baterai = false;
unsigned int konturAtas, konturBawah;
unsigned int konturObjek=0;
float konturPersen;
double waktu1, waktu2;
char command =' ';
static const char WINDOW[]="SRC Image";
static const char WINDOW2[]="DST Image";
static const char WINDOW3[]="Black White Image";


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

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
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
	cv_bridge::CvImagePtr bridge;
	bridge = cv_bridge::toCvCopy(cam_image, sensor_msgs::image_encodings::BGR8);
	gray = bridge->image;
	resize(gray,src,Size(1080,720));
	

	cv::cvtColor(src, gray, CV_BGR2GRAY);

	// Use Canny instead of threshold to catch squares with gradient shading
	blur( gray, bw, Size(3,3) );
	cv::Canny(gray, bw, 80, 240, 3);
	cv::imshow("bw", bw);
	//cv::bitwise_not(bw, bw);

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

		if (approx.size() == 3)
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
						if (vtc == 4 )
							setLabel(dst, "RECT", contours[i]);
						else if (vtc == 5 )
							setLabel(dst, "PENTA", contours[i]);
						else if (vtc == 6 )
							setLabel(dst, "HEXA", contours[i]);
					}
					else
					{
						// Detect and label circles
						double area = cv::contourArea(contours[i]);
						cv::Rect r = cv::boundingRect(contours[i]);
						int radius = r.width / 2;

						if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
							std::abs(1 - (area / (CV_PI * (radius*radius)))) <= 0.2)
							setLabel(dst, "CIR", contours[i]);
		}
	}

	Point CoordA;
    Point CoordB;
    CoordA.x = 0;
    CoordA.y = 0;
    CoordB.x = 0;
    CoordB.y = 0;

	cv::imshow(WINDOW,src);
	cv::imshow(WINDOW2,dst);
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
    battery = msg_in.batteryPercent;
}




int main(int argc, char **argv){
	ros::init(argc, argv,"shapeDetection");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    ros::Subscriber sub_navdata;
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("/ardrone/image_raw",1,process);

	cv::namedWindow(WINDOW);
	cv::namedWindow(WINDOW2);
	cv::namedWindow(WINDOW3);

	cv::destroyWindow(WINDOW);
	cv::destroyWindow(WINDOW2);
	cv::destroyWindow(WINDOW3);

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


    pub_twist         = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);/* Message queue length is just 1 */
    pub_empty_land    = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
    pub_empty_reset   = node.advertise<std_msgs::Empty>("/ardrone/reset", 1);

    cout<<"Ready to Fly!"<<endl;
    cout<<"Map keyboard:\n"
        <<"t = Take Off\n"
        <<"l = Landing\n"
        <<"h = Hovering\n"
        <<"m = Battery\n";
    cout<<"Ketinggian(cm)"<<"\t"<<"LuasKontur"<<"\t"<<"WaktuRespon"<<endl;

while (ros::ok())
{
    init_keyboard();
    if(kbhit())
        {
        command=readch();
        switch(command)
        {
        case 't':
            pub_empty_takeoff.publish(msg); //launches the drone
            pub_twist.publish(hover); //drone is flat
            cout<<"Taking Off"<<endl;
            command=' ';
            break;

        case 'l':
            pub_twist.publish(hover); //drone is flat
            pub_empty_land.publish(msg); //lands the drone
            cout<<"Landing"<<endl;
            exit(0);
            command = ' ';
            break;

        case 'h':
            cout<<"Hovering"<<endl;
            pub_twist.publish(hover);
            command = ' ';
            break;
            
        case 'm':
            if(baterai)baterai=false;
            else if(!baterai)baterai=true;
            cout<<"Battery: "<<battery<<endl;
            command = ' ';
            break;
        }
    }

    ros::spinOnce();
    loop_rate.sleep();
}//ros::ok
	return 0;
}
