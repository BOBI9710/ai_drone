#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ai_drone/error_data.h>

#include <iostream>
#include <iomanip>
#include <math.h>

using namespace cv;
using namespace std;

//set variable for drone pose
int x,y,x_s,y_s;
int iter =0;
int mode =0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ros::NodeHandle nh;
  ros::Publisher error_data_pub = nh.advertise<ai_drone::error_data>("error_data_msg",30);
  
  ai_drone::error_data error;
  
  try
  {
    Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
   
    //gray
    Mat gray_img;
    cvtColor(img, gray_img, CV_BGR2GRAY);
    
    //blur
    Mat blur_img;
    GaussianBlur(gray_img, blur_img, Size(7,7), 2, 2);

    //circle detection
    Mat img_houghC;
    img.copyTo(img_houghC);
	
    vector<Vec3f> circles;
    HoughCircles(blur_img, circles, CV_HOUGH_GRADIENT, 1, 30, 130, 50, 0, 0);
	
    for (size_t i = 0; i < circles.size(); i++)
	{
		
                Vec3d c = circles[i];
		Point center(c[0], c[1]);
		int radius = c[2];

		circle(img_houghC, center, radius, Scalar (255, 0, 0), 2);
		circle(img_houghC, center, 2, Scalar (255, 0, 0), 3);

        
        // y error data
   	 if (fabs(c[0] - 160.0)<8){
    		y_s=0; // error under 8 
  	  }
   	 else { 
    		y_s=-1; // error over 8 
    	  }
        
          y = - c[0] + 160 ; 
       
         // x error data
   	 if (fabs(c[1] - 120.0)<8){
    		x_s=0; // error under 8
    	  }
    	 else { 
    		x_s=-1; // error over 8 
          }
         
          x = - c[1] + 120;

         //mode setting
         if(x_s ==0 && y_s==0){
         iter = iter +1;
         }
         else{
         iter =0;
         }
     
         if(iter >= 100){
          mode = -1;
         }
         else{
          mode = 1;
         }

        char status_x[30];
        sprintf(status_x, "x = %d , x_s = %d",x,x_s);
        char status_y[30];
        sprintf(status_y, "y = %d , y_s = %d",y,y_s);
        char status_mode[30];
        sprintf(status_mode, "mode = %d",mode);

        //show variable status at cam
        putText(img_houghC, status_mode , Point(20,190) ,1,1, Scalar(0,0,0), 1,8);
        putText(img_houghC, status_x , Point(20,210) ,1,1, Scalar(0,0,0), 1,8);
        putText(img_houghC, status_y , Point(20,230) ,1,1, Scalar(0,0,0), 1,8);
	}

    //show camera center 
    circle(img_houghC, Point(160, 120),10, Scalar (255, 0, 255),2 );
    
   // publish message to flight_control node
   error.mode_data = mode;
   error.x_status = x_s;
   error.y_status = y_s;
   error.x_data = x;
   error.y_data = y;
  
   error_data_pub.publish(error);
   
    imshow("view", img_houghC);
    cv::waitKey(20);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_process");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  ros::Publisher error_data_pub = nh.advertise<ai_drone::error_data>("error_data_msg",30);
  
  ai_drone::error_data error;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/iris/usb_cam/image_raw", 1, imageCallback);


  ros::spin();
  cv::destroyWindow("view");
}
