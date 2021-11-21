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
int d;
int iter =0;
int mode =0;
float theta_e=0.0;
int vert, paral = 0;
float rho_avg, theta_avg = 0;
float theta_arr=0;

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

    //Canny
    Mat canny_img;
    Canny(blur_img, canny_img, 50, 125);

    //circle detection
    Mat img_houghC;
    img.copyTo(img_houghC);
	
    vector<Vec3f> circles;
    HoughCircles(blur_img, circles, CV_HOUGH_GRADIENT, 1, 30, 130, 50, 0, 100);

    //line detection
    vector<Vec2f> lines;
    HoughLines(canny_img, lines, 1, CV_PI / 180, 100);

    // circle center detection and move drone to center -> mode 1
    // circle center dectection and land -> mode5
    for (size_t i = 0; i < circles.size(); i++) 
	{  
          // show circle
            Vec3d c = circles[i];
	    Point center(c[0], c[1]);
	    int radius = c[2];

	    circle(img_houghC, center, radius, Scalar (255, 0, 0), 2);
	    circle(img_houghC, center, 2, Scalar (255, 0, 0), 3);
          
        // detect one circle -> spwan to start

        if(circles.size() == 1){
       
         // y error data
   	   if (fabs(c[0] - 160.0)<8){
    		y_s=0; // error under 8 
  	    }
   	   else { 
    	  	y_s=1; // error over 8 
    	    }
        
            y = - c[0] + 160 ; // y error save
       
         // x error data
   	   if (fabs(c[1] - 120.0)<8){
    	  	x_s=0; // error under 8
    	    }
    	   else { 
       		x_s=1; // error over 8 
            }
         
            x = - c[1] + 120; // x error save

         //mode setting
           if(x_s ==0 && y_s==0){ // x_s =0 and y_s =0 for certain time -> alt down to 0.9m
           iter = iter +1;
           }
           else{
           iter =0;
           }
       
           if(iter >= 50){
            mode = -1;
           }else{
            mode = 1;
           }
        
        }else if(circles.size() >= 2){
          // mode 5 landing ; break to ignore situation for one circle detection
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

      // line detection and track line -> mode2 
      // if dectect parallel lines -> mode3 -> move left for few and yaw + 90deg
 
      if(circles.size() == 0){
        for (size_t j = 0; j < lines.size(); j++ ){
          // show line         
            float rho = lines[j][0], theta = lines[j][1];
	    Point pt1, pt2;
	    double a = cos(theta), b = sin(theta);
	    double x0 = a * rho, y0 = b * rho;
	    pt1.x = cvRound(x0 + 1000 * (-b));
	    pt1.y = cvRound(y0 + 1000 * (a));
	    pt2.x = cvRound(x0 - 1000 * (-b));
	    pt2.y = cvRound(y0 - 1000 * (a));

	    line(img_houghC, pt1, pt2, Scalar(255,128,0), 2, 8);

            if(circles.size() == 0){

            // theta sorting 

           for(size_t k =0 ; k < lines.size(); k++){
             if (lines[k][1] >= 1.39626 && lines[k][1] <= 1.74533){
                 paral = paral + 1;
                }
             }

         // paral == 0 -> mode2

          if(paral == 0){
           if(lines.size() == 1){
 
            // theta change to -pi to pi

             if (lines[0][1] >= 0 && lines[0][1] <= 3.14159/9){
                 theta_arr = lines[0][1];    
                }
             else if(lines[0][1] > 8 * 3.14159 && lines[0][1]<= 3.14159){
                 theta_arr = lines[0][1]-3.14159;
                }else{
                 theta_arr = 0;
                }
              
              theta_avg = theta_arr;
              rho_avg = lines[0][0];
 
              theta_e = theta_avg; //theta error save
              d = floor(160 + tan(theta_avg)*120 - tan(theta_avg)*rho_avg*sin(theta_avg) - rho_avg*cos(theta_avg) / sqrt(1+pow(tan(theta_avg),2))); // d error save

              rho_avg = 0;
              theta_avg = 0;
             }
           else if(lines.size() > 1){

               if (lines[0][1] >= 0 && lines[0][1] <= 3.14159/9){
                 theta_avg = lines[0][1];    
                }
               else if(lines[0][1] > 8 * 3.14159/9 && lines[0][1]<= 3.14159){
                 theta_avg = lines[0][1]-3.14159;
                }else{
                 theta_avg = 0;
                }

              rho_avg = abs(lines[0][0]);
           
              for(size_t k = 1 ; k < lines.size(); k++){

                 if (lines[k][1] >= 0 && lines[k][1] <= 3.14159/9){
                 theta_arr = lines[k][1];    
                }
                 else if(lines[k][1] > 8 * 3.14159/9 && lines[k][1]<= 3.14159){
                 theta_arr = lines[k][1]-3.14159;
                }else{
                 theta_arr = theta_arr;
                }
                
                theta_avg = (theta_avg + theta_arr)/2;
                rho_avg = (rho_avg + abs(lines[k][0]))/2;
                }

              theta_e = theta_avg; //theta error save
              d = floor(160 + tan(theta_avg)*120 - tan(theta_avg)*rho_avg*sin(theta_avg) - rho_avg*cos(theta_avg) / sqrt(1+pow(tan(theta_avg),2))); // d error save

              rho_avg = 0;
              theta_avg = 0;
           }

             x = (d) * sin(theta_e);
             y = (d) * cos(theta_e);
             x_s = 1;
             y_s = 1;
             mode = 2; 
             vert = 0;
             paral = 0;
             iter = 0;
           }
         // paral =! 0 -> if detect parallel lines -> stop -> wait for mode -3
         else if(paral > 0){ 

           iter = iter +1;
       
           if(iter >= 40){
            mode = -3;
           }else{
            mode = 3;
           }

           vert = 0;
           paral = 0;
         }
    
            char status_theta[30];
            sprintf(status_theta, "theta_e = %.3f",theta_e*180/3.14159);
            char status_d[30];
            sprintf(status_d, "d error = %d",d);
            char status_mode[30];
            sprintf(status_mode, "mode = %d",mode);

            //show variable status at cam
            putText(img_houghC, status_mode , Point(20,190) ,1,1, Scalar(0,0,0), 1,8);
            putText(img_houghC, status_theta , Point(20,210) ,1,1, Scalar(0,0,0), 1,8);
            putText(img_houghC, status_d , Point(20,230) ,1,1, Scalar(0,0,0), 1,8);
  
            } 
         }
       }

     // nothing detected -> start -> line tracking
     if(circles.size() == 0 && lines.size() == 0){ // alt down enough which can't detect the circle -> still move front
        mode = -2;
        
        char status_mode[30];
        sprintf(status_mode, "mode = %d",mode);
        putText(img_houghC, status_mode , Point(20,190) ,1,1, Scalar(0,0,0), 1,8);
         }

    //show camera center 
    circle(img_houghC, Point(160, 120),10, Scalar (255, 0, 255),2 );
    
   // publish message to flight_control node
   error.mode_data = mode;
   error.x_status = x_s;
   error.y_status = y_s;
   error.x_data = x;
   error.y_data = y;
   error.theta_data = theta_e;
  
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
