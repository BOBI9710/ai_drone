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

int iter =0;
int mode =0;
int vert1 = 0;
int vert2 = 0;
int a = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ros::NodeHandle nh;
  ros::Publisher front_data_pub = nh.advertise<ai_drone::error_data>("front_data_msg",30);
  
  ai_drone::error_data frontd;
  
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

    Mat img_houghC;
    img.copyTo(img_houghC);

    //show roi area
    rectangle(img_houghC, Rect(30, 0, 120, 110), Scalar(0, 0, 0), 1, 8, 0);
    rectangle(img_houghC, Rect(170, 0, 120, 110), Scalar(0, 0, 0), 1, 8, 0);

    //roi
    Mat img_roi_1, img_roi_2, mask_img;
    canny_img.copyTo(mask_img);
   
    img_roi_1 = mask_img(Rect(30, 0, 120, 110));
    img_roi_2 = mask_img(Rect(170, 0, 120, 110));

    //line detection
    vector<Vec2f> lines_1;
    vector<Vec2f> lines_2;
    HoughLines(img_roi_1, lines_1, 1, CV_PI / 180, 100);
    HoughLines(img_roi_2, lines_2, 1, CV_PI / 180, 100);

     // if detect line -> avoidance -> mode4
        //lines 1

        mode = 0;
        vert1 = 0;
        vert2 = 0;

     // mode
      if(a == 1){
        mode = 4;        
        iter = iter + 1;

        if(iter >= 150){
          mode = 0;
          a = 0;     
           }

       }else{

        iter = 0;
        for(size_t k =0 ; k < lines_1.size(); k++){
             if (lines_1[k][1] >= 0 && lines_1[k][1] <= 0.174533 || lines_1[k][1] >= 2.96706 && lines_1[k][1] <= 3.14159){
                 vert1 = vert1 + 1;
                }
             }
        if(lines_1.size() > 0){
           rectangle(img_houghC, Rect(30, 0, 120, 110), Scalar(0, 0, 255), 2.5, 8, 0);
          }
       

         // lines 2

        for(size_t k =0 ; k < lines_2.size(); k++){
             if (lines_2[k][1] >= 0 && lines_2[k][1] <= 0.174533 || lines_2[k][1] >= 2.96706 && lines_2[k][1] <= 3.14159){
                 vert2 = vert2 + 1;
                }
             }
        if(lines_2.size() > 0){
           rectangle(img_houghC, Rect(170, 0, 120, 110), Scalar(0, 0, 255), 2.5, 8, 0);
          }      

        if( vert1 > 0 && vert2 > 0 ){
           a = 1;
          }
        else{
           a = 0;
           vert1 = 0;
           vert2 = 0;
          }
         }

      if(mode == 4){
       rectangle(img_houghC, Rect(30, 0, 120, 110), Scalar(255, 0, 0), 2.5, 8, 0);
       rectangle(img_houghC, Rect(170, 0, 120, 110), Scalar(255, 0, 0), 2.5, 8, 0);
   
       }

      //show variable status at cam
      char status_mode[30];
      sprintf(status_mode, "mode = %d",mode);
      char status_iter[30];
      sprintf(status_iter, "iter = %d",iter);

      putText(img_houghC, status_mode , Point(20,190) ,1,1, Scalar(0,0,0), 1,8);
      putText(img_houghC, status_iter , Point(20,210) ,1,1, Scalar(0,0,0), 1,8);
    
    // publish message to flight_control node
    frontd.mode_data = mode;

    front_data_pub.publish(frontd);
   
    imshow("view_front",img_houghC);
    cv::waitKey(20);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "front_image");
  ros::NodeHandle nh;
  cv::namedWindow("view_front");

  ros::Publisher front_data_pub = nh.advertise<ai_drone::error_data>("front_data_msg",30);
  
  ai_drone::error_data frontd;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/iris/usb_cam_2/image_raw_2", 1, imageCallback);


  ros::spin();
  cv::destroyWindow("view_front");
}
