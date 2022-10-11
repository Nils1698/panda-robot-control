#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <stdio.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// sensor_msgs::ImageConstPtr img_msg;
// sensor_msgs::CameraInfo cam_info;

#include <opencv2/core/version.hpp>

static const std::string OPENCV_WINDOW = "Image window";


// class ObjectDetector
// {
//   ros::NodeHandle nh_;
//   image_transport::ImageTransport it_;
//   image_transport::Subscriber     image_sub_;
//   image_transport::Publisher      image_pub_;
//   ros::Publisher                  target_xy_pub_;

// public:
//   ObjectDetector():it_(nh_) // Constructor
//   {
//     // Subscribe to input video (that is in OpenCV format) and publish coordinate detections
//     image_sub_ = it_.subscribe("/image_converter/output_video",1, &ObjectDetector::qrDetector, this);

//     cv::namedWindow(OPENCV_WINDOW);


//   }
  
//   ~ObjectDetector(){} // Destructor

//   void qrDetector(const cv_bridge::CvImageConstPtr& const_cv_ptr)
//   {
//     cv::Mat bbox;
//     std::string data = cv::detect.detectAndDecode(const_cv_ptr->image, bbox);
//     ROS_INFO("qrDetector");
//     uint32_t cam_info = const_cv_ptr.height();
//     ROS_INFO("catch");

//     char str[16];
//     int n = sprintf(str,"%d",cam_info);
//     ROS_INFO("I Head [%s]",str);
//     ROS_INFO("%s",data.c_str());
//   }


// };

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


void qrDetector(const sensor_msgs::ImageConstPtr& image)
{
  ROS_INFO("qrDetector");
  uint32_t cam_info = CV_VERSION_MAJOR;//image->height;
  ROS_INFO("catch");
  char str[16];
  int n = sprintf(str,"%d",cam_info);
  ROS_INFO("I Head [%s]",CV_VERSION);
}



void cameraInfoPrint(const sensor_msgs::ImagePtr& image){
  ROS_INFO("cameraInfoPrint");
  
  //  img_msg  = image;
  //  ROS_INFO("try");
  
  
  // try{
  //   image_msg = image
  // }
//  uint32_t img_height = *image.height;
//  std::string image_topic = image.getTopic();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  // ObjectDetector od;
 ros::NodeHandle n;
 ros::Subscriber sub1 = n.subscribe("image_raw",   10, qrDetector);
//  ros::Subscriber sub2 = n.subscribe("camera_info", 10, cameraInfoPrint);
  ros::spin();
  return 0;
}
