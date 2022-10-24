#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <stdio.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/highgui.hpp>
#include <opencv2/core/version.hpp>


static const std::string OPENCV_WINDOW = "Image window";

class ObjectDetector
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber     image_sub_;
  image_transport::Publisher      image_pub_;
  ros::Publisher                  target_xy_pub_;
  bool detected;
public:
  ObjectDetector()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed and pixel target locations
    image_sub_     = it_.subscribe("/image_raw", 1, &ObjectDetector::imageCb, this);
    image_pub_     = it_.advertise("/image_converter/output_video", 1);
    target_xy_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/object_detector/qr_vertecies", 2); // 
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ObjectDetector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  // void display(Mat &im, Mat &bbox)
  // {
  //   int n = bbox.rows;
  //   for(int i = 0 ; i < n ; i++)
  //   {
  //     line(im, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,0,0), 3);
  //   }
  //   imshow("Result", im);
  // }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat inputImage = cv::imread("/home/christian/msc_ws/src/object_detector/src/img5.png",cv::IMREAD_GRAYSCALE);
    // inputImage = imread("/home/christian/msc_ws/src/object_detector/src/img5.png",IMREAD_GRAYSCALE);
    if(inputImage.empty()){
      ROS_ERROR("inputImage was empty, maybe directory or filename was incorrect?\nexiting program");
      return;
    }

    cv::QRCodeDetector qrDecoder;
    cv::Mat bbox, rectifiedImage;
    geometry_msgs::PointStamped pointStamped;
    pointStamped.header.stamp.sec = msg->header.stamp.sec;
    pointStamped.header.stamp.nsec = msg->header.stamp.nsec;
    try
    {
      cv_ptr   = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // cv_ptr->image = inputImage;
      // detected = qrDecoder.detect(cv_ptr->image, bbox);
      std::string data = qrDecoder.detectAndDecode(cv_ptr->image, bbox, rectifiedImage);

      if(0){
      if(data.length()>0)
      {
        ROS_INFO("Decoded Data : %s",data.c_str());
        // cout << "Decoded Data : " << data << endl;
        ROS_INFO("bbox number of dimensions : %d",bbox.dims);
        ROS_INFO("bbox height/rows    : %d",bbox.size().height);
        ROS_INFO("bbox width /columns : %d",bbox.size().width);
        int n = bbox.rows;
        for(int i = 0 ; i < n ; i++)
        {
          ROS_INFO("bbox coordinates: x%d = %f, y%d = %f",n,bbox.at<float>(i,0),n,bbox.at<float>(i,1));
          cv::line(cv_ptr->image, cv::Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), cv::Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), cv::Scalar(255,122,0), 3);
        }      

        // Draw an example circle on the video stream
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
          cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
        // display(inputImage, bbox);
        // rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
        // imshow("Rectified QRCode", rectifiedImage);

        // cv::waitKey(0);
      }
      else
      {
        ROS_INFO("QR Code not detected");
      }
    }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::imshow(OPENCV_WINDOW, inputImage);

    // // std::string data = qrDecoder.detectAndDecode(cv_ptr->image, bbox, rectifiedImage);
    // std::string data = qrDecoder.detectAndDecode(inputImage, bbox, rectifiedImage); // for Testing
    // ROS_INFO("After detectAndDecode()");
    // if(data.length()<0)
    // {
    //   ROS_INFO("%s",data.c_str());
    // //   display(cv_ptr->image, bbox);
    // //   rectifiedImage.convertTo(rectifiedImage,CV_8UC3);

    // //   cv::imshow("Rectified QRCode", rectifiedImage);
    // }
    cv::waitKey(3);
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

//     // Draw an example circle on the video stream
//     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//       cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

//     // Update GUI Window
//     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//     cv::waitKey(3);

//     // Output modified video stream
//     image_pub_.publish(cv_ptr->toImageMsg());
//     // target_xy_pub_.publish(pointStamped);
  }
  // void display(cv::Mat &im, cv::Mat &bbox)
  // {
  //   int n = bbox.rows;
  //   for(int i = 0 ; i < n ; i++)
  //   {
  //     cv::line(im, cv::Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), cv::Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), cv::Scalar(255,0,0), 3);
  //   }
  //   cv::imshow("Result", im);
  // }
  // void qrDetector(const cv_bridge::CvImageConstPtr& const_cv_ptr)
  // {
  //   cv::QRCodeDetector qrDecoder = cv::QRCodeDetector::QRCodeDetector();
  //   cv::Mat bbox;
  //   std::string data = qrDecoder.detect(const_cv_ptr->image, bbox);
  //   ROS_INFO("qrDetector");
  //   // uint32_t cam_info = const_cv_ptr->height;
  //   // ROS_INFO("catch");
    
  //   // char str[16];
  //   // int n = sprintf(str,"%d",cam_info);
  //   // ROS_INFO("I Head [%s]",str);
  //   // ROS_INFO("%s",data.c_str());
  // }


};







int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detector");
  ObjectDetector od;
//  ros::NodeHandle n;
//  ros::Subscriber sub1 = n.subscribe("image_raw",   10, qrDetector);
//  ros::Subscriber sub2 = n.subscribe("camera_info", 10, cameraInfoPrint);
  ros::spin();
  return 0;
}

























































