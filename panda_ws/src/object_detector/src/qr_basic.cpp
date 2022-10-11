#include <ros/ros.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void display(Mat &im, Mat &bbox)
{
  int n = bbox.rows;
  for(int i = 0 ; i < n ; i++)
  {
    cv::line(im, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), cv::Scalar(0,0,127), 3);
    // ROS_INFO("Someting from bbox?: %s",std::string((char *)bbox.at<float>(i,0)).c_str());
  }
  imshow("Result", im);
}

int main(int argc, char* argv[])
{
  // Read image
  Mat inputImage;
  // if(argc>1)
  //   inputImage = imread(argv[1]);
  // else
  // {
  while(inputImage.empty()){
    ROS_INFO("inputImage is empty trying to load again");
    inputImage = imread("/home/christian/msc_ws/src/object_detector/src/img5.png",cv::IMREAD_COLOR);
    // imshow("inputImage",inputImage);
    // waitKey(0);
  }
  // }
  QRCodeDetector qrDecoder;

  Mat bbox, rectifiedImage;

  std::string data = qrDecoder.detectAndDecode(inputImage, bbox, rectifiedImage);
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
      // line(inputImage, Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), Scalar(255,122,0), 20);

      // ROS_INFO("Someting from bbox?: %s",std::string((char *)bbox.at<float>(i,0)).c_str());
    }
    display(inputImage, bbox);
    // rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
    // imshow("Rectified QRCode", rectifiedImage);
    // cv::imshow("inputImage",inputImage);
    // cv::imshow("inputImage",inputImage);
    waitKey(0);
  }
  else
  {
    ROS_INFO("QR Code not detected");
  }
    // ROS_INFO("%s",data.c_str());
    // cout << "QR Code not detected" << endl;
}
