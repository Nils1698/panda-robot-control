#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

void chatterCallback(const apriltag_ros::& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "apriltag_topic_converter");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("tag_detections", 1000, chatterCallback);

  ros::spin();

  return 0;
}


























