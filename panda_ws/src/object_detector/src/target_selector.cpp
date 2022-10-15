
// #include <object_detector/target_selector.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <vector>

// #include "tf2_ros/message_filter.h" // Jeg er måske interesseret i at bruge den har pakke til filterering af /tf topicket

// namespace target_selector {

class TargetSelector
{
public:
  // int target_tag;
  // private:
  ros::Subscriber sub;              // Subscriber to apriltag /tag_detections
  ros::Publisher pub_pose_cov_stmp; // publisher of target object pose covariance stamped
  ros::Publisher pub_pose;          // publisher of target object pose
  // apriltag_ros::AprilTagDetection target;
  geometry_msgs::Pose target_pose;
  geometry_msgs::PoseWithCovarianceStamped target_pose_measurement;

public:
  TargetSelector(ros::NodeHandle *n)
  { // Constructor

    pub_pose_cov_stmp = n->advertise<geometry_msgs::PoseWithCovarianceStamped>("/target_pose_meas_cov_stamped", 10);
    pub_pose = n->advertise<geometry_msgs::Pose>("/target_pose_meas", 10);

    sub = n->subscribe("/tag_detections", 1000,
                       &TargetSelector::target_select_callback, this);
  }

  void target_select_callback(const apriltag_ros::AprilTagDetectionArray &msg)
  // void chatterCallback(const std_msgs::String::ConstPtr& msg)
  {
    target_pose_measurement.header = msg.header; // Copy header from AprilTagDetectionArray
    // target_pose.header              = msg.header;
    std::for_each(std::begin(msg.detections), std::end(msg.detections),
                  [this](apriltag_ros::AprilTagDetection const &detection) { // this has to be in the [this] to be able to call the class members from inside the lambda function.
                    ROS_INFO("I heard: [%d]", detection.id[0]);
                    if (detection.id[0] == 1)
                    {
                      ROS_INFO("Target %d Selected", detection.id[0]);
                      // target_pose_measurement.pose = detection.pose.pose; //https://answers.ros.org/question/191012/problem-with-subscribe-and-callback-function/
                      target_pose_measurement = detection.pose; // PoseWithCovarianceStamped is set (probably, try to print a measurement) //https://answers.ros.org/question/191012/problem-with-subscribe-and-callback-function/
                      target_pose = detection.pose.pose.pose;
                      // target_pose_measurement.covariance = ;TODO: just assume zero for now and take care of it in the kalman filter initialization. Answer: This has been handled in the kalman filter
                      // ROS_INFO("target_pose_measurement.pose.x = %f",target_pose_measurement.pose.pose.position.x);
                      pub_pose_cov_stmp.publish(target_pose_measurement);
                      pub_pose.publish(target_pose);
                      // target = msg.detections[0];
                      // pub.publish(msg.detections[0].pose);
                    }
                    // std::cout << detection << "\n";
                  });
    // geometry_msgs::PoseWithCovarianceStamped TagDetector::makeTagPose(
    //     const Eigen::Matrix4d& transform,
    //     const Eigen::Quaternion<double> rot_quaternion,
    //     const std_msgs::Header& header)
    // {
    //   geometry_msgs::PoseWithCovarianceStamped pose;
    //   pose.header = header;
    //   //===== Position and orientation
    //   pose.pose.pose.position.x    = transform(0, 3);
    //   pose.pose.pose.position.y    = transform(1, 3);
    //   pose.pose.pose.position.z    = transform(2, 3);
    //   pose.pose.pose.orientation.x = rot_quaternion.x();
    //   pose.pose.pose.orientation.y = rot_quaternion.y();
    //   pose.pose.pose.orientation.z = rot_quaternion.z();
    //   pose.pose.pose.orientation.w = rot_quaternion.w();
    //   return pose;
    // }

    // std::vector<apriltag_ros::AprilTagDetection> = msg.detections[0];
    // ROS_INFO("I heard: [%s]", std::to_string(msg.detections.id).c_str());
  }
};
// }

int main(int argc, char **argv)
{
  ROS_INFO("target_selector.cpp running...");

  ros::init(argc, argv, "my_tf2_listener");
  ros::NodeHandle n;
  // int target_tag = 6;
  // ROS_INFO("Here");
  // if (argc == 1)
  //   int target_tag = std::stoi(argv[1]);
  // // Faster comparison can be found at https://stackoverflow.com/questions/17095324/fastest-way-to-determine-if-an-integer-is-between-two-integers-inclusive-with
  // if (argc > 0 && 0 <= target_tag && target_tag <= 19){
  //   ROS_INFO("Custom target_tag %d is used.",target_tag);
  TargetSelector ts = TargetSelector(&n);
  // } else {
  //   ROS_INFO("Invalid target tag %s was received",argv[1]);
  //   ROS_INFO("Default target_tag 6 is used.");
  //   TargetSelector ts = TargetSelector(&n,6);
  // }

  ros::spin();
  return 0;
};
