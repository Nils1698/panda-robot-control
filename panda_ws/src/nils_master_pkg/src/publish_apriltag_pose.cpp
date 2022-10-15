
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

class TargetSelector
{
public:
  ros::Subscriber sub;              // Subscriber to apriltag /tag_detections
  ros::Publisher pub_pose_cov_stmp; // publisher of target object pose covariance stamped
  ros::Publisher pub_pose;          // publisher of target object pose
  geometry_msgs::Pose target_pose;
  geometry_msgs::PoseWithCovarianceStamped target_pose_measurement;

public:
  TargetSelector(ros::NodeHandle *n)
  { // Constructor

    // Target_pose_meas_cov_stamped includes time stamps inside of the topic
    pub_pose_cov_stmp = n->advertise<geometry_msgs::PoseWithCovarianceStamped>("/target_pose_meas_cov_stamped", 10);
    pub_pose = n->advertise<geometry_msgs::Pose>("/target_pose_meas", 10);

    sub = n->subscribe("/tag_detections", 1000,
                       &TargetSelector::target_select_callback, this);
  }

  void target_select_callback(const apriltag_ros::AprilTagDetectionArray &msg)
  {
    target_pose_measurement.header = msg.header;
    std::for_each(std::begin(msg.detections), std::end(msg.detections),
                  [this](apriltag_ros::AprilTagDetection const &detection)
                  {
                    // If the AprilTag on the screen is April Tag 1 then publish the AprilTag coordinates
                    if (detection.id[0] == 1)
                    {
                      pub_pose_cov_stmp.publish(detection.pose);
                    }
                  });
  }
};

int main(int argc, char **argv)
{
  ROS_INFO("publish_apriltag_pose.cpp running...");

  ros::init(argc, argv, "publish_apriltag_pose");
  ros::NodeHandle n;
  TargetSelector ts = TargetSelector(&n);

  ros::spin();
  return 0;
};
