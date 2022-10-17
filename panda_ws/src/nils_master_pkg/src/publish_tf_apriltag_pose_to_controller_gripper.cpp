#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class PoseDrawer
{
private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  ros::NodeHandle n_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> point_sub_;
  tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped> tf2_filter_;
  ros::Publisher pub_;
public:
  PoseDrawer() :
    tf2_(buffer_),  target_frame_("panda_link0"),
    tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
  {
    point_sub_.subscribe(n_, "target_detection_camera", 10);
    tf2_filter_.registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
    pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>
      ("/desktop/target_pose_cov_stmp_panda_link0",10);
  }

  //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
  void msgCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& point_ptr) 
  {
    geometry_msgs::PoseWithCovarianceStamped point_out;
    try 
    {
      //Transform from Camera to Base Link 0
      buffer_.transform(*point_ptr, point_out, target_frame_);
      //Cheating so that there is space for the gripper
      point_out.pose.pose.position.z     = point_out.pose.pose.position.z + 0.195;

      if (std::sqrt(std::pow(point_out.pose.pose.position.x,2) + std::pow(point_out.pose.pose.position.y,2)) > 0.8){
          point_out.pose.pose.position.x     = 0.6;
          point_out.pose.pose.position.y     = 0.0;
          point_out.pose.pose.position.z     = 0.6;
          point_out.pose.pose.orientation.x  = 0;
          point_out.pose.pose.orientation.y  = 0;
          point_out.pose.pose.orientation.z  = 1;
          point_out.pose.pose.orientation.w  = 0;
      }

      ROS_INFO("x=%f", point_out.pose.pose.position.x);
      ROS_INFO("y=%f", point_out.pose.pose.position.y);
      ROS_INFO("z=%f", point_out.pose.pose.position.z);

      pub_.publish(point_out);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
  }
};

int main(int argc, char ** argv)
{
  ROS_INFO("publish_tf_apriltag_pose_to_controller_gripper.cpp running...");
  
  ros::init(argc, argv, "publish_tf_apriltag_pose_to_controller"); //Init ROS
  PoseDrawer pd; //Construct class
  ros::spin(); // Run until interupted 
  return 0;
};