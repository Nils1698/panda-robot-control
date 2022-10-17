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
      ("/target_pose_meas_cov_stamped_panda_link0",10);
  }

  //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
  void msgCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& point_ptr) 
  {
    geometry_msgs::PoseWithCovarianceStamped point_out;
    try 
    {
        buffer_.transform(*point_ptr, point_out, target_frame_);

        //Makes sure that april tag is seen higher so that gripper collision is avoided
        point_out.pose.pose.position.z     = point_out.pose.pose.position.z + 0.195;

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
  ROS_INFO("publish_tf_apriltag_pose_to_predictor_gripper.cpp running...");
  
  ros::init(argc, argv, "publish_tf_apriltag_pose_to_predictor_gripper"); //Init ROS
  PoseDrawer pd; //Construct class
  ros::spin(); // Run until interupted 
  return 0;
};