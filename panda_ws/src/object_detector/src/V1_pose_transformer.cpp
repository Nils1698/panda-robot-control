#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>

class PoseTransformerCam2Panda{
  public:
  ros::Subscriber       sub;
  ros::Publisher        pub_pose_cov_stmp;
  ros::Publisher        pub_pose;

  geometry_msgs::Pose   target_pose_panda_link_0;
  geometry_msgs::Pose   target_pose_cov_stmp_panda_link_0;

  PoseTransformerCam2Panda(ros::NodeHandle *n){
    pub_pose_cov_stmp = n->advertise<geometry_msgs::PoseWithCovarianceStamped>
      ("/target_pose_cov_stmp_panda_link_0",10);
    pub_pose = n->advertise<geometry_msgs::Pose>
      ("/target_pose_panda_link_0",10);

    sub = n->subscribe("/target_detection_camera", 1000, 
      &PoseTransformerCam2Panda::transformPose,this); 
  }

  void transformPose(const tf::TransformListener& listener){
    //we'll create a point in the base_laser frame that we'd like to transform to the panda_link_0 frame
    geometry_msgs::Pose camera_pose;
    camera_pose.header.frame_id = "camera"
    // geometry_msgs::PointStamped laser_point;
    // laser_point.header.frame_id = "base_laser";

    //we'll just use the most recent transform available for our simple example
    laser_point.header.stamp = ros::Time();

    // //just an arbitrary point in space
    // laser_point.point.x = 1.0;
    // laser_point.point.y = 0.2;
    // laser_point.point.z = 0.0;

    try{
      geometry_msgs::Pose panda_link_0_pose;
      listener.transformPoint("panda_link_0", camera_pose, panda_link_0_pose);
      // geometry_msgs::PointStamped base_point;
      // listener.transformPoint("panda_link_0", laser_point, base_point);
      // TODO: Publish the transformed detection
      // ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> panda_link_0: (%.2f, %.2f, %.2f) at time %.2f",
      //     laser_point.point.x, laser_point.point.y, laser_point.point.z,
      //     base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"panda_link_0\": %s", ex.what());
    }
  }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_detection_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}

