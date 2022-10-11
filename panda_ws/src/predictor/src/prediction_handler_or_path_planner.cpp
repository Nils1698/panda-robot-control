#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
// #include <Eigen/Core>
// #include <Eigen/Core>
// #include <unsupported/Eigen/MatrixFunctions>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <predictor/PredictedPoses.h>

#include <string>
#include <iostream>
#include <sstream>

class PredictionHandler{
  private:
  ros::Publisher  pub;
  ros::Subscriber sub;
  ros::NodeHandle n;
  predictor::PredictedPoses predicted_poses;
  geometry_msgs::PoseWithCovarianceStamped single_target_pose;
  std::stringstream buffer; // My debugging buffer string for printing
  int target_prediction_step = 20; // The prediction step which should be passed onto the inverse kinematics solver
  int target_prediction_order = 0; // 0 to get position prediction, 1 to get velocity prediction. This parameter will probably not be used when a target selector using both the position and velocity prediction is implemented. 
  const static int horizon    = 60;  // Prediction horizon
  const static int pred_order = 2;   // Prediction order                             // position and velocity, for now. In future acceleration and jerk may also be predicted.
  public:

  PredictionHandler(){
    sub = n.subscribe("/predicted_poses",1000,&PredictionHandler::msgCallback, this);
    pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>
      ("/target_pose_cov_stamped_panda_link0",10);

    predicted_poses.layout.dim.push_back(std_msgs::MultiArrayDimension()); // Increase dimension of predicted pose matrix by 1, to 1.
    predicted_poses.layout.dim.push_back(std_msgs::MultiArrayDimension()); // Increase dimension of predicted pose matrix by 1, to 2.
    predicted_poses.layout.dim[0].label  = "rows, prediction step";
    predicted_poses.layout.dim[0].size   = horizon;
    predicted_poses.layout.dim[0].stride = horizon*pred_order;
    predicted_poses.layout.dim[1].label  = "columns, prediction order"; // xp,yp,zp,xv,yv,zv
    predicted_poses.layout.dim[1].size   = pred_order;
    predicted_poses.layout.dim[1].stride = pred_order;
    predicted_poses.poses.push_back(geometry_msgs::PoseWithCovariance()); // This line has a compilation error. Do I even need to "push_back" as this array?
    predicted_poses.poses.resize(pred_order*horizon);
  }

  void msgCallback(const predictor::PredictedPoses& msg) 
  {
    single_target_pose.header                   = msg.header;
    single_target_pose.pose.pose.position.x     = msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.position.x;
    single_target_pose.pose.pose.position.y     = msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.position.y;
    single_target_pose.pose.pose.position.z     = msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.position.z;
    single_target_pose.pose.pose.orientation.x  = msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.orientation.x;
    single_target_pose.pose.pose.orientation.y  = msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.orientation.y;
    single_target_pose.pose.pose.orientation.z  = msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.orientation.z;
    single_target_pose.pose.pose.orientation.w  = msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.orientation.w;
    
    // If the target is further than 60cm away from the base of the robot arm in the 
    // xy plane use instead a default target position untill the target gets within range
    if (std::sqrt(std::pow(single_target_pose.pose.pose.position.x,2) + std::pow(single_target_pose.pose.pose.position.y,2)) > 0.8){
      single_target_pose.pose.pose.position.x     = 0.7;  //msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.position.x;
      single_target_pose.pose.pose.position.y     = -0.2;  //msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.position.y;
      single_target_pose.pose.pose.position.z     = 0.5;  //msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.position.z;
      single_target_pose.pose.pose.orientation.x  = 0;    //msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.orientation.x;
      single_target_pose.pose.pose.orientation.y  = 0;    //msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.orientation.y;
      single_target_pose.pose.pose.orientation.z  = 1;    //msg.poses[target_prediction_step*pred_order + target_prediction_order].pose.orientation.z;
      single_target_pose.pose.pose.orientation.w  = 0;
    }
    // single_target_pose.pose.pose.position.y
    // single_target_pose.pose.pose.position.z
    // single_target_pose.pose.pose.orientation.x 
    // single_target_pose.pose.pose.orientation.y
    // single_target_pose.pose.pose.orientation.z
    // single_target_pose.pose.pose.orientation.w
  //   geometry_msgs::PoseWithCovarianceStamped point_out;
  //   try 
  //   {
  //     buffer_.transform(*point_ptr, point_out, target_frame_);
  //     pub_.publish(point_out);
  //   }
  //   catch (tf2::TransformException &ex) 
  //   {
  //     ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
  //   }
    pub.publish(single_target_pose);
  }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "prediction_handler");
    PredictionHandler ph;
    ros::spin();
    return 0;
}