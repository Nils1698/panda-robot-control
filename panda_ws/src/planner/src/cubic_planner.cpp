#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
// #include <Eigen/Core>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <predictor/PredictedPoses.h>

#include <string>
#include <iostream>
#include <sstream>

#include "planner/cubic_trajectory_generator/cubic_trajectory_generator.h"
#include "planner/cubic_trajectory_generator/cubic_trajectory_generator_terminate.h"
#include "planner/cubic_trajectory_generator/rt_nonfinite.h"


class Planner {

  private:
    const static int horizon    = 60;  // Prediction horizon
    const static int pred_order = 2;   // Prediction order 

    ros::Publisher pub;
    ros::Subscriber measurement_subscriber;
    ros::ServiceServer reset_service;
    // predictor::PredictedPoses predicted_poses; // prediction message for publishing
    // geometry_msgs::PoseWithCovariance = goal_object_pose_task_space;
    
    // std::vector<geometry_msgs::PoseWithCovariance> = 
    predictor::PredictedPoses predicted_poses; // prediction message for publishing

    std::stringstream buffer; // My debugging buffer string for printing

  public:
  Planner(ros::NodeHandle *nh)
  { // Constructor

    // Publisher, Subscriber & Service initialization
    pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/goal_object_pose_task_space", 10);   
    pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/target_pose_task_space", 10);   
    measurement_subscriber = nh->subscribe("/predicted_poses", 1000, 
        &Planner::callback_measurement, this);

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

    buffer << predicted_poses.poses.size() << std::endl;
    ROS_INFO("predicted_poses.poses.size(): \n%s",buffer.str().c_str());
    buffer.str("");
    ROS_INFO("The Predictor object was Constructed");
  }


  void callback_measurement(const predictor::PredictedPoses& msg) {
      predicted_poses = msg;

      // predicted_poses.header = msg.header; // Copy header
      // y(0,0) = msg.pose.pose.position.x; // x position
      // y(1,0) = msg.pose.pose.position.y; // y position
      // y(2,0) = msg.pose.pose.position.z; // z position
      // // ROS_INFO("We heard position x was: %f",y(0,0));
      // // ROS_INFO("We heard position y was: %f",y(1,0));
      // // ROS_INFO("We heard position z was: %f",y(2,0));
      // y_orientation(0,0) = msg.pose.pose.orientation.x;
      // y_orientation(1,0) = msg.pose.pose.orientation.y;
      // y_orientation(2,0) = msg.pose.pose.orientation.z;
      // y_orientation(3,0) = msg.pose.pose.orientation.w;
      // // y(0,0) += 10.0; // Add 10 to the measurement artificially
      // new_measurement_acquired = true;
      // first_measurement_callback = true;
  }


};




int main (int argc, char **argv)
{
    ros::init(argc, argv, "cubic_planner");
    ros::NodeHandle nh;
    Planner nc = Planner(&nh);
    // nc.predictive_kalman_filter();

    ros::spin();
}

