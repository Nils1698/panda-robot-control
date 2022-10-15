// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
* Modified version for active inference
*
* Author: Corrado Pezzato
* Date 09-09-2019
*
* This script implements an active inference controller for the Panda Franka
* Emika through the ROS framework.
*
*/

#include <franka_aic/AIC_controller.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka/gripper.h>

// TODO:
// Packages
// #include "std_msgs/MultiArrayLayout.h"
// #include "std_msgs/MultiArrayDimension.h"
// #include "std_msgs/Float32MultiArray.h"

#include <sstream>
#include <vector>
#include <iterator>
#include <array>
#include <cmath>
#include <memory>
#include <string>


#include <hardware_interface/hardware_interface.h>

#include <tf/transform_listener.h>

namespace franka_aic {
// It's a problem that the function is static, seen as if multiple instances of the
// AICController class is made, the callback function might update something for 
// all the other instances of the class
// as long as there are no static variables I think there isn't a problem, I hope!

bool AICController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names; // TODO: Why is definition not in .h file?
  std::string arm_id;                   // TODO: Why is definition not in .h file?

  // Subscribers
//  refSub = node_handle.subscribe(
//      "ref_pos",20, &AICController::ref_callback, this,
//      ros::TransportHints().reliable().tcpNoDelay());

  // HIRO Subscriber stuff
  _target_sub_task_space = node_handle.subscribe("/desktop/target_pose_cov_stmp_panda_link0", 10, &AICController::targetCartesianPoseCb, this,
                                                 ros::TransportHints().reliable().tcpNoDelay());


  // node_handle.param<std::string>("/robot_description",urdf_param, std::string("min straeng"));
  // ROS_INFO("printing parameter: /robot_description\n%s",urdf_param.c_str());

  // node_handle.param<std::string>("/xacro_param",      urdf_param, std::string("min straeng"));
  // ROS_INFO("printing parameter: /xacro_param\n%s",urdf_param.c_str());

  // node_handle.param<std::string>("/urdf_xacro_param", urdf_param, std::string("min straeng"));
  // ROS_INFO("printing parameter: /urdf_xacro_param\n%s",urdf_param.c_str());

  // node_handle.param<std::string>("/robot_description",urdf_param);
  // ROS_INFO("printing parameter: /robot_description\n%s",urdf_param.c_str());

  // node_handle.param<std::string>("/xacro_param",      urdf_param);
  // ROS_INFO("printing parameter: /xacro_param\n%s",urdf_param.c_str());

  // node_handle.param<std::string>("/urdf_xacro_param", urdf_param);
  // ROS_INFO("printing parameter: /urdf_xacro_param\n%s",urdf_param.c_str());


  // // get parameters and trajectory method selection
  int traj_method_int = 1;
  node_handle.getParam("/hiro_panda/trajectory_method", traj_method_int);
  traj_method = (TrajectoryMethod) traj_method_int;

  // node_handle.param<std::string>("/robot_description", urdf_param, std::string("min straeng"));
  // ROS_INFO("AIC_controller.cpp\n%s",urdf_param.c_str());
  
  // refSub = node_handle.subscribe<std_msgs::Float32MultiArray> ("ref_pos",1, boost::bind(&AICController::ref_callback,_1, &test));

  // Publishers
//  aPub1                 = node_handle.advertise<std_msgs::Float64>("AICactions1", 10);
//  aPub2                 = node_handle.advertise<std_msgs::Float64>("AICactions2", 10);
//  aPub3                 = node_handle.advertise<std_msgs::Float64>("AICactions3", 10);
//  aPub4                 = node_handle.advertise<std_msgs::Float64>("AICactions4", 10);
//  aPub5                 = node_handle.advertise<std_msgs::Float64>("AICactions5", 10);
//  aPub6                 = node_handle.advertise<std_msgs::Float64>("AICactions6", 10);
//  aPub7                 = node_handle.advertise<std_msgs::Float64>("AICactions7", 10);
  pub_limited_j_cmds    = node_handle.advertise<sensor_msgs::JointState>("aic_limited_j_cmds",10);
  pub_j_cmds            = node_handle.advertise<sensor_msgs::JointState>("aic_j_cmds",10);
  pub_mu                = node_handle.advertise<sensor_msgs::JointState>("aic_theta_hat_star",10);
  pub_mu_dot            = node_handle.advertise<sensor_msgs::JointState>("aic_theta_dot_hat_star",10);
  pub_mu_d              = node_handle.advertise<sensor_msgs::JointState>("aic_theta_r",10);
  pub_u                 = node_handle.advertise<sensor_msgs::JointState>("aic_u_0",10);
  pub_seq = 0;

  _limited_j_cmds.header.seq = pub_seq;
  _limited_j_cmds.header.frame_id = "joint_space";
  _limited_j_cmds.name.resize(7);
  _limited_j_cmds.position.resize(7);
  _limited_j_cmds.velocity.resize(7);
  _limited_j_cmds.effort.resize(7);

  _j_cmds.header.seq = pub_seq;
  _j_cmds.header.frame_id = "joint_space";
  _j_cmds.name.resize(7);
  _j_cmds.position.resize(7);
  _j_cmds.velocity.resize(7);
  _j_cmds.effort.resize(7);

  _mu.header.seq = pub_seq;
  _mu.header.frame_id = "joint_space";
  _mu.name.resize(7);
  _mu.position.resize(7);
  _mu.velocity.resize(7);
  _mu.effort.resize(7);

  _mu_dot.header.seq = pub_seq;
  _mu_dot.header.frame_id = "joint_space";
  _mu_dot.name.resize(7);
  _mu_dot.position.resize(7);
  _mu_dot.velocity.resize(7);
  _mu_dot.effort.resize(7);

  _mu_d.header.seq = pub_seq;
  _mu_d.header.frame_id = "joint_space";
  _mu_d.name.resize(7);
  _mu_d.position.resize(7);
  _mu_d.velocity.resize(7);
  _mu_d.effort.resize(7);

  _u.header.seq = pub_seq;
  _u.header.frame_id = "joint_space";
  _u.name.resize(7);
  _u.position.resize(7);
  _u.velocity.resize(7);
  _u.effort.resize(7);

  _limited_j_cmds.name  = jointNames;
  _j_cmds.name          = jointNames;
  _mu.name              = jointNames;
  _mu_dot.name          = jointNames;
  _mu_d.name            = jointNames;
  _u.name               = jointNames;

  ROS_WARN("AICController: Ready to start the magic?!");

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("AICController: Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("AICController: Invalid or no joint_names parameters provided, aborting controller init!");
    return false;
  }

  // joint names check
  if (!node_handle.getParam("joint_names", joint_names)){
    ROS_ERROR("PandaPoseController: Could not parse joint names");
    return false;
  }

  if (joint_names.size() != 7){
    ROS_ERROR_STREAM("PandaPoseController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }

//  Initialize hardware interface:
  franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("AICController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "AICController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("AICController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "AICController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("AICController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) { // NOTE: HIRO uses position joint interface, named _position_joint_handles. This pushback call corresponds to their getHandle() call on lines 61 to 75 in panda_pose_controller.cpp
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("AICController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // try {// Initialise gripper

//     franka::Gripper gripper(arm_id);

  //   double grasping_width = 0.08;
  //   gripper.move(grasping_width,0.1);

  // } catch (franka::Exception const& e) {
  //   std::cout << e.what() << std::endl;
  //   return false;
  // }


  // hiro panda trac ik service
  _panda_ik_service = franka_aic::PandaTracIK();
  // Does the fact that PandaTracIK is part of the same namespace make a difference?
  // 
  _is_executing_cmd = false;

  return true;
}

void AICController::starting(const ros::Time& /*time*/) {

  // Initialise the controller's parameters
  franka::RobotState robot_state = state_handle_->getRobotState();



  // mu_d << 0,0,0,1.5708, 0, 1.8675, 0; //ref_p; // TODO

  // Center
  // Setting Initial position to a safe location:
  // mu_d << 0.0345169,-0.167844,-0.000332711,-1.74526,0.00905814,1.56589,0.824567;
  mu_d << 0.0,0.0, 0.0, -1.57, 0.0, 1.57, -1.57;

  // Right
  //mu_d << 0.15086,0.235858,-0.458875,-2.44786,0.0599695,2.6241,0.450365;

  //////////// Controller tuning //////////////
  // Variances associated with the beliefs and the sensory inputs
  //var_mu = 5.0;
  //var_muprime = 10.0;
  // For no gravity use these
  var_mu = 15.0;
  var_muprime = 30.0;
  var_q = 1;
  var_qdot = 1;

  // Learning rates for the gradient descent
  k_mu = 11.67;
  k_a = 200;

  // Integration step
  h = 0.001;

  // Precision matrices (first set them to zero then populate the diagonal)
  SigmaP_yq0 = Eigen::Matrix<double, 7, 7>::Zero();
  SigmaP_yq1 = Eigen::Matrix<double, 7, 7>::Zero();
  SigmaP_mu = Eigen::Matrix<double, 7, 7>::Zero();
  SigmaP_muprime = Eigen::Matrix<double, 7, 7>::Zero();
  //////////////////////////////////////////////

  /// Initialization
  // Initialize Angular Position Reference returned from IK:
  _joints_result.resize(7);
  for( int i = 0; i < 7; i = i + 1 ) {
    SigmaP_yq0(i,i) = 1/var_q;
    SigmaP_yq1(i,i) = 1/var_qdot;
    SigmaP_mu(i,i) = 1/var_mu;
    SigmaP_muprime(i,i) = 1/var_muprime;
    // Internal belief starting from initial pose
    jointPos(i) = robot_state.q[i];
    jointVel(i) = robot_state.dq[i];
    _joints_result(i) = jointPos(i);      // Initialize angular position reference | _position_joint_handles[i].getPosition();
    jointPosPrev(i) = _joints_result(i);  // Initialize previous angular position reference
    _iters[i] = 0; /// Only used in TrapezoidVel trajectory method
  }

  // Re-tunin of last joints to have less jitter, another equivalent approach is to have a vector for k_a
  SigmaP_yq0(4,4) = 0.1 *1/var_q;
  SigmaP_yq1(4,4) = 0.1 *1/var_qdot;

  SigmaP_yq0(5,5) = 0.05*1/var_q;
  SigmaP_yq1(5,5) = 0.05*1/var_qdot;

  SigmaP_yq0(6,6) = 0.01*1/var_q;
  SigmaP_yq1(6,6) = 0.01*1/var_qdot;

  // Initial belief
  mu = jointPos;
  mu_p = jointVel;
  mu_pp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // Initial control actions
  u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Set initial time
  time_passed = 0.0;

  // Set via-points
  mu_dRight << 0.15,0.23,-0.45,-2.40,0.0,2.6,0.45;
  mu_dGrab << 0.15,0.4,-0.45,-2.4,0.17,2.65,0.35;
  mu_dCenter << 0.0,-0.16,-0.00,-1.75,0.00,1.5,0.80;
  mu_dRelease << 0.75,0.35,-0.4,-2.4,0.25,2.75,0.90;
}

void AICController::update(const ros::Time& time, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  interval_length = period.toSec();
  time_passed += interval_length;
  //  time_passed += period.toSec();
  // Save the robot state for algebric manipulation
  for( int i = 0; i < 7; i = i + 1 ) {
    // Set current sensory input
    jointPos(i) = robot_state.q[i];   // Measured joint position
    jointVel(i) = robot_state.dq[i];  // Measured joint velocity
  }

  //  Get most recent desired angular position from inverse kinematics
  for (int i = 0; i < 7; i++){
//    ROS_INFO("Before: _joint_cmds[%d] = _joints_result(%d);\n",i,i);
//    ROS_INFO("Before: %f = %f;\n",_joint_cmds[i],_joints_result(i));

    _joint_cmds[i] = _joints_result(i);
//    ROS_INFO("After : _joint_cmds[%d] = _joints_result(%d);\n",i,i);
//    ROS_INFO("After : %f = %f;\n",_joint_cmds[i],_joints_result(i));
//    ROS_INFO("_joints_result(%d): %f",i,_joints_result(i));
  }

  // if goal is reached and robot is originally executing command, we are done
  if (_isGoalReached() && _is_executing_cmd){
    _is_executing_cmd = false;
  }

  /// TrapezoidVel trajectory method:
  // increase velocity linearly to a certain point, maintain it,
  // then linearly decrease to 0 near the end of the motion
  trapezoidVelCmd(max_velo, v_change);

  /// set position reference for low level controller:
  for (int i=0; i<7; i++)
  {
    mu_d (i) = _limited_joint_cmds[i];


  }

  // This is the filter used in the MRAC, but for the AIC, it just seems to make it unstable
//  mu_d      = 0.005 * mu       + mu_d_prev;
//  mu_d_dot  = 0.998 * mu_d_dot + 0.001998 * (mu_d - mu_d_prev) / h; // emulating velocity reference // TODO: Replace with inverse dynamics when available
//  mu_d_prev = mu_d;

  mu_d_dot  = (mu_d - mu_d_prev) / h; // emulating velocity reference // TODO: Replace with inverse dynamics when available
  mu_d_prev = mu_d;

  /// CatmullRomVel trajectory method: // Not working for non-zero x-position commands
  // best and most recommended method for trajectory computation after
  // inverse kinematics calculations
  // normalize position, we need to do this separately for every joint
//  double current_pos, p_val;
//  for (int i=0; i<7; i++)
//  {
//    // norm position
//    p_val = 2 - (2 * (abs(_joint_cmds[i] - jointPos(i)) / abs(calc_max_pos_diffs[i])));
//    // if p val is negative, treat it as 0
//    p_val = std::max(p_val, 0.);
//    catmullRomSplineVelCmd(p_val, i, interval_length);
//  }



  // // Set here the via-points
  // if (time_passed >= 0.0 && time_passed < 6.0){
  //   mu_d = mu_dCenter;
  //   //franka::Gripper::move(0.08,0.1)
  // }
  // if (time_passed >= 6.0 && time_passed < 12.0){
  //   mu_d = mu_dRight;
  //   //franka::Gripper::move(0.01,0.1)
  // }
  // if (time_passed >= 12.0 && time_passed < 18.0){
  //   mu_d = mu_dCenter;
  //   //franka::Gripper::move(0.10,0.1)
  // }
  // if (time_passed >= 18.0 && time_passed < 24.0){
  //   mu_d = mu_dRelease;
  //   //franka::Gripper::move(0.04,0.1)
  // }
  // //if (time_passed >= 24.0){
  //   //mu_d = mu_dCenter;
  // //}

  /////////// Active inference /////////// 
  // Constant Position Reference State Estimator
  // Belief update
  mu_dot = mu_p - k_mu*(-SigmaP_yq0*(jointPos-mu)+SigmaP_mu*(mu_p+mu-mu_d));
  mu_dot_p = mu_pp - k_mu*(-SigmaP_yq1*(jointVel-mu_p)+SigmaP_mu*(mu_p+mu-mu_d)+SigmaP_muprime*(mu_pp+mu_p-mu_d_dot));
  mu_dot_pp = - k_mu*(SigmaP_muprime*(mu_pp+mu_p-mu_d_dot));

  mu = mu + h*mu_dot;             // Belief about the position
  mu_p = mu_p + h*mu_dot_p;       // Belief about motion of mu
  mu_pp = mu_pp + h*mu_dot_pp;    // Belief about motion of mu'

  // Compute control actions
  u = u-h*k_a*(SigmaP_yq1*(jointVel-mu_p)+SigmaP_yq0*(jointPos-mu));

//  // Publish control actions
//  //for (int i=0;i<7;i++){
//     a1.data = u(0);
//     a2.data = u(1);
//     a3.data = u(2);
//     a4.data = u(3);
//     a5.data = u(4);
//     a6.data = u(5);
//     a7.data = u(6);
//  //}
//  aPub1.publish(a1);
//  aPub2.publish(a2);
//  aPub3.publish(a3);
//  aPub4.publish(a4);
//  aPub5.publish(a5);
//  aPub6.publish(a6);
//  aPub7.publish(a7);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(u(i));
  }

  _limited_j_cmds.header.stamp       = time;
  _limited_j_cmds.header.seq         = pub_seq;
  _limited_j_cmds.position = std::vector<double>(_limited_joint_cmds.begin(),_limited_joint_cmds.end());

  _j_cmds.header.stamp       = time;
  _j_cmds.header.seq         = pub_seq;
  _j_cmds.position = std::vector<double>(_joint_cmds.begin(),_joint_cmds.end());

  _mu_d.header.stamp       = time;
  _mu_d.header.seq         = pub_seq;
  Eigen::VectorXd::Map(&_mu_d.position[0], mu_d.size())     = mu_d;

  _mu.header.stamp       = time;
  _mu.header.seq         = pub_seq;
  Eigen::VectorXd::Map(&_mu.position[0], mu.size())     = mu;
  Eigen::VectorXd::Map(&_mu.velocity[0], mu_p.size())   = mu_p;
  Eigen::VectorXd::Map(&_mu.effort[0]  , mu_pp.size())  = mu_pp;

  _mu_dot.header.stamp       = time;
  _mu_dot.header.seq         = pub_seq;
  Eigen::VectorXd::Map(&_mu_dot.position[0], mu.size())     = mu;
  Eigen::VectorXd::Map(&_mu_dot.velocity[0], mu_p.size())   = mu_p;
  Eigen::VectorXd::Map(&_mu_dot.effort[0]  , mu_pp.size())  = mu_pp;

  _u.header.stamp       = time;
  _u.header.seq         = pub_seq;
  Eigen::VectorXd::Map(&_u.position[0], u.size()) = u;

  pub_limited_j_cmds.publish(_limited_j_cmds);
  pub_j_cmds.publish(_j_cmds);
  pub_mu.publish(_mu);
  pub_mu_dot.publish(_mu_dot);
  pub_mu_d.publish(_mu_d);
  pub_u.publish(_u);

  jointPosPrev = jointPos;
  ++pub_seq;
}

void AICController::stopping(const ros::Time &time){
    // can't send immediate commands for 0 velocity to robot
}

// void AICController::ref_callback(const std_msgs::Float32MultiArray::ConstPtr& ref, 
//                                         Eigen::Matrix<double, 7, 1>* hest){
//   // for (int i = 0; i < 7; i++)
//   //   mu_d(i,0)  = ref->data[i];

// }
void AICController::ref_callback(const std_msgs::Float32MultiArray::ConstPtr& ref){
  for (int i = 0; i < 7; i++){
    ref_p(i,0)  = ref->data[i];
    // ROS_INFO("%d: %s in callback",i,std::to_string(ref_p(i,0)).c_str());
  }
}

// Functions added from HIRO 

// AICController 


void AICController::targetCartesianPoseCb(const geometry_msgs::PoseWithCovarianceStamped &target_pose)
{

//  buffer << "mu_d \n" << mu_d << "\n_limited_joint_cmds" << std::endl; // "\nik_result: \n" << ik_result <<
//  ROS_INFO("%s",buffer.str().c_str());
//  for (int i = 0; i < 7; ++i) {
//    ROS_INFO("_limited_joint_cmds(%d): %f",i,_limited_joint_cmds[i]);
//  }
//  for (int i = 0; i < 7; ++i) {
//    ROS_INFO("_joint_cmds(%d): %f",i,_joint_cmds[i]);
//  }
//  for (int i = 0; i < 7; ++i) {
//  }
//
//  ROS_INFO("_joint_cmds = _joints_result;\n");
//  for (int i = 0; i < 7; ++i) {
//    ROS_INFO("%f = %f;\n",_joint_cmds[i],_joints_result(i));
//  }
//  ROS_INFO("case_used_in_trapezoid_vel: %d",case_used_in_trapezoid_vel);
//
//  buffer.str("");

//  if (_is_executing_cmd) // TODO: Denne funktion deaktiveres for nu da jeg skal kunne opdatere task space mål positionen live, via kalman filteret
//  {
//    ROS_ERROR("Panda Pose Controller: Still executing command! returning");
//    ROS_INFO("_epsilon: %f, err: %f, Goal reached: %d",_epsilon,err,err <= _epsilon); // TODO: Delete when done debugging
//
//    return;
//    // panda is still executing command, cannot publish yet to this topic.
//  }
  _target_pose = target_pose.pose.pose;

// Get the original orientation of 'commanded_pose'
  tf2::convert(_target_pose.orientation , q_orig);

//  double r=3.14159, p=0, y=0;  // Rotate the previous pose by 180* about X
  double r=0, p=3.14159, y=0;  // Rotate the previous pose by 180* about X
  q_rot.setRPY(r, p, y);

  q_new = q_rot*q_orig;  // Calculate the new orientation
  q_new.normalize();

// Stuff the new rotation back into the pose. This requires conversion into a msg type
  tf2::convert(q_new, _target_pose.orientation);
  _target_pose.orientation.w = q_new.getW();
  _target_pose.orientation.x = q_new.getX();
  _target_pose.orientation.y = q_new.getY();
  _target_pose.orientation.z = q_new.getZ();
//
// _target_pose.position.x = target_pose.position.x;
// _target_pose.position.y = target_pose.position.y;
// _target_pose.position.z = target_pose.position.z;

  // use tracik to get joint positions from target pose
  ik_result = _panda_ik_service.perform_ik(_target_pose);
  // NOTE: The print from this for-loop and for-loop IK fun 2
  // shows that the _joints_result is indeed updated by ik_result
//  for (int i = 0; i < 7; i++){
//    ROS_INFO("IK fun 1: _joints_result(%d): %f",i,_joints_result(i));
//  }
  if (_panda_ik_service.is_valid) {
    _joints_result = ik_result;
  } else {
    _joints_result = _joints_result;
  }

//  for (int i = 0; i < 7; i++){ //For-loop IK fun 2
//    ROS_INFO("IK fun 2: _joints_result(%d): %f",i,_joints_result(i));
//  }
  ROS_INFO("_joints_result: %d",_panda_ik_service.is_valid);
  if (_joints_result.rows() != 7)
  {
    ROS_ERROR("Panda Pose Controller: Wrong Amount of Rows Received From TRACIK");
    return;
  }
  // for catmull rom spline, break up into two splines
  // spline 1 - increases to max provided velocity
  // spline 2 - decreases down to 0 smoothly
//  double max_vel;
//  for (int i=0; i<7; i++)
//  {
//    // get current position
//    double cur_pos = joint_handles_[i].getPosition();
//    // difference between current position and desired position from ik
//    calc_max_pos_diffs[i] = _joints_result(i) - cur_pos;
//    // if calc_max_poss_diff is negative, flip sign of max vel
//    max_vel = calc_max_pos_diffs[i] < 0 ? -_max_abs_vel : _max_abs_vel;
//    int sign = calc_max_pos_diffs[i] < 0 ? -1 : 1;
//    // get p_i-2, p_i-1, p_i, p+i+1 for catmull rom spline
//
//    velocity_points_first_spline = {0, 0.3*sign, max_vel, max_vel};
//    velocity_points_second_spline = {max_vel, max_vel, 0, 0};
//
//    // tau of 0.3
//
//    // velocity as function of position
//    _vel_catmull_coeffs_first_spline[i] = catmullRomSpline(0.3, velocity_points_first_spline);
//    _vel_catmull_coeffs_second_spline[i] = catmullRomSpline(0.3, velocity_points_second_spline);
//
//  }
  _is_executing_cmd = true;
  for (int i = 0; i < 7; i++)
  {
     _iters[i] = 0;
  }
//  _start_time = ros::Time::now();
}

//std::array<double, 4> AICController::catmullRomSpline(const double &tau, const std::array<double,4> &points)
//{
//    // catmullRomSpline calculation for any 4 generic points
//    // result array for 4 coefficients of cubic polynomial
//    std::array<double, 4> coeffs;
//    // 4 by 4 matrix for calculating coefficients
//    std::array<std::array<double, 4>, 4> catmullMat = {{{0, 1, 0, 0}, {-tau, 0, tau, 0},
//                                                        {2*tau, tau-3, 3 - (2*tau), -tau},
//                                                        {-tau,2-tau,tau-2,tau}}};
//    // calculation of coefficients
//    for (int i=0; i<4; i++)
//    {
//        coeffs[i] = (points[0]*catmullMat[i][0]) + (points[1]*catmullMat[i][1])
//                    + (points[2]*catmullMat[i][2]) + (points[3]*catmullMat[i][3]);
//    }
//    return coeffs;
//
//}

//double AICController::calcSplinePolynomial(const std::array<double,4> &coeffs, const double &x)
//{
//    // function that calculates third degree polynomial given input x and
//    // 4 coefficients
//    double output = 0.;
//    int power = 0;
//    for (int i=0; i<4; i++)
//    {
//        output+=(coeffs[i]*(pow(x, power)));
//        power++;
//    }
//    return output;
//}

//void AICController::catmullRomSplineVelCmd(const double &norm_pos, const int &joint_num,
//                                                    const double &interval)
//{
//    // individual joints
//    // velocity is expressed as a function of position (position is normalized)
//    if (norm_pos <= 2 && _is_executing_cmd)
//    {
//        // valid position and is executing a command
//        double vel;
//        if (norm_pos < 1)
//        {
//            // use first spline to get velocity
//            vel = calcSplinePolynomial(_vel_catmull_coeffs_first_spline[joint_num], norm_pos);
//
//        }
//        else
//        {
//            // use second spline to get velocity
//            vel = calcSplinePolynomial(_vel_catmull_coeffs_second_spline[joint_num], norm_pos - 1);
//        }
//
//        // calculate velocity step
//        _limited_joint_cmds[joint_num] = jointPosPrev(joint_num) + (vel * interval);
//    }
//    else
//    {
//        for (int i=0; i<7; i++)
//        {
//            _limited_joint_cmds[i] = _joint_cmds[i];
//        }
//
//        _is_executing_cmd = false;
//    }
//
//}

bool AICController::_isGoalReached()
{
    // sees if goal is reached given a distance threshold epsilon
    // l2 norm is used for overall distance
    err = 0;
    for (int i = 0; i < 7; i++)
    {
        err += pow(joint_handles_[i].getPosition() - _joints_result(i), 2);
    }
    err = sqrt(err);
//    ROS_INFO("_epsilon: %f, err: %f, Goal reached: %d",_epsilon,err,err <= _epsilon); // TODO: Delete when done debugging

    return err <= _epsilon;

}

//void AICController::constantVelCmd(const double &min_step)
//{
//
//    for (int i = 0; i < 7; i++)
//    {
//        // difference between joint command and position, will be big initially
//        double diff = _joint_cmds[i] - joint_handles_[i].getPosition();
//        if (abs(diff) > min_step)
//        {
//            // difference is too big, reduce to min step instead, min step is rad/s
//            _limited_joint_cmds[i] = jointPosPrev(i) + min_step * (diff > 0 ? 1.0 : -1.0);
//        }
//        else
//        {
//            // difference is small, near the end, send the raw command
//            _limited_joint_cmds[i] = _joint_cmds[i];
//        }
//    }
//}
//
void AICController::trapezoidVelCmd(const double &max_step, const double &v_change)
{
//    ROS_INFO("trapezoidVelCmd");
    // generates waypoints that increase velocity, then decrease
    step_time = max_step / v_change;
    // time to go from 0 to min_step for each time step
    change_dist = step_time * max_step * 0.5;

    for (int i = 0; i < 7; i++)
    {
        double diff = _joint_cmds[i] - joint_handles_[i].getPosition();

        if (_iters[i] == 0 && abs(diff) <= change_dist)
        {
          case_used_in_trapezoid_vel = 1;// TODO: Delete when done debugging
            // case where we have new trajectory, but it doesn't need the whole path
            _limited_joint_cmds[i] = _joint_cmds[i];
        }

        else if (abs(diff) <= change_dist)
        {
          case_used_in_trapezoid_vel = 2;// TODO: Delete when done debugging
            // when distance is small enough, decrease _iter value to 0 in decreased velocity
            _iters[i] -= v_change;
            _iters[i] = std::max(_iters[i], 0.);
            _limited_joint_cmds[i] = jointPosPrev(i) + _iters[i] * (diff > 0 ? 1.0 : -1.0);

        }
        else if (abs(diff) > change_dist)
        {
          case_used_in_trapezoid_vel = 3;// TODO: Delete when done debugging
            // distance is big, so increase velocity to max, if it's max,
            // send max command to controller from prev. pos.
            _iters[i] += v_change;
            _iters[i] = std::min(_iters[i], max_step);
            _limited_joint_cmds[i] = jointPosPrev(i) + _iters[i] * (diff > 0 ? 1.0 : -1.0);
        }
    }
}

// std::string get_urdf_param(){
//   return AICController::urdf_param;
// }

}  // namespace franka_aic

PLUGINLIB_EXPORT_CLASS(franka_aic::AICController,
                       controller_interface::ControllerBase)
