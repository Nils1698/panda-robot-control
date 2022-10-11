// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
* Modified version for active inference
*
* Author: Corrado Pezzato
* Date 09-09-2019
*
* This script implements a model reference adaptive controller for the Panda Franka
* Emika through the ROS framework.
*
*/

#include <franka_aic/MRAC_controller.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_aic {

bool MRACController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;

  _target_sub_task_space = node_handle.subscribe("/desktop/target_pose_cov_stmp_panda_link0", 10, &MRACController::targetCartesianPoseCb, this,
                                                 ros::TransportHints().reliable().tcpNoDelay());

  int traj_method_int = 1;
  node_handle.getParam("/hiro_panda/trajectory_method", traj_method_int);
  traj_method = (TrajectoryMethod) traj_method_int;

  pub_limited_j_cmds     = node_handle.advertise<sensor_msgs::JointState>("mrac_limited_j_cmds",10);  // 7 elements
  pub_j_cmds             = node_handle.advertise<sensor_msgs::JointState>("mrac_j_cmds",10);          // 7 elements
  pub_u                  = node_handle.advertise<sensor_msgs::JointState>("mrac_u",10);               // 7 elements
  pub_qr                 = node_handle.advertise<sensor_msgs::JointState>("mrac_qr",10);              // 7 elements
  pub_f                  = node_handle.advertise<sensor_msgs::JointState>("mrac_f",10);               // 7 elements
  pub_qe_integral        = node_handle.advertise<sensor_msgs::JointState>("mrac_qe_integral",10);     // 7 elements
  pub_qe_q_integral      = node_handle.advertise<sensor_msgs::JointState>("mrac_qe_q_integral",10);   // 49 elements
  pub_qe_qr_integral     = node_handle.advertise<sensor_msgs::JointState>("mrac_qe_qr_integral",10);  // 49 elements
  pub_K                  = node_handle.advertise<sensor_msgs::JointState>("mrac_K",10);               // 49 elements
  pub_Q                  = node_handle.advertise<sensor_msgs::JointState>("mrac_Q",10);               // 49 elements

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

  _u.header.seq = pub_seq;
  _u.header.frame_id = "joint_space";
  _u.name.resize(7);
  _u.position.resize(7);
  _u.velocity.resize(7);
  _u.effort.resize(7);

  _qr.header.seq = pub_seq;
  _qr.header.frame_id = "joint_space";
  _qr.name.resize(7);
  _qr.position.resize(7);
  _qr.velocity.resize(7);
  _qr.effort.resize(7);

  _qe_integral.header.seq = pub_seq;
  _qe_integral.header.frame_id = "joint_space";
  _qe_integral.name.resize(7);
  _qe_integral.position.resize(7);
  _qe_integral.velocity.resize(7);
  _qe_integral.effort.resize(7);

  _f.header.seq = pub_seq;
  _f.header.frame_id = "joint_space";
  _f.name.resize(7);
  _f.position.resize(7);
  _f.velocity.resize(7);
  _f.effort.resize(7);

  _qe_q_integral.header.seq = pub_seq;
  _qe_q_integral.header.frame_id = "joint_space";
  _qe_q_integral.name.resize(49);
  _qe_q_integral.position.resize(49);
  _qe_q_integral.velocity.resize(49);
  _qe_q_integral.effort.resize(49);

  _qe_qr_integral.header.seq = pub_seq;
  _qe_qr_integral.header.frame_id = "joint_space";
  _qe_qr_integral.name.resize(49);
  _qe_qr_integral.position.resize(49);
  _qe_qr_integral.velocity.resize(49);
  _qe_qr_integral.effort.resize(49);

  _K.header.seq = pub_seq;
  _K.header.frame_id = "joint_space";
  _K.name.resize(49);
  _K.position.resize(49);
  _K.velocity.resize(49);
  _K.effort.resize(49);

  _Q.header.seq = pub_seq;
  _Q.header.frame_id = "joint_space";
  _Q.name.resize(49);
  _Q.position.resize(49);
  _Q.velocity.resize(49);
  _Q.effort.resize(49);

  _limited_j_cmds.name  = jointNames;
  _j_cmds.name          = jointNames;
  _u.name               = jointNames;
  _qr.name              = jointNames;
  _f.name               = jointNames;
  _qe_integral.name     = jointNames;
  _qe_q_integral.name   = jointNamesMatrix;
  _qe_qr_integral.name  = jointNamesMatrix;
  _K.name               = jointNamesMatrix;
  _Q.name               = jointNamesMatrix;

  ROS_WARN("MRACController: Ready to start.");

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("MRACController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "MRACController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
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

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("MRACController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MRACController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("MRACController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "MRACController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("MRACController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("MRACController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // hiro panda trac ik service
  _panda_ik_service = franka_aic::PandaTracIK();
  // Does the fact that PandaTracIK is part of the same namespace make a difference?
  //
  _is_executing_cmd = false;

  return true;
}

void MRACController::starting(const ros::Time& /*time*/) {
  // Initialise the controller's parameters
  franka::RobotState robot_state = state_handle_->getRobotState();
  // Center
  // Set desired goal
//  qr_dRight << 0.15,0.23,-0.45,-2.40,0.0,2.6,0.45;
//  qr_dGrab << 0.15,0.4,-0.45,-2.4,0.17,2.65,0.35;
//  qr_dCenter << 0.0,-0.16,-0.00,-1.75,0.00,1.5,0.80;
//  qr_dRelease << 0.75,0.35,-0.4,-2.4,0.25,2.75,0.90;

  _joints_result.resize(7);
  for( int i = 0; i < 7; i = i + 1 ) {
    // Internal belief starting from initial pose
    jointPos(i) = robot_state.q[i];
    jointVel(i) = robot_state.dq[i];
    _joints_result(i) = jointPos(i);      // Initialize angular position reference | _position_joint_handles[i].getPosition();
    jointPosPrev(i) = _joints_result(i);  // Initialize previous angular position reference
    _iters[i] = 0; /// Only used in TrapezoidVel trajectory method
  }



  // Initialize controller
  // Set goal for velocity (always zero), the goal for the position is set by the main node MRAC_controller_node
  dqr << 0, 0, 0, 0, 0, 0, 0;
//  qr  << 0.0, 0.0, 0.0, -1.57, 0.0, 1.57, -1.57;
  // Set initial conditions for the states for the integrals
  x_qe << 0, 0, 0, 0, 0, 0, 0;
  X_qr << Eigen::Matrix<double, 7, 7>::Zero();
  X_dqr << Eigen::Matrix<double, 7, 7>::Zero();
  X_q << Eigen::Matrix<double, 7, 7>::Zero();
  X_dq << Eigen::Matrix<double, 7, 7>::Zero();

  // Integration step
  h = 0.001;

  // Initialize control actions
  u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // Set initial time
  time_passed = 0.0;

  //////////// Controller tuning //////////////
  // MRAC Variables
  omega << 2, 2, 2, 2, 1, 1, 1;
//  omega << 2, 1, 1, 4, 4, 4, 4;
  // Damping
  zeta << 1, 1, 1, 1, 1, 1, 1;
//  zeta << 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7;
//  zeta << 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9;

  // Controller parameters
// First value is for joint 0, then joint 1 and so on so if fx:
// alpha1 << 1, 2, 3, 4, 5, 6, 7; then the alpha1 value for joint0 would be 1, it would be 2 for joint1, 3 for joint 2 and so on.
  alpha1 << .2, .1, .2, .5, 1, 2, 2;
//  alpha2 << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  alpha2 << .1, .5, .1, 3., .01, .1, .1;

  e01 << 2, 2, 2, 2, 2, 2, 1;
//  e02 << .51, .51, .51, .51, .51, .1, .1;
  e02 << .02, 5., .2, 10.0, .20, .2, .1;

//  e11 << 1, 1, 1, 1, 1, 1, 1;
  e11 << .1, .1, .1, .1, .1, .1, .1;
//  e12 << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;
  e12 << .0,.0,.0,.0,.0,.0,.0;

  f01 << 5.105, 5.105, 5.105, .105, .105, .105, .105;
//  f02 << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
  f02 << .0,.0,.0,.0,.0,.0,.0;
  // Not used for constant ref
//  f11 << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  f11 << 0.01, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001;
//  f12 << 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001;
  f12 << .0,.0,.0,.0,.0,.0,.0;

  ////////////////////////////
  alpha3  << 0, 0, 0, 0, 0, 0, 0;
  e03     << 0, 0, 0, 0, 0, 0, 0;
  e13     << 0, 0, 0, 0, 0, 0, 0;
  f03     << 0, 0, 0, 0, 0, 0, 0;
  f13     << 0, 0, 0, 0, 0, 0, 0;
  l1 << 1, 1, 1, 1, 1, 1, 1;
  l2 << 1, 1, 1, 1, 1, 1, 1;

  // From here on a normal user should not modify
  p2 << l1[0]/(2*omega[0]*omega[0]), l1[1]/(2*omega[1]*omega[1]), l1[2]/(2*omega[2]*omega[2]),
        l1[3]/(2*omega[3]*omega[3]), l1[4]/(2*omega[4]*omega[4]), l1[5]/(2*omega[5]*omega[5]), l1[6]/(2*omega[6]*omega[6]);
  p3 << l2[0]/(4*zeta[0]*omega[0])+l1[0]/(4*zeta[0]*pow(omega[0],3)), l2[1]/(4*zeta[1]*omega[1])+l1[1]/(4*zeta[1]*pow(omega[1],3)),
        l2[2]/(4*zeta[2]*omega[2])+l1[2]/(4*zeta[2]*pow(omega[2],3)), l2[3]/(4*zeta[3]*omega[3])+l1[3]/(4*zeta[3]*pow(omega[3],3)),
        l2[4]/(4*zeta[4]*omega[4])+l1[4]/(4*zeta[4]*pow(omega[4],3)), l2[5]/(4*zeta[5]*omega[5])+l1[5]/(4*zeta[5]*pow(omega[5],3)),
        l2[6]/(4*zeta[6]*omega[6])+l1[6]/(4*zeta[6]*pow(omega[6],3));

  // Initial adaptive gains and constant matrices
  K0_hat = Eigen::Matrix<double, 7, 7>::Zero();
  K1_hat = Eigen::Matrix<double, 7, 7>::Zero();
  Q0_hat = Eigen::Matrix<double, 7, 7>::Zero();
  Q1_hat = Eigen::Matrix<double, 7, 7>::Zero();

  // Initialize integration parameters
  qe_integral     = Eigen::Matrix<double, 7, 1>::Zero();

  qe_q_integral   = Eigen::Matrix<double, 7, 7>::Zero();
  qe_dq_integral  = Eigen::Matrix<double, 7, 7>::Zero();
  qe_qr_integral  = Eigen::Matrix<double, 7, 7>::Zero();
  qe_dqr_integral = Eigen::Matrix<double, 7, 7>::Zero();



  // Controller parameters
  ALPHA1 = Eigen::Matrix<double, 7, 7>::Zero();
  ALPHA2 = Eigen::Matrix<double, 7, 7>::Zero();
  ALPHA3 = Eigen::Matrix<double, 7, 7>::Zero();
  E01 = Eigen::Matrix<double, 7, 7>::Zero();
  E02 = Eigen::Matrix<double, 7, 7>::Zero();
  E03 = Eigen::Matrix<double, 7, 7>::Zero();
  E11 = Eigen::Matrix<double, 7, 7>::Zero();
  E12 = Eigen::Matrix<double, 7, 7>::Zero();
  E13 = Eigen::Matrix<double, 7, 7>::Zero();
  F01 = Eigen::Matrix<double, 7, 7>::Zero();
  F02 = Eigen::Matrix<double, 7, 7>::Zero();
  F03 = Eigen::Matrix<double, 7, 7>::Zero();
  F11 = Eigen::Matrix<double, 7, 7>::Zero();
  F12 = Eigen::Matrix<double, 7, 7>::Zero();
  F13 = Eigen::Matrix<double, 7, 7>::Zero();
  P2 = Eigen::Matrix<double, 7, 7>::Zero();
  P3 = Eigen::Matrix<double, 7, 7>::Zero();

  for( int i = 0; i < P2.rows(); i = i + 1 ) {
    ALPHA1(i,i) = alpha1(i)  * 1.0;
    ALPHA2(i,i) = alpha2(i)  * 1.0;
    ALPHA3(i,i) = alpha3(i)  * 1.0;
    E01(i,i) = e01(i)        * 1.0;
    E02(i,i) = e02(i)        * 1.0;
    E03(i,i) = e03(i)        * 1.0;
//    E11(i,i) = e11(i)        * 1.0;
//    E12(i,i) = e12(i)        * 1.0;
//    E13(i,i) = e13(i)        * 1.0;
    F01(i,i) = f01(i)        * 1.0;
    F02(i,i) = f02(i)        * 1.0;
    F03(i,i) = f03(i)        * 1.0;
//    F11(i,i) = f11(i)        * 1.0;
//    F12(i,i) = f12(i)        * 1.0;
//    F13(i,i) = f13(i)        * 1.0;
    P2(i,i) = p2(i)          * 1.0;
    P3(i,i) = p3(i)          * 1.0;
//    ALPHA1(i,i) = alpha1(i)  * 0.1;
//    ALPHA2(i,i) = alpha2(i)  * 0.1;
//    ALPHA3(i,i) = alpha3(i)  * 0.1;
//    E01(i,i) = e01(i)        * 0.1;
//    E02(i,i) = e02(i)        * 0.1;
//    E03(i,i) = e03(i)        * 0.1;
    E11(i,i) = e11(i)        * 0.01;
    E12(i,i) = e12(i)        * 0.01;
    E13(i,i) = e13(i)        * 0.01;
//    F01(i,i) = f01(i)        * 0.1;
//    F02(i,i) = f02(i)        * 0.0;
//    F03(i,i) = f03(i)        * 0.1;
    F11(i,i) = f11(i)        * 0.01;
    F12(i,i) = f12(i)        * 0.01;
    F13(i,i) = f13(i)        * 0.01;
//    P2(i,i) = p2(i)          * 0.1;
//    P3(i,i) = p3(i)          * 0.1;
  }
}

void MRACController::update(const ros::Time& time, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  interval_length = period.toSec();
  time_passed += interval_length;

  // Save the robot state for algebric manipulation
  for( int i = 0; i < 7; i = i + 1 ) {
    // Set current sensory input
    jointPos(i) = robot_state.q[i];
    jointVel(i) = robot_state.dq[i];
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
    qr(i) = _limited_joint_cmds[i];
  }
  qr  =  0.005 *qr  + 0.995 * qr_prev; // LPF of position command
  dqr = +0.998*dqr + 0.001998 * (qr-qr_prev)/h; // LPF of velocity command // emulating velocity reference // TODO: Replace with inverse dynamics when available
  qr_prev = qr;

  /////////// MRAC ///////////
  // Definition of the modified joint angle error
  qe = P2*(qr-jointPos)+P3*(dqr-jointVel);

  // Feed-forward term f
  // Integral of qe using first order system as integrator
//  qe_integral = x_qe;
//  x_qe = x_qe + h*qe;
  f = ALPHA1*qe + ALPHA2*qe_integral;
  qe_integral += h*qe;
  // Adaptive gain K0
//  qe_q_integral = X_q;
//  X_q = X_q + qe*jointPos.transpose();

  K0 = K0_hat + E01*(qe*jointPos.transpose()) + E02*qe_q_integral;
  qe_q_integral += h*qe*jointPos.transpose();

  // Adaptive gain K1
//  qe_dq_integral = X_dq;
//  X_dq = X_dq + qe*jointVel.transpose();
  K1 = K1_hat + E11*(qe*jointVel.transpose()) + E12*qe_dq_integral;
  qe_dq_integral += h*qe*jointVel.transpose();

  // Adaptive gain Q0
//  qe_qr_integral = X_qr;
//  X_qr = X_qr + qe*qr.transpose();
  Q0 = Q0_hat + F01*(qe*qr.transpose()) + F02*qe_qr_integral;
  qe_qr_integral += h*qe*qr.transpose();

  // Adaptive gain K1
//  qe_dqr_integral = X_dqr;
//  X_dqr = X_dqr + qe*dqr.transpose();
  Q1 = Q1_hat + F11*(qe*dqr.transpose()) + F12*qe_dqr_integral;
  qe_dqr_integral += h*qe*dqr.transpose();

  // Torque to the robot arm
  u = (K0*jointPos + K1*jointVel + Q0*qr + Q1*dqr + f);


  // Send commands
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(u(i));
  }

  _limited_j_cmds.header.stamp       = time;
  _limited_j_cmds.header.seq         = pub_seq;
  _limited_j_cmds.position = std::vector<double>(_limited_joint_cmds.begin(),_limited_joint_cmds.end());
//
  _j_cmds.header.stamp       = time;
  _j_cmds.header.seq         = pub_seq;
  _j_cmds.position = std::vector<double>(_joint_cmds.begin(),_joint_cmds.end());
//
  _u.header.stamp       = time;
  _u.header.seq         = pub_seq;
  Eigen::VectorXd::Map(&_u.position[0], u.size()) = u;
//
  _qr.header.stamp      = time;
  _qr.header.seq        = pub_seq;
  Eigen::VectorXd::Map(&_qr.position[0], qr.size()) = qr;
  Eigen::VectorXd::Map(&_qr.velocity[0], dqr.size()) = dqr;
//
  _qe_integral.header.stamp    = time;
  _qe_integral.header.seq      = pub_seq;
  Eigen::VectorXd::Map(&_qe_integral.position[0], qe_integral.size()) = qe_integral;
  Eigen::VectorXd::Map(&_qe_integral.velocity[0], qe.size()) = qe;
//
  _f.header.stamp       = time;
  _f.header.seq         = pub_seq;
  Eigen::VectorXd::Map(&_f.position[0], f.size()) = f;
//
  _qe_q_integral.header.stamp       = time;
  _qe_q_integral.header.seq         = pub_seq;
  Eigen::VectorXd::Map(&_qe_q_integral.position[0] , qe_q_integral.size() ) = Eigen::VectorXd::Map(qe_q_integral.data() , qe_q_integral.size() );
  Eigen::VectorXd::Map(&_qe_q_integral.velocity[0], qe_dq_integral.size()) = Eigen::VectorXd::Map(qe_dq_integral.data(), qe_dq_integral.size());

  _qe_qr_integral.header.stamp       = time;
  _qe_qr_integral.header.seq         = pub_seq;
  Eigen::VectorXd::Map(&_qe_qr_integral.position[0], qe_qr_integral.size())  = Eigen::VectorXd::Map(qe_qr_integral.data() , qe_qr_integral.size() );
  Eigen::VectorXd::Map(&_qe_qr_integral.velocity[0], qe_dqr_integral.size()) = Eigen::VectorXd::Map(qe_dqr_integral.data(), qe_dqr_integral.size());

  _K.header.stamp       = time;
  _K.header.seq         = pub_seq;
  Eigen::VectorXd::Map(&_K.position[0], K0.size()) = Eigen::VectorXd::Map(K0.data(), K0.size());
  Eigen::VectorXd::Map(&_K.velocity[0], K1.size()) = Eigen::VectorXd::Map(K1.data(), K1.size());

  _Q.header.stamp       = time;
  _Q.header.seq         = pub_seq;
  Eigen::VectorXd::Map(&_Q.position[0], Q0.size()) = Eigen::VectorXd::Map(Q0.data(), Q0.size());
  Eigen::VectorXd::Map(&_Q.velocity[0], Q1.size()) = Eigen::VectorXd::Map(Q1.data(), Q1.size());
//
  pub_limited_j_cmds.publish(_limited_j_cmds);
  pub_j_cmds.publish(_j_cmds);
  pub_u.publish(_u);
  pub_qr.publish(_qr);
  pub_qe_integral.publish(_qe_integral);
  pub_f.publish(_f);

  pub_qe_q_integral.publish(_qe_q_integral);
  pub_qe_qr_integral.publish(_qe_qr_integral);
  pub_K.publish(_K);
  pub_Q.publish(_Q);




  jointPosPrev = jointPos;
  ++pub_seq;
}
void MRACController::stopping(const ros::Time &time){
  // can't send immediate commands for 0 velocity to robot
}

void MRACController::ref_callback(const std_msgs::Float32MultiArray::ConstPtr& ref){
  for (int i = 0; i < 7; i++){
    ref_p(i,0)  = ref->data[i];
    // ROS_INFO("%d: %s in callback",i,std::to_string(ref_p(i,0)).c_str());
  }
}

// Functions added from HIRO
// MRACController

void MRACController::targetCartesianPoseCb(const geometry_msgs::PoseWithCovarianceStamped &target_pose)
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


bool MRACController::_isGoalReached()
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

void MRACController::trapezoidVelCmd(const double &max_step, const double &v_change)
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
//   return MRACController::urdf_param;
// }




}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_aic::MRACController,
                       controller_interface::ControllerBase)
