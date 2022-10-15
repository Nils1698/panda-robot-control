// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
 * File:   MRACcontroller.h
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on September 5th, 2019
 *
 * Header file for the model reference class. Definition of the variables for
 * the MRAC for the real Franka Emika Panda robot arm
 *
 */

#pragma once


#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h> // Probably unused? TODO: FW investigate if it's unused
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "panda_utils/panda_trac_ik.h"

#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka/gripper.h>

#include <memory>
#include <string>
#include <vector>
#include <iterator>
#include <Eigen/Core>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"

#include <sensor_msgs/JointState.h>

namespace franka_aic {

class MRACController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  // Mandatory from dicumentation
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time &time) override;
 private:

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // Define variables for the controller
  // Variables for sensory information and control input
  Eigen::Matrix<double, 7, 1> jointPos, jointVel, u;
  // Desired robot's states and error, column vector of 7 elements
  Eigen::Matrix<double, 7, 1> qr, dqr, qr_prev, qe, x_qe, qe_integral;
  // States to perform integrals of the error signal simulating a first order system as integrator
  Eigen::Matrix<double, 7, 7> X_qr, X_dqr, X_q, X_dq, qe_q_integral, qe_qr_integral, qe_dq_integral, qe_dqr_integral;
  // Support variable to control the flow of the script
  // MRAC Variables
  // Natural frequencies and damping ratio
  Eigen::Matrix<double, 7, 1> omega, zeta;
  // Adaptive gains and initial guesses
  Eigen::Matrix<double, 7, 7> K0, K1, Q0, Q1, K0_hat, K1_hat, Q0_hat, Q1_hat;
  // Controller parameters
  Eigen::Matrix<double, 7, 7> ALPHA1, ALPHA2, ALPHA3, E01, E02, E03, E11, E12, E13, F01, F02, F03, F11, F12, F13;
  // Support variables for adaptation law
  Eigen::Matrix<double, 7, 1> f, l1, l2, p2, p3, alpha1, alpha2, alpha3, e01, e02, e03, e11, e12, e13, f01, f02, f03, f11, f12, f13;
  Eigen::Matrix<double, 7, 7> P2, P3;
  // Integration step
  double h;
  // Via-points
  Eigen::Matrix<double, 7, 1>  qr_dRight, qr_dGrab, qr_dCenter, qr_dRelease;
  // Auxiliary variables
  double time_passed = 0.0;
  double interval_length;


  uint pub_seq;
  std::vector<std::string> jointNames = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7" };
  std::vector<std::string> jointNamesMatrix = { "panda_joint11", "panda_joint21", "panda_joint31", "panda_joint41", "panda_joint51", "panda_joint61", "panda_joint71",
                                                "panda_joint12", "panda_joint22", "panda_joint32", "panda_joint42", "panda_joint52", "panda_joint62", "panda_joint72",
                                                "panda_joint13", "panda_joint23", "panda_joint33", "panda_joint43", "panda_joint53", "panda_joint63", "panda_joint73",
                                                "panda_joint14", "panda_joint24", "panda_joint34", "panda_joint44", "panda_joint54", "panda_joint64", "panda_joint74",
                                                "panda_joint15", "panda_joint25", "panda_joint35", "panda_joint45", "panda_joint55", "panda_joint65", "panda_joint75",
                                                "panda_joint16", "panda_joint26", "panda_joint36", "panda_joint46", "panda_joint56", "panda_joint66", "panda_joint76",
                                                "panda_joint17", "panda_joint27", "panda_joint37", "panda_joint47", "panda_joint57", "panda_joint67", "panda_joint77"};
  sensor_msgs::JointState _limited_j_cmds   , _j_cmds, _u, _qr /*, _dqr*/   , _qe_integral/*,  _qe */, _f; // 7 elements long vectors.
  sensor_msgs::JointState _qe_q_integral /*, qe_dq_integral */, _qe_qr_integral /*, qe_dqr_integral */,  _K /*,contains K0 and K1*/ , _Q /*,contains Q0 and Q1*/ ; // I will make it a 49 element long vector to emulate the 7x7 matrces.
  ros::Publisher          pub_limited_j_cmds, pub_j_cmds, pub_u, pub_qr, pub_qe_integral, pub_f;
  ros::Publisher          pub_qe_q_integral, pub_qe_qr_integral, pub_K, pub_Q;
  // Added to compute Inverse kinematics and interpolated path between goal and start pose:
  // Subscribers for references
  ros::Subscriber refSub;
  Eigen::Matrix<double, 7, 1> ref_p, ref_v;
  void ref_callback(const std_msgs::Float32MultiArray::ConstPtr& ref);

  /// Parameters mostly for debugging:
  std::string urdf_param; // mostly for debugging
  std::stringstream buffer;
  int case_used_in_trapezoid_vel;


  tf2::Quaternion q_orig, q_rot, q_new; // Quaternion parameters for rotating apriltag detections to be reachable by robot

  // Added due to HIRO
  ros::Duration _elapsed_time;
  ros::Time _start_time;
  // target pose from user
  geometry_msgs::Pose _target_pose;

  ros::Subscriber _target_sub_task_space; // This is called _target_subscriber in the HIRO project

  std::array<double, 7> _joint_cmds;
  std::array<double, 7> _limited_joint_cmds;
  std::array<double, 7> _last_commanded_pos;

  //  CatmullRomVel parameters:
  double p_val;
  std::array<double,4> velocity_points_first_spline;
  std::array<double,4> velocity_points_second_spline;
  Eigen::Matrix<double, 7, 1>  jointPosPrev;

  // 7 by 4 matrix for coefficients for each franka panda joint
  std::array<std::array<double, 4>, 7> _vel_catmull_coeffs_first_spline;
  std::array<std::array<double, 4>, 7> _vel_catmull_coeffs_second_spline;
  std::array<double, 7> calc_max_pos_diffs;

  bool _is_executing_cmd;
//  double _max_abs_vel = 2.1;
  // threshold for when it is acceptable to say robot has reached goal
  double _epsilon = 0.1; // The error doesn't seem to get below 0.0019
  // user selected trajectory method after inverse kinematics solver completes
  enum class TrajectoryMethod
  {
      ConstantVel = 1,
      TrapezoidVel,
      CatmullRomVel
  };

  TrajectoryMethod traj_method = TrajectoryMethod::ConstantVel;
  // Parameters for Trapezoidal trajectory solver:
  double step_time;
  double change_dist;
  double max_velo = 3.0;
  double v_change = 0.3;
  double err;
  std::array<double, 7> _iters;
  // Parameters for CatmullRom trajectory solver: // 2022-06-20 not working
  std::array<double, 4> catmullRomSpline(const double &tau, const std::array<double,4> &points);

  franka_aic::PandaTracIK _panda_ik_service;

  KDL::JntArray _joints_result;
  KDL::JntArray ik_result;

  double calcSplinePolynomial(const std::array<double,4> &coeffs, const double &x);

  void targetCartesianPoseCb(const geometry_msgs::PoseWithCovarianceStamped &target_pose);

  void constantVelCmd(const double &min_step);

  void trapezoidVelCmd(const double &min_step, const double &v_change);

  void catmullRomSplineVelCmd(const double &norm_pos, const int &joint_num, const double &interval);

  bool _isGoalReached();


};

}  // namespace franka_aic
