// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
 * File:   AICcontroller.h
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on September 5th, 2019
 *
 * Header file for the active inference class. Definition of the variables for
 * the AIC for the real Franka Emika Panda robot arm
 *
 */

#pragma once


#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
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
#include <std_msgs/UInt32.h>
//#include <kdl/chainiksolverpos_nr_jl.hpp>
// #include <trac_ik/trac_ik.hpp>


namespace franka_aic {

class AICController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  // Mandatory from documentation
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time &time) override;
  
  // TODO:
  // void ref_callback(const std_msgs::Float32MultiArray::ConstPtr& ref, Eigen::Matrix<double, 7, 1>* k);
  // std::string get_urdf_param();

  // Passing urdf_param (the urdf_parameter file to IK class)

 private:

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // Define variables for the controller
  // Variances for internal belief and sensory input, learning rates, integration step
  double var_q, var_qdot, var_mu, var_muprime, k_mu, k_a, h;
  // Auxiliari variables
  double time_passed = 0.0;
  double interval_length;
  // Precision matrices, diagonal matrices with the inverce of the variance
  Eigen::Matrix<double, 7, 7> SigmaP_yq0, SigmaP_yq1, SigmaP_mu, SigmaP_muprime;
  // Belief about the states and their derivatives mu, mu', mu'', joint states, desired value, control
  Eigen::Matrix<double, 7, 1> mu, mu_p, mu_pp, mu_dot, mu_dot_p, mu_dot_pp, jointPos, jointVel, mu_d, mu_d_dot, mu_d_prev, u;
  // Via-points
  Eigen::Matrix<double, 7, 1>  mu_dRight, mu_dGrab, mu_dCenter, mu_dRelease;
  // Definition of variables in order to publish the control actions
  std_msgs::Float64 a1, a2, a3, a4, a5, a6, a7;
  // Publishers for beliefs
  ros::Publisher aPub1, aPub2, aPub3, aPub4, aPub5, aPub6, aPub7;

  uint pub_seq;
  std::vector<std::string> jointNames = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7" };
  sensor_msgs::JointState _limited_j_cmds   , _j_cmds   , _mu   , _mu_dot   , _mu_d   , _u;
  ros::Publisher          pub_limited_j_cmds, pub_j_cmds, pub_mu, pub_mu_dot, pub_mu_d, pub_u;
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
//  double _max_abs_vel = 2.1; // Unused
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
  double v_change = 1.0;
  double err;
  std::array<double, 7> _iters;
  // Parameters for CatmullRom trajectory solver: // 2022-06-20 not working
  std::array<double, 4> catmullRomSpline(const double &tau, const std::array<double,4> &points);

  franka_aic::PandaTracIK _panda_ik_service;  // TODO This line when uncommented by itself causes a runtime error. 
        // maybe this is the error:
        //  Controller Spawner error while taking down controllers: unable to connect to service: [Errno 104] 
        // Connection reset by peer though it is declared only as a warning. 
        // I will now test if the other lines by themselves cause an error or if this line is the only problematic one.
        
        // Question/TODO: First I will check if the file with the PandaTracIK class is included only in header for 
        // panda_pose_controller from HIRO or if it is both in the .h and .cpp file.
        // Answer: panda_trac_ik.h is included in panda_pose_controller.h and panda_pose_controller.h is included in 
        // panda_pose_controller.cpp
        // The file is the PandaTracIK class in panda_utils and include/panda_utils with file name panda_trac_ik.h 
        // and panda_trac_ik.cpp 
        
        // Question/TODO: Tag funktionen i PandaTracIK og overfør den til hovedfunktionen? eller er den i en separat 
        // klasse da den måske interargerer med KDL på en måde der gør at den skal være i sit eget objekt?

        // Q: Jeg har en backup af panda_trac_ik.h/cpp så jeg kan modificere den som jeg vil. 
        // Måske kan jeg lave instansen af objektet og så bare udkommentere i panda_trac_ik.h/cpp ind til det virker igen
        // og så begynde at kommentere ting ind igen
        // A1: Jeg kan godt kalde objektet når alt koden i constructoren og .perform_ik er udkommenteret. 
        // Jeg vil nu begynde at kommentere ting ind for at se hvor problemet er.
        // Husk at bruge binary search.

        // 2022-06-15T14:38 I think I found the solution, the solution was to set the parameter "_urdf_param_string" in the constructor in the PandaTracIK class correctly.
        // When i sat it to "~/ckk_ws/src/tmp/panda_arm.urdf" the location of the urdf file for the panda_arm it didn't get a runtime error. 
        // I will now try to uncomment some of the lines I commented out earlier and see what happens. 


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
