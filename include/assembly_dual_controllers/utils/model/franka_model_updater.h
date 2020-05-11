#pragma once

#include <ros/ros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>

struct FrankaModelUpdater
{
  // arm parameters --
  Eigen::Matrix<double, 7, 7> mass_matrix_;
  Eigen::Matrix<double, 6, 6> lambda_matrix_;
  Eigen::Matrix<double, 7, 1> coriolis_;
  Eigen::Matrix<double, 7, 1> q_;
  Eigen::Matrix<double, 7, 1> qd_;
  Eigen::Matrix<double, 7, 1> tau_measured_;
  Eigen::Matrix<double, 7, 1> tau_desired_read_;
  Eigen::Matrix<double, 6, 1> f_measured_;
  Eigen::Matrix<double, 7, 1> gravity_;
  Eigen::Matrix<double, 6, 7> jacobian_;
  Eigen::Matrix<double, 7, 6> jacobian_bar_; ///< dynamically consistent inverse
  Eigen::Matrix<double, 7, 7> null_projector_; ///< dynamically consistent inverse
  Eigen::Matrix<double, 3, 1> position_;
  Eigen::Matrix<double, 3, 3> rotation_;
  Eigen::Isometry3d transform_;
  Eigen::Matrix<double, 6, 1> xd_;
  Eigen::Matrix<double, 7, 1> q_limit_center_;

  Eigen::Matrix<double, 7, 1> initial_q_;  ///< initial joint configuration for idle control
  Eigen::Isometry3d initial_transform_; ///< initial transform for idle control
  bool idle_controlled_ {false}; ///< it indicates that this arm is under the idle control status. that is FrankaModelUpdater has valid initial transform and is using the transform.
  bool target_updated_ {false}; ///< it is used to check whether any other action is on going excep the idle control
  // -- arm parameters 

  ros::Time task_start_time_; ///< time when a task starts
  ros::Time task_end_time_; ///< you may use this to indicate the timeout

  std::shared_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::shared_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;  ///< To command joint torques.

  double delta_tau_max_ {0.05};

  FrankaModelUpdater() ;
	FrankaModelUpdater(std::shared_ptr<franka_hw::FrankaModelHandle> model_handle, 
                     std::shared_ptr<franka_hw::FrankaStateHandle> state_handle) ;

  void initialize();
  void updateModel();
  void setTorque(const Eigen::Matrix<double,7,1> &torque_command, bool idle_control = false);
  void setInitialValues();
  void setInitialValues(const Eigen::Ref<const Eigen::VectorXd> &q);
  void printState();
  
  static const double PRINT_RATE;
  int print_count_ {0};
  // const ros::Duration print_rate_;
  // ros::Time next_print_time_;

  std::array<double, 16> F_T_EE_, EE_T_K_;
};