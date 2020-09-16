#pragma once

#include <ros/ros.h>
#include <fstream>

#include <franka/lowpass_filter.h>
#include <assembly_dual_controllers/utils/timer/suhan_benchmark.h>
#ifdef USE_REAL_ROBOT_INTERFACE
#include <franka/robot_state.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <assembly_dual_controllers/robot_model/franka_panda_model.h>
#else
#include <assembly_mujoco_sim/utils/sim/mujoco_helper.h>
#endif

#include <Eigen/Dense>

struct FrankaModelUpdater
{
  // calibration parameters ---
  Eigen::Matrix<double, 7, 1> q_offset_;
  // arm parameters --
  Eigen::Matrix<double, 7, 7> mass_matrix_;
  Eigen::Matrix<double, 7, 7> modified_mass_matrix_;
  Eigen::Matrix<double, 6, 6> lambda_matrix_;
  Eigen::Matrix<double, 6, 6> modified_lambda_matrix_;
  Eigen::Matrix<double, 7, 1> coriolis_;
  Eigen::Matrix<double, 7, 1> q_;
  Eigen::Matrix<double, 7, 1> qd_;
  Eigen::Matrix<double, 7, 1> tau_measured_;
  Eigen::Matrix<double, 7, 1> tau_desired_read_;
  Eigen::Matrix<double, 7, 1> tau_ext_filtered_;
  Eigen::Matrix<double, 6, 1> f_ext_;
  Eigen::Matrix<double, 7, 1> tau_contact_;
  Eigen::Matrix<double, 7, 1> tau_contact_filtered_;
  Eigen::Matrix<double, 6, 1> f_measured_;
  // Eigen::Matrix<double, 6, 1> f_measured_filtered_;
  Eigen::Matrix<double, 6, 1> f_contact_;
  Eigen::Matrix<double, 7, 1> gravity_;
  Eigen::Matrix<double, 6, 7> jacobian_;
  Eigen::Matrix<double, 7, 6> jacobian_bar_; ///< dynamically consistent inverse
  Eigen::Matrix<double, 6, 6> manipulability_;
  double manipulability_measure_;
  Eigen::Matrix<double, 7, 7> null_projector_; ///< dynamically consistent inverse
  Eigen::Matrix<double, 3, 1> position_;
  Eigen::Matrix<double, 3, 3> rotation_;
  Eigen::Isometry3d transform_;
  Eigen::Matrix<double, 6, 1> xd_;
  Eigen::Matrix<double, 6, 1> xd_prev_;
  Eigen::Matrix<double, 7, 1> q_limit_center_;

  Eigen::Matrix<double, 7, 1> initial_q_;  ///< initial joint configuration for idle control
  Eigen::Isometry3d initial_transform_; ///< initial transform for idle control
  Eigen::Isometry3d t_7e_;
  bool idle_controlled_ {false}; ///< it indicates that this arm is under the idle control status. that is FrankaModelUpdater has valid initial transform and is using the transform.
  bool target_updated_ {false}; ///< it is used to check whether any other action is on going excep the idle control
  // -- arm parameters 

  SuhanBenchmark sb_;
  std::ofstream debug_file_td_{"time_debug_model.txt"};
  ros::Time task_start_time_; ///< time when a task starts
  ros::Time task_end_time_; ///< you may use this to indicate the timeout

  FrankaPandaModel rbdl_model_;
#ifdef USE_REAL_ROBOT_INTERFACE
  std::shared_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::shared_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;  ///< To command joint torques.
#else
  std::shared_ptr<MujocoHelper> mujoco_helper_;
  int arm_num_;
#endif

  double delta_tau_max_ {0.05};

  FrankaModelUpdater() ;
#ifdef USE_REAL_ROBOT_INTERFACE
	FrankaModelUpdater(std::shared_ptr<franka_hw::FrankaModelHandle> model_handle, 
                     std::shared_ptr<franka_hw::FrankaStateHandle> state_handle) ;
#else
	FrankaModelUpdater(std::shared_ptr<MujocoHelper> mujoco_helper, int arm_num);
#endif
  void initialize();
  void updateModel();
  void setTorque(const Eigen::Matrix<double,7,1> &torque_command, bool idle_control = false);
  void setInitialValues();
  void setInitialValues(const Eigen::Isometry3d & transform);
  void setInitialValues(const Eigen::Ref<const Eigen::VectorXd> &q);
  void printState();

  static const double PRINT_RATE;
  int print_count_ {1000};
  // const ros::Duration print_rate_;
  // ros::Time next_print_time_;

  std::array<double, 16> F_T_EE_, EE_T_K_;

  std::ofstream q_out_file_ ;
  std::ofstream x_out_file_ ;

  std::string arm_name_;
};