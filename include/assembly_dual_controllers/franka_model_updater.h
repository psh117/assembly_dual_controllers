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
  Eigen::Matrix<double, 7, 1> coriolis_;
  Eigen::Matrix<double, 7, 1> q_;
  Eigen::Matrix<double, 7, 1> qd_;
  Eigen::Matrix<double, 7, 1> tau_measured_;
  Eigen::Matrix<double, 7, 1> tau_desired_read_;
  Eigen::Matrix<double, 7, 1> gravity_;
  Eigen::Matrix<double, 6, 7> jacobian_;
  Eigen::Matrix<double, 3, 1> position_;
  Eigen::Matrix<double, 3, 3> rotation_;
  Eigen::Matrix<double, 6, 1> xd_;

  // -- arm parameters 

  ros::Time task_start_time_; ///< time when a task starts
  ros::Time task_end_time_; ///< you may use this to indicate the timeout

  FrankaModelUpdater() {}

	FrankaModelUpdater(
  std::shared_ptr<franka_hw::FrankaModelHandle> model_handle, 
  std::shared_ptr<franka_hw::FrankaStateHandle> state_handle) :
	model_handle_(model_handle), state_handle_(state_handle)
	{ updateModel(); }

  void updateModel()
  {
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    std::array<double, 49> massmatrix_array = model_handle_->getMass();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 7> gravity_array = model_handle_->getGravity();

    mass_matrix_ = Eigen::Map<const Eigen::Matrix<double, 7, 7>>(massmatrix_array.data());
    coriolis_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
    q_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
    qd_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

    jacobian_ = Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
    tau_measured_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J.data());
    tau_desired_read_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());
    gravity_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(gravity_array.data());
  
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    position_ = transform.translation();
    rotation_ = transform.rotation();
		
    xd_ = jacobian_ * qd_;
  }

  void setTorque(Eigen::Matrix<double,7,1> &torque_command)
  {
    for(int i=0; i<7; ++i)
    {
      joint_handles_[i].setCommand(torque_command(i));
    }
  }
public:
  std::shared_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::shared_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;  ///< To command joint torques.


  double delta_tau_max_ {0.05};
};