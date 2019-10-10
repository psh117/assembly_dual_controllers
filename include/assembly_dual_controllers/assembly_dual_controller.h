// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/transport_hints.h>
#include <Eigen/Dense>

#include <assembly_dual_controllers/franka_model_updater.h>
#include <assembly_dual_controllers/assemble_approach_action_server.h>
#include <assembly_dual_controllers/idle_control_server.h>
// #include <assembly_dual_controllers/single_peginhole_action_server.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseStamped.h>

#include <fstream>

#define EYE(X) Matrix<double, X, X>::Identity()
namespace assembly_dual_controllers {

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class AssemblyDualController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&time, const ros::Duration& period) override;

  // static Eigen::Vector2d Spiral(double time, double time_0,
	// double time_f, Eigen::Vector2d x_0, double line_v, double pitch );

 private:

  // std::unique_ptr<SinglePegInHoleActionServer> single_peginhole_action_server_;
  std::unique_ptr<AssembleApproachActionServer> assemble_approach_action_server_;
  std::unique_ptr<IdleControlServer> idle_control_server_;
  
  std::map<std::string, std::shared_ptr<FrankaModelUpdater> >  arms_data_; ///< Holds all relevant data for both arms.
  std::string left_arm_id_;   ///< Name of the left arm, retreived from the parameter server.
  std::string right_arm_id_;  ///< Name of the right arm, retreived from the parameter server.
  
  ///< Transformation between base frames of the robots.
  Eigen::Affine3d Ol_T_Or_;  // NOLINT (readability-identifier-naming)
  ///< Target transformation between the two endeffectors.
  Eigen::Affine3d EEr_T_EEl_;  // NOLINT (readability-identifier-naming)
  ///< Transformation from the centering frame to the left endeffector.
  Eigen::Affine3d EEl_T_C_;

  ros::Time start_time_;
  ros::Time task_start_time_;

  //ros::Duration ;

  std::ofstream debug_file_q{"q.txt"};
  std::ofstream debug_file_fuck{"f.txt"};
  std::ofstream debug_file_qd{"qd.txt"};
  std::ofstream debug_file_qdd{"qdd.txt"};
  std::ofstream debug_file_qc{"iamdebugqc.txt"};
  std::ofstream debug_file_tq{"iamdebugt.txt"};
  std::ofstream debug_file_3{"iamdebug.txt"};

  std::ofstream debug_file_td{"time_debug.txt"};

  /**
   * Saturates torque commands to ensure feasibility.
   *
   * @param[in] arm_data The data container of the arm.
   * @param[in] tau_d_calculated The raw command according to the control law.
   * @param[in] tau_J_d The current desired torque, read from the robot state.
   * @return The saturated torque commmand for the 7 joints of one arm.
   */
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const FrankaModelUpdater& arm_data,
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  /**
   * Initializes a single Panda robot arm.
   *
   * @param[in] robot_hw A pointer the RobotHW class for getting interfaces and resource handles.
   * @param[in] arm_id The name of the panda arm.
   * @param[in] joint_names The names of all joints of the panda.
   * @return True if successfull, false otherwise.
   */
  bool initArm(hardware_interface::RobotHW* robot_hw,
               const std::string& arm_id,
               const std::vector<std::string>& joint_names);

  /**
   * Computes the decoupled controller update for a single arm.
   *
   * @param[in] arm_data The data container of the arm to control.
   */
  void updateArm(FrankaModelUpdater& arm_data);
  /**
   * Prepares all internal states to be ready to run the real-time control for one arm.
   *
   * @param[in] arm_data The data container of the arm to prepare for the control loop.
   */
  void startingArm(FrankaModelUpdater& arm_data);
 
};

}  // namespace assembly_controllers