#include <assembly_dual_controllers/assembly_triple_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace assembly_dual_controllers {

bool AssemblyTripleController::initArm(
    hardware_interface::RobotHW* robot_hw,
    const std::string& arm_id,
    const std::vector<std::string>& joint_names) {
  std::shared_ptr<FrankaModelUpdater> arm_data = std::make_shared<FrankaModelUpdater>();
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Error getting model interface from hardware");
    return false;
  }
  try {
    arm_data->model_handle_ = std::make_shared<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data->state_handle_ = std::make_shared<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      arm_data->joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "AssemblyTripleController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }
  arm_data->arm_name_ = arm_id;
  arms_data_.emplace(std::make_pair(arm_id, arm_data));

  return true;
}

bool AssemblyTripleController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {

  if (!node_handle.getParam("left/arm_id", left_arm_id_)) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Could not read parameter left_arm_id_");
    return false;
  }
  std::vector<std::string> left_joint_names;
  if (!node_handle.getParam("left/joint_names", left_joint_names) || left_joint_names.size() != 7) {
    ROS_ERROR(
        "AssemblyTripleController: Invalid or no left_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("right/arm_id", right_arm_id_)) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Could not read parameter right_arm_id_");
    return false;
  }

  std::vector<std::string> right_joint_names;
  if (!node_handle.getParam("right/joint_names", right_joint_names) ||
      right_joint_names.size() != 7) {
    ROS_ERROR(
        "AssemblyTripleController: Invalid or no right_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("top/arm_id", top_arm_id_)) {
    ROS_ERROR_STREAM(
        "AssemblyTripleController: Could not read parameter top_arm_id_");
    return false;
  }

  std::vector<std::string> top_joint_names;
  if (!node_handle.getParam("top/joint_names", top_joint_names) ||
      top_joint_names.size() != 7) {
    ROS_ERROR(
        "AssemblyTripleController: Invalid or no top_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  // ROS_INFO("1: %s \n 2: %s",left_arm_id_.c_str(), right_arm_id_.c_str());
  
  bool left_success = initArm(robot_hw, left_arm_id_, left_joint_names);
  bool right_success = initArm(robot_hw, right_arm_id_, right_joint_names);
  bool top_success = initArm(robot_hw, top_arm_id_, top_joint_names);

  // Get the transformation from right_O_frame to left_O_frame
  tf::StampedTransform transform;
  tf::TransformListener listener;
  try {
    if (listener.waitForTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0", 
                                  ros::Time(0), ros::Duration(4.0))) {
      listener.lookupTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0",
                                  ros::Time(0), transform);
    } else {
      ROS_ERROR(
          "AssemblyTripleController: Failed to read transform from %s to %s. "
          "Aborting init!",
          (right_arm_id_ + "_link0").c_str(), (left_arm_id_ + "_link0").c_str());
      return false;
    }
  } catch (tf::TransformException& ex) {
    ROS_ERROR("AssemblyTripleController: %s", ex.what());
    return false;
  }
  tf::transformTFToEigen(transform, Ol_T_Or_);  // NOLINT (readability-identifier-naming)


  idle_control_server_ = std::make_unique<IdleControlServer>
  ("/assembly_dual_controller/idle_control", node_handle, arms_data_);

  assemble_approach_action_server_ = std::make_unique<AssembleApproachActionServer>
  ("/assembly_dual_controller/assemble_approach_control", node_handle, arms_data_);
  assemble_spiral_action_server_ = std::make_unique<AssembleSpiralActionServer>
  ("/assembly_dual_controller/assemble_spiral_control", node_handle, arms_data_);
  assemble_insert_action_server_ = std::make_unique<AssembleInsertActionServer>
  ("/assembly_dual_controller/assemble_exert_force_control", node_handle, arms_data_);
  assemble_verify_action_server_ = std::make_unique<AssembleVerifyActionServer>
  ("/assembly_dual_controller/assemble_verify_completion_control", node_handle, arms_data_);
  joint_trajectory_action_server_ = std::make_unique<JointTrajectoryActionServer>
  ("/assembly_dual_controller/joint_trajectory_control", node_handle, arms_data_);
  assemble_parallel_action_server_ = std::make_unique<AssembleParallelActionServer>
  ("/assembly_dual_controller/assemble_parallel_control", node_handle, arms_data_);
  assemble_move_action_server_ = std::make_unique<AssembleMoveActionServer>
  ("/assembly_dual_controller/assemble_move_control", node_handle, arms_data_);
  assemble_press_action_server_ = std::make_unique<AssemblePressActionServer>
  ("/assembly_dual_controller/assemble_press_control", node_handle, arms_data_);
  assemble_side_chair_action_server_ = std::make_unique<AssembleSideChairActionServer>
  ("/assembly_dual_controller/assemble_side_chair_control", node_handle, arms_data_);
  assemble_dual_spiral_action_server_ = std::make_unique<AssembleDualSpiralActionServer>
  ("/assembly_dual_controller/assemble_dual_spiral_control", node_handle, arms_data_);
  assemble_dual_approach_action_server_ = std::make_unique<AssembleDualApproachActionServer>
  ("/assembly_dual_controller/assemble_dual_approach_control", node_handle, arms_data_);
  assemble_rotation_action_server_ = std::make_unique<AssembleRotationActionServer>
  ("/assembly_dual_controller/assemble_rotation_control", node_handle, arms_data_);
  assemble_triple_recovery_action_server_ = std::make_unique<AssembleTripleRecoveryActionServer>
  ("/assembly_dual_controller/assemble_triple_recovery_control", node_handle, arms_data_);
  assemble_bolt_action_server_ = std::make_unique<AssembleBoltActionServer>
  ("/assembly_dual_controller/assemble_bolt_control", node_handle, arms_data_);
  task_space_move_action_server_ = std::make_unique<TaskSpaceMoveActionServer>
  ("/assembly_dual_controller/task_space_move", node_handle, arms_data_);
  assemble_probe_edge_action_server_ = std::make_unique<AssembleProbeEdgeActionServer>
  ("/assembly_dual_controller/assemble_probe_edge_control", node_handle, arms_data_);
  // single_peginhole_action_server_ = std::make_unique<SinglePegInHoleActionServer>
  // ("/assembly_dual_controller/single_peg_in_hole_control", node_handle, dual_arm_info_);

  return left_success && right_success && top_success;
}


void AssemblyTripleController::startingArm(FrankaModelUpdater& arm_data) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Isometry3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
}

void AssemblyTripleController::starting(const ros::Time& time) {
  for (auto& arm_data : arms_data_) {
    startingArm(*arm_data.second);
  }
  // franka::RobotState robot_state_right =
  //     arms_data_.at(right_arm_id_).state_handle_->getRobotState();
  // franka::RobotState robot_state_left = arms_data_.at(left_arm_id_).state_handle_->getRobotState();
  // Eigen::Isometry3d Ol_T_EEl(Eigen::Matrix4d::Map(  // NOLINT (readability-identifier-naming)
  //     robot_state_left.O_T_EE.data()));           // NOLINT (readability-identifier-naming)
  // Eigen::Isometry3d Or_T_EEr(Eigen::Matrix4d::Map(  // NOLINT (readability-identifier-naming)
  //     robot_state_right.O_T_EE.data()));          // NOLINT (readability-identifier-naming)
  // EEr_T_EEl_ =
  //     Or_T_EEr.inverse() * Ol_T_Or_.inverse() * Ol_T_EEl;  // NOLINT (readability-identifier-naming)
  // EEl_T_C_.setIdentity();
  // Eigen::Vector3d EEr_r_EEr_EEl =  // NOLINT (readability-identifier-naming)
  //     EEr_T_EEl_.translation();    // NOLINT (readability-identifier-naming)
  // EEl_T_C_.translation() = -0.5 * EEr_T_EEl_.inverse().rotation() * EEr_r_EEr_EEl;


  // Bias correction for the current external torque
  start_time_ = ros::Time::now();
}

void AssemblyTripleController::update(const ros::Time& time, const ros::Duration& period) {

  // std::cout << period.toSec() << std::endl;
  for (auto& arm : arms_data_) {
    arm.second->updateModel();
    arm.second->target_updated_ = false;
  }


  joint_trajectory_action_server_->compute(time);
  assemble_approach_action_server_->compute(time);
  assemble_spiral_action_server_->compute(time);
  assemble_insert_action_server_->compute(time);
  assemble_verify_action_server_->compute(time);
  assemble_parallel_action_server_->compute(time);
  assemble_move_action_server_->compute(time);
  assemble_press_action_server_->compute(time);
  assemble_side_chair_action_server_ ->compute(time);
  assemble_rotation_action_server_->compute(time);
  assemble_triple_recovery_action_server_->compute(time);
  assemble_dual_spiral_action_server_->compute(time);
  assemble_dual_approach_action_server_->compute(time);
  assemble_bolt_action_server_->compute(time);
  task_space_move_action_server_->compute(time);
  idle_control_server_->compute(time);
  assemble_probe_edge_action_server_->compute(time);
  

  // Eigen::Matrix<double, 7, 1> tau_cmd;  
  // tau_cmd.setZero();
  // for (size_t i = 0; i < 7; ++i) {
  //     arms_data_[right_arm_id_].joint_handles_[i].setCommand(tau_cmd(i));
  //   }
  // for (size_t i = 0; i < 7; ++i) {
  //   arms_data_[left_arm_id_].joint_handles_[i].setCommand(tau_cmd(i));
  // }
///////////////////////////////////////////////
}

Eigen::Matrix<double, 7, 1> AssemblyTripleController::saturateTorqueRate(
    const FrankaModelUpdater& arm_data,
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, arm_data.delta_tau_max_),
                                               -arm_data.delta_tau_max_);
  }
  return tau_d_saturated;
}

/*
void AssemblyTripleController::updateArm(FrankaModelUpdater& arm_data) {
  // get state variables
  arm_data.updateModel();

  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);
  tau_d.setZero();
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(arm_data, tau_d, arm_data.tau_desired_read_);
  for (size_t i = 0; i < 7; ++i) {
    arm_data.joint_handles_[i].setCommand(tau_d(i));
  }
      //arm_data.orientation_d_.slerp(arm_data.filter_params_, arm_data.orientation_d_target_);
}
*/


}  // namespace assembly_dual_controllers

PLUGINLIB_EXPORT_CLASS(assembly_dual_controllers::AssemblyTripleController,
                       controller_interface::ControllerBase)
