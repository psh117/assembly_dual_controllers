#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleDualArmApproachAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleDualApproachActionServer : public ActionServerBase
{
  enum ASSEMBLY_STATE : int
  {
    READY = 0,
    EXEC = 1,
    TILT_BACK = 2,
    IGNORE = 3
  };
  actionlib::SimpleActionServer<assembly_msgs::AssembleDualArmApproachAction> as_;

  assembly_msgs::AssembleDualArmApproachFeedback feedback_;
  assembly_msgs::AssembleDualArmApproachResult result_;
  assembly_msgs::AssembleDualArmApproachGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

  Eigen::Matrix<double, 6, 1> f_measured_;

  Eigen::Isometry3d task_arm_origin_;
  Eigen::Isometry3d task_arm_current_;
  Eigen::Isometry3d assist_arm_origin_;
  Eigen::Isometry3d assist_arm_current_;

  Eigen::Vector3d task_arm_to_assembly_point_, asssist_to_target_point_, task_arm_to_assist_arm_point_;
  Eigen::Quaterniond task_arm_to_assembly_quat_, asssist_to_target_quat_, task_arm_to_assist_arm_quat_;

  Eigen::Isometry3d T_7A_; //task arm EE to the assembly frame
  Eigen::Isometry3d T_7D_; // assist arm EE to the target point frame
  Eigen::Isometry3d T_task_A_; // the base of the task arm to the assembly frame
  Eigen::Isometry3d T_assist_D_; //the base of the assist arm to teh target point frame
  Eigen::Isometry3d T_AD_; //the assembly frame to the target point frame
  Eigen::Isometry3d T_task_assist_;
  Eigen::Vector3d tilt_axis_;

  Eigen::Vector3d approach_origin_;

  double descent_speed_;
  double contact_force_;
  double time_limit_;
  bool set_tilt_;
  bool set_tilt_back_;
  double tilt_angle_;
  double tilt_duration_;

  double tilt_start_time_;
  double approach_star_time_;

  ASSEMBLY_STATE state_;

  bool is_ready_first_;
  bool is_approach_first_;
  bool is_tilt_back_first_;

  std::ofstream force_moment;
  std::ofstream force_moment_lpf;

public:
  AssembleDualApproachActionServer(std::string name, ros::NodeHandle &nh,
                                 std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu);

  bool compute(ros::Time time) override;
  bool computeTaskArm(ros::Time time, FrankaModelUpdater &arm);
  bool computeAssistArm(ros::Time time, FrankaModelUpdater &arm);

private:
  void setSucceeded();
  void setAborted();
};