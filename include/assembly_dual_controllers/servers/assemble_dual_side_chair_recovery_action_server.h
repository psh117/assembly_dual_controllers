#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleDualArmSideChairRecoveryAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

enum ASSIST_ARM_STATE : int
{
  HOLD = 0,
  LIFT_UP = 1,
  LIFT_DOWN = 2,
  FINISH = 3
};

enum TASK_ARM_STATE : int
{
  READY = 0,
  MOVE = 1,
  FAIL = 2,
  SUCCESS = 3
};

class AssembleDualSideChairRecoveryActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::AssembleDualArmSideChairRecoveryAction> as_;

    assembly_msgs::AssembleDualArmSideChairRecoveryFeedback feedback_;
    assembly_msgs::AssembleDualArmSideChairRecoveryResult result_;
    assembly_msgs::AssembleDualArmSideChairRecoveryGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Isometry3d task_arm_origin_;
    Eigen::Isometry3d task_arm_current_;
    Eigen::Isometry3d assist_arm_origin_;
    Eigen::Isometry3d assist_arm_current_;
    Eigen::Isometry3d T_7A_, T_WA_; //for Task Arm
    Eigen::Isometry3d T_7p_, T_wp_; //for Assist Arm
    Eigen::Vector3d tilt_axis_;
    
    TASK_ARM_STATE task_state_;
    ASSIST_ARM_STATE assist_state_;

    bool is_task_mode_changed_;
    bool is_assist_mode_changed_;

    // parameters from .action file ---
    Eigen::Vector3d flange_to_assembly_point_;
    Eigen::Quaterniond flange_to_assembly_quat_; // task arm to assembly point
    Eigen::Vector3d flange_to_pivot_point_;
    Eigen::Quaterniond flange_to_pivot_quat_; // assist arm to pivot point

    double tilt_angle_;
    double tilt_duration_;
    //--- parameters from .action file

    // std::ofstream save_sprial_data;
    
public:
  AssembleDualSideChairRecoveryActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeTaskArm(ros::Time time, FrankaModelUpdater &arm);
  bool computeAssistArm(ros::Time time, FrankaModelUpdater &arm);
  void stateTransferFromTaskArm();
  void stateTransferFromAssisArm();
private:
  void setSucceeded();
  void setAborted();

};