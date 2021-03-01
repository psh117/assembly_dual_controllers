#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleDualArmSpiralAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <assembly_dual_controllers/utils/control/EM_GMM_estimator.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;
using namespace Estimator;

class AssembleGMMSpiralActionServer : public ActionServerBase
{ 
  enum ASSEMBLY_STATE : int
  {
    READY = 0,
    EXEC = 1,
    RETURN = 2
  };
    actionlib::SimpleActionServer<assembly_msgs::AssembleDualArmSpiralAction> as_;

    assembly_msgs::AssembleDualArmSpiralFeedback feedback_;
    assembly_msgs::AssembleDualArmSpiralResult result_;
    assembly_msgs::AssembleDualArmSpiralGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Isometry3d task_arm_origin_;
    Eigen::Isometry3d task_arm_current_;
    Eigen::Isometry3d assist_arm_origin_;
    Eigen::Isometry3d assist_arm_current_;

    // parameters from .action file ---
    double lin_vel_;
    double pitch_;
    int mode_;
    double depth_;
    double friction_;
    double pressing_force_;
    double spiral_duration_;
    double range_;
    double twist_duration_;
    int assist_arm_action_;

    bool is_mode_changed_;
    ASSEMBLY_STATE state_;
    int count_;
    double time_delay_;
    Eigen::Vector6d accumulated_wrench_;
    Eigen::Vector6d accumulated_wrench_a_;

    Eigen::Vector3d flange_to_assembly_point_task_;
    Eigen::Vector3d flange_to_assembly_point_assist_;
    Eigen::Quaterniond flange_to_assembly_quat_task_;   
    Eigen::Quaterniond flange_to_assembly_quat_assist_;

    Eigen::Isometry3d T_7A_task_, T_WA_task_;
    Eigen::Isometry3d T_7A_assist_, T_WA_assist_;
    //--- parameters from .action file

    Eigen::VectorXd torque_captured_;
    Eigen::VectorXd position_captured_;
    Eigen::VectorXd torque_init_;
    Eigen::VectorXd position_init_;

    double spiral_elapsed_time_;

    Estimator::GMM_model torque_model_small_;
    Estimator::GMM_model torque_model_large_;
    Estimator::GMM_model position_model_small_;
    Estimator::GMM_model position_model_medium_;
    Estimator::GMM_model position_model_large_;


    std::ofstream save_task_arm_torque {"gmm_task_arm_torque.txt"};
    std::ofstream save_task_arm_pose {"gmm_task_arm_pose.txt"};
    std::ofstream save_task_arm_force {"gmm_task_arm_force.txt"};
    std::ofstream save_task_arm_joint {"gmm_task_arm_joint.txt"};
    std::ofstream save_task_arm_command_local {"gmm_task_arm_command_force.txt"};
    std::ofstream save_task_arm_command_global {"gmm_task_arm_global_command_force.txt"};

    std::ofstream save_assist_arm_torque {"gmm_assist_arm_torque.txt"};
    std::ofstream save_assist_arm_pose {"gmm_assist_arm_pose.txt"};
    std::ofstream save_assist_arm_force {"gmm_assist_arm_force.txt"};
    std::ofstream save_assist_arm_joint {"gmm_assist_arm_joint.txt"};

    std::ofstream save_contact_estimation{"gmm_contact_state_estimation.txt"};
    std::ofstream save_captured_torque{"gmm_captured_torque.txt"};
    std::ofstream save_captured_position{"gmm_captured_position.txt"};

public:
  AssembleGMMSpiralActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeTaskArm(ros::Time time, FrankaModelUpdater &arm);
  bool computeAssistArm(ros::Time time, FrankaModelUpdater &arm);
  bool estimateContactState();

protected:
  void setSucceeded() override;
  void setAborted() override;

};