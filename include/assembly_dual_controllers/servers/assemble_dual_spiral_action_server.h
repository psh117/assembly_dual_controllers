#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleDualArmSpiralAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleDualSpiralActionServer : public ActionServerBase
{
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

    Eigen::Vector3d flange_to_assembly_point_task_;
    Eigen::Vector3d flange_to_assembly_point_assist_;
    Eigen::Quaterniond flange_to_assembly_quat_task_;   
    Eigen::Quaterniond flange_to_assembly_quat_assist_;

    Eigen::Isometry3d T_7A_task_, T_WA_task_;
    Eigen::Isometry3d T_7A_assist_, T_WA_assist_;
    //--- parameters from .action file

    std::ofstream save_sprial_data {"save_sprial_data.txt"};
    
public:
  AssembleDualSpiralActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeTaskArm(ros::Time time, FrankaModelUpdater &arm);
  bool computeAssistArm(ros::Time time, FrankaModelUpdater &arm);

protected:
  void setSucceeded() override;
  void setAborted() override;

};