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

    Eigen::Matrix<double, 6, 1> f_measured_;

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

    Eigen::Vector3d ee_to_assembly_point_;
    Eigen::Quaterniond ee_to_assembly_quat_;
   
    Eigen::Isometry3d T_EA_, T_WA_;
    //--- parameters from .action file

    std::ofstream save_sprial_data;
    
public:
  AssembleDualSpiralActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeTaskArm(ros::Time time, FrankaModelUpdater &arm);
  bool computeAssistArm(ros::Time time, FrankaModelUpdater &arm);

private:
  void setSucceeded();
  void setAborted();

};