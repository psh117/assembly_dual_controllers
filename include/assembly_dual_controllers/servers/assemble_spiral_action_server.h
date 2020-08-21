#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleSpiralAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleSpiralActionServer : public ActionServerBase
{
  enum ASSEMBLY_STATE : int
  {
    READY = 0,
    EXEC = 1
  };

  actionlib::SimpleActionServer<assembly_msgs::AssembleSpiralAction> as_;

  assembly_msgs::AssembleSpiralFeedback feedback_;
  assembly_msgs::AssembleSpiralResult result_;
  assembly_msgs::AssembleSpiralGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

  Eigen::Matrix<double, 6, 1> f_measured_;

  Eigen::Vector3d init_pos_;
  Eigen::Isometry3d origin_;
  Eigen::Isometry3d current_;

  Eigen::Vector3d force_compensation_;
  Eigen::Vector3d moment_compensation_;

  Eigen::Vector6d twist_pos_save_;
  Eigen::Vector6d spiral_pos_save_;
  // parameters from .action file ---
  double lin_vel_;
  double pitch_;
  int mode_;
  int assemble_dir_;
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

  bool is_first_;
  bool is_mode_changed_;
  ASSEMBLY_STATE state_;

  int count_;
  Eigen::Vector6d accumulated_wrench_;
  Eigen::Vector6d accumulated_wrench_a_;

  // !!! DO NOT USE RAW POINTER OF FILE !!!
  // FILE *spiral_data;

  std::ofstream spiral_search;
  std::ofstream twist_search;
  std::ofstream spiral_arm_position;
  std::ofstream force_spiral_cmd; 
    
public:
  AssembleSpiralActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);


private:
  void setSucceeded();
  void setAborted();
};