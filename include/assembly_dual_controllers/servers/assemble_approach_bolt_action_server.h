#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleApproachBoltAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

#include <fstream>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleApproachBoltActionServer : public ActionServerBase
{
  enum ASSEMBLY_STATE : int
  {
    READY = 0,
    EXEC = 1
  };

  actionlib::SimpleActionServer<assembly_msgs::AssembleApproachBoltAction> as_;

  assembly_msgs::AssembleApproachBoltFeedback feedback_;
  assembly_msgs::AssembleApproachBoltResult result_;
  assembly_msgs::AssembleApproachBoltGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

  Eigen::Isometry3d origin_;
  Eigen::Isometry3d current_;

  Eigen::Vector3d flange_to_assembly_point_;
  Eigen::Quaterniond flange_to_assembly_quat_;

  Eigen::Isometry3d T_7A_, T_WA_;

  Eigen::Vector3d approach_origin_;

  double descent_speed_;
  double contact_force_;  
  double time_limit_;
  double init_yaw_angle_;
  double revolve_duration_;

  ASSEMBLY_STATE state_;

  bool is_mode_changed_;

  int count_;
  
  Eigen::Vector6d accumulated_wrench_;
  Eigen::Vector6d accumulated_wrench_a_;
  geometry_msgs::Wrench wrench_rtn_;

  std::ofstream force_moment {"fm_bolt_approach_data.txt"};

public:
  AssembleApproachBoltActionServer(std::string name, ros::NodeHandle &nh,
                               std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu);

  bool compute(ros::Time time) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  
};