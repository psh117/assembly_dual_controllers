#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleBackForthAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#define TEST_PRINT

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleBackForthActionServer : public ActionServerBase
{
  enum MOVE_STATE : int
  {
    KEEPCURRENT = 0,
    EXEC = 1,
    EXERTFORCE = 2
  };
  actionlib::SimpleActionServer<assembly_msgs::AssembleBackForthAction> as_;
  assembly_msgs::AssembleBackForthFeedback feedback_;
  assembly_msgs::AssembleBackForthResult result_;
  assembly_msgs::AssembleBackForthGoalConstPtr goal_;
  void goalCallback() override;
  void preemptCallback() override;

  int count_{0};
  double motion_start_time_;
  bool heavy_mass_;
  bool is_mode_changed_{true};
  bool control_running_{false};
  MOVE_STATE move_state_;
  Eigen::Isometry3d origin_;
  Eigen::Vector3d target_;
  Eigen::Vector6d accumulated_wrench_;
  std::string arm_name_;
  Eigen::Vector3d flange_to_assembly_point_;
  Eigen::Quaterniond flange_to_assembly_quat_;
  Eigen::Isometry3d T_7A_, T_WA_;
  int axis_;
  double linear_vel_;
  double pitch_;
  double duration_;
  double contact_force_;

public:
  AssembleBackForthActionServer(std::string name, ros::NodeHandle &nh,
                                 std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);

protected:
  void setSucceeded() override;
  void setAborted() override;
};