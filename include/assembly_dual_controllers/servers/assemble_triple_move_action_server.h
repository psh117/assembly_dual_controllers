#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleTripleMoveAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <geometry_msgs/Wrench.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleTripleMoveActionServer : public ActionServerBase
{
  enum MOVE_STATE : int
  {
    KEEPCURRENT = 0,
    EXEC = 1
  };

  actionlib::SimpleActionServer<assembly_msgs::AssembleTripleMoveAction> as_;

  assembly_msgs::AssembleTripleMoveFeedback feedback_;
  assembly_msgs::AssembleTripleMoveResult result_;
  assembly_msgs::AssembleTripleMoveGoalConstPtr goal_;
  
  void goalCallback() override;
  void preemptCallback() override;

  Eigen::Matrix<double, 6, 1> f_measured_;

  Eigen::Isometry3d left_arm_origin_, right_arm_origin_, top_arm_origin_;
  Eigen::Vector3d left_target, right_target, top_target;

  double duration_;
  double contact_force_;
  double motion_start_time_;

  bool first_{true};
  std::ofstream force_moment;
  int succeed_flag{0};
  int count_{0};
  
  MOVE_STATE left_state_, right_state_;

  Eigen::Vector6d accumulated_wrench_;
  geometry_msgs::Wrench wrench_rtn_;
  bool is_mode_changed_{false};

public:
  AssembleTripleMoveActionServer(std::string name, ros::NodeHandle &nh,
                                 std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm, Eigen::Isometry3d origin, Eigen::Vector3d target_pos, MOVE_STATE &state_);

private:
  void setSucceeded();
  void setAborted();
};