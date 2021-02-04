#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleTripleHoldAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleTripleHoldActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<assembly_msgs::AssembleTripleHoldAction> as_;

  assembly_msgs::AssembleTripleHoldFeedback feedback_;
  assembly_msgs::AssembleTripleHoldResult result_;
  assembly_msgs::AssembleTripleHoldGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

  Eigen::Matrix<double, 6, 1> target_force_left_;
  Eigen::Matrix<double, 6, 1> target_force_right_;
  Eigen::Matrix<double, 6, 1> target_force_top_;

  double duration_;

  std::ofstream wrench {"stefan_hold_wrench.txt"};  

public:
  AssembleTripleHoldActionServer(std::string name, ros::NodeHandle &nh,
                                 std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm,const std::string &arm_name);

protected:
  void setSucceeded() override;
  void setAborted() override;
};