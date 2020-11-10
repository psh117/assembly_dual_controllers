#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleKittingAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

#include <fstream>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleKittingActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<assembly_msgs::AssembleKittingAction> as_;

  assembly_msgs::AssembleKittingFeedback feedback_;
  assembly_msgs::AssembleKittingResult result_;
  assembly_msgs::AssembleKittingGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

  Eigen::Isometry3d origin_;
  Eigen::Isometry3d current_;

  double speed_;
  double contact_force_;
  int dir_;
  int count_;
  Eigen::Vector3d kitting_dir_;
  Eigen::Vector6d accumulated_wrench_;
  std::ofstream kitting_fm_data {"kitting_fm_data.txt"};

public:
  AssembleKittingActionServer(std::string name, ros::NodeHandle &nh,
                               std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu);

  bool compute(ros::Time time) override;

private:
  void setSucceeded();
  void setAborted();
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};