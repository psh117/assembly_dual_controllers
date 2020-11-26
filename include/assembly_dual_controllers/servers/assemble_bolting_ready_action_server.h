#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleBoltingReadyAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleBoltingReadyActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::AssembleBoltingReadyAction> as_;

    assembly_msgs::AssembleBoltingReadyFeedback feedback_;
    assembly_msgs::AssembleBoltingReadyResult result_;
    assembly_msgs::AssembleBoltingReadyGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Isometry3d origin_;
    Eigen::Isometry3d current_;

    Eigen::Vector3d target_pos_;
    Eigen::Quaterniond target_quat_;
   
    Eigen::Isometry3d T_target_;

    double duration_;

public:
  AssembleBoltingReadyActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot

protected:
  void setSucceeded() override;
  void setAborted() override;
};