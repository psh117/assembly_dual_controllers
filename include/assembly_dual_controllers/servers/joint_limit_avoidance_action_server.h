#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/JointLimitAvoidanceAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <assembly_msgs/SetTrajectoryFollowerGain.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class JointLimitAvoidanceActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::JointLimitAvoidanceAction> as_;

    assembly_msgs::JointLimitAvoidanceFeedback feedback_;
    assembly_msgs::JointLimitAvoidanceResult result_;
    assembly_msgs::JointLimitAvoidanceGoalConstPtr goal_;

    Eigen::Vector7d q_init_, q_target_;
    double delta_q_;
    double duration_;
    
    void goalCallback() override;
    void preemptCallback() override;

public:
  JointLimitAvoidanceActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
  Eigen::Vector7d setJointTarget(const Eigen::Ref<const Eigen::Matrix<double, 7, 2>> &joint_limit_info,
                                 const Eigen::Ref<const Eigen::Vector7d> &q_init,
                                 const double joint_limit_margin);

protected:
  void setSucceeded() override;
  void setAborted() override;

};