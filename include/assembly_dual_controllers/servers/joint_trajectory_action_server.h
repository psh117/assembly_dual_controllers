#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace dyros_math;
class JointTrajectoryActionServer : public ActionServerBase
{
public:
  // Joint Trajectory Execution Server (Moveit Planning)
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory_;

  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;
  
  std::string active_arm_;

  JointTrajectoryActionServer(std::string name, ros::NodeHandle &nh, 
                          std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);
  // bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
};
