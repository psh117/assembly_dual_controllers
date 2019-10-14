#pragma once

#include <assembly_controllers/action_server_base.h>
#include <assembly_controllers/dyros_math.h>
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

  JointTrajectoryActionServer(std::string name, ros::NodeHandle &nh, 
                                std::shared_ptr<FrankaModelUpdater> &mu);

  bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};
