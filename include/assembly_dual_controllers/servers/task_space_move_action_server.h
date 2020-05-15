#pragma once

#include <eigen_conversions/eigen_msg.h>

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_msgs/TaskSpaceMoveAction.h>

using namespace dyros_math;
class TaskSpaceMoveActionServer : public ActionServerBase
{
public:
  // Joint Trajectory Execution Server (Moveit Planning)
  actionlib::SimpleActionServer<assembly_msgs::TaskSpaceMoveAction> as_;

  assembly_msgs::TaskSpaceMoveFeedback feedback_;
  assembly_msgs::TaskSpaceMoveResult result_;
  assembly_msgs::TaskSpaceMoveGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

  bool control_running {false}; // multi-threading error prevention
  
  std::string active_arm_;

  Eigen::VectorXd q_desired_;
  Eigen::VectorXd qd_desired_;
  Eigen::VectorXd qdd_desired_;

  Eigen::Isometry3d target_pose_;

  TaskSpaceMoveActionServer(std::string name, ros::NodeHandle &nh, 
                          std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);
  // bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
};
