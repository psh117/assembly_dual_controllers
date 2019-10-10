#pragma once

#include <assembly_controllers/action_server_base.h>
#include <assembly_msgs/AssemblePegInHoleAction.h>
#include <assembly_controllers/dyros_math.h>

using namespace dyros_math;

class AssemblePegInHoleActionServer : public ActionServerBase
{
  // Joint Trajectory Execution Server (Moveit Planning)
  actionlib::SimpleActionServer<assembly_msgs::AssemblePegInHoleAction> as_;
  
  assembly_msgs::AssemblePegInHoleFeedback feedback_;
  assembly_msgs::AssemblePegInHoleResult result_;
  assembly_msgs::AssemblePegInHoleGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;


  // variable
  //Parameters for assembleUpdate
  Eigen::Vector3d x_desired_assembly_; //desired postion of spiral motion
  Eigen::Vector3d spiral_starting_pos_assembly_;
  Eigen::Matrix<double, 3, 3> ori_init_assembly_;
  double time_starting_assembly_;
  double check_stop_assemlby_;

public:
  AssemblePegInHoleActionServer(std::string name, ros::NodeHandle &nh, 
                                std::shared_ptr<FrankaDataContainer> &mu);

  bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};
