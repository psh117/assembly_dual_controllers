#pragma once

#include <assembly_dual_controllers/action_server_base.h>
#include <assembly_msgs/AssembleApproachAction.h>
#include <assembly_dual_controllers/dyros_math.h>

using namespace dyros_math;

class AssembleApproachActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::AssembleApproachAction> as_;

    assembly_msgs::AssembleApproachFeedback feedback_;
    assembly_msgs::AssembleApproachResult result_;
    assembly_msgs::AssembleApproachGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Matrix<double, 6, 1> f_measured_;
    Eigen::Matrix<double, 6, 1> desired_xd_;
    Eigen::Vector3d desired_x_;
    Eigen::Matrix<double, 7, 6> j_inverse_;
    Eigen::Matrix<double, 3, 3> ori_init_assembly_;
    double descent_speed_ {-0.02};

public:
  AssembleApproachActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};