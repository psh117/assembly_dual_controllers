#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleApproachAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

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
    
    Eigen::Vector3d origin_;
    Eigen::Matrix<double, 3, 3> init_rot_;

    double descent_speed_; 
    double contact_force_;    
    int assemble_dir_;
    int mode_;

public:
  AssembleApproachActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};