#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleSpiralAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleSpiralActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::AssembleSpiralAction> as_;

    assembly_msgs::AssembleSpiralFeedback feedback_;
    assembly_msgs::AssembleSpiralResult result_;
    assembly_msgs::AssembleSpiralGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Matrix<double, 6, 1> f_measured_;
    Eigen::Matrix<double, 6, 1> desired_xd_;
    Eigen::Vector3d desired_x_;
    Eigen::Matrix<double, 7, 6> j_inverse_;
    
    Eigen::Matrix<double, 3, 3> init_rot_;
    Eigen::Vector3d origin_;

    // parameters from .action file ---
    double lin_vel_;
    double pitch_;
    int mode_;
    int assemble_dir_;
    //--- parameters from .action file

    int ori_change_dir_;
    bool is_first_;
    double ori_start_time_;
    double ori_duration_;
    

public:
  AssembleSpiralActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  void motionForSingle(ros::Time time, FrankaModelUpdater &arm, Eigen::Vector3d m_star, Eigen::Matrix3d rot, Eigen::Matrix<double, 6, 1> xd);
  void motionForDual(ros::Time time, Eigen::Vector3d m_star, Eigen::Matrix3d rot, Eigen::Vector3d angular);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};