#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleExertForceAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleBoltActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::AssembleExertForceAction> as_;

    assembly_msgs::AssembleExertForceFeedback feedback_;
    assembly_msgs::AssembleExertForceResult result_;
    assembly_msgs::AssembleExertForceGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Matrix<double, 6, 1> f_measured_;
    Eigen::Matrix<double, 6, 1> desired_xd_;
    Eigen::Vector3d desired_x_;
    Eigen::Matrix<double, 7, 6> j_inverse_;
    
    Eigen::Matrix<double, 3, 3> init_rot_;
    Eigen::Vector3d origin_;

    Eigen::Vector3d target_force_;
    Eigen::Vector3d target_torque_;
    
    int assemble_dir_;

    double target_time_ {3.0};
    double task_p_gain_ {500.}; 
    double rot_p_gain_ {200.0};
    double rot_d_gain_ {5.0};

    bool rot_compliant_mode_;

    int count_;
    double prev_position_;

    // !!! DO NOT USE RAW POINTER OF FILE !!!
    // FILE *pos_rot;

    

public:
  AssembleBoltActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};