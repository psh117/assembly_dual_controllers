#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleRetreatBoltAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleRetreatBoltActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::AssembleRetreatBoltAction> as_;
    assembly_msgs::AssembleRetreatBoltFeedback feedback_;
    assembly_msgs::AssembleRetreatBoltResult result_;
    assembly_msgs::AssembleRetreatBoltGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Isometry3d origin_;
    Eigen::Isometry3d current_;

    Eigen::Vector3d flange_to_assembly_point_;
    Eigen::Quaterniond flange_to_assembly_quat_;
   
    Eigen::Isometry3d T_7A_, T_WA_;
    
    bool wiggle_motion_;
    bool wiggle_motion_z_axis_;
    bool zero_moment_;
    
    double duration_;
    double retreat_force_;
    double retreat_distance_;
    double time_limit_;    
    double wiggle_angle_;
    double wiggle_angular_vel_;

    struct wiggle_z_axis{
      int a;
      double b;
      double t_offset;
    }wiggle_z_axis_param_;
    
    Eigen::Matrix3d compliant_translation_ee_select_mtx_;
    
    int mode_;

    std::ofstream reaction_force {"reaction_force.txt"};

public:
  AssembleRetreatBoltActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot

  private:
    void setSucceeded();
    void setAborted();
};