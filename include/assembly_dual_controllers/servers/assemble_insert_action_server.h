#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleExertForceAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleInsertActionServer : public ActionServerBase
{
    enum CONTROL_TYPE : int
    {
      SIMPLE = 1,
      COMPLEX = 2
    };

    actionlib::SimpleActionServer<assembly_msgs::AssembleExertForceAction> as_;

    assembly_msgs::AssembleExertForceFeedback feedback_;
    assembly_msgs::AssembleExertForceResult result_;
    assembly_msgs::AssembleExertForceGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Isometry3d origin_;
    Eigen::Isometry3d current_;

    Eigen::Vector3d ee_to_assembly_point_;
    Eigen::Quaterniond ee_to_assembly_quat_;
   
    Eigen::Isometry3d T_EA_, T_WA_;
    
    bool wiggle_motion_;
    bool wiggle_motion_z_axis_;
    bool yawing_motion_;

    double duration_;
    double insertion_force_;
    double yawing_angle_;
    double init_yaw_angle_;
    double yaw_low_limit_ = -2.8973;
    double yaw_up_limit_ = 2.8973;

    double wiggle_angle_;
    double wiggle_angular_vel_;

    CONTROL_TYPE mode_;

    Eigen::Vector3d U_EA_;
    Eigen::Vector3d U_dir_; //w.r.t {E} 
    
public:
  AssembleInsertActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot

  private:
    void setSucceeded();
    void setAborted();
    void savePosture();
};