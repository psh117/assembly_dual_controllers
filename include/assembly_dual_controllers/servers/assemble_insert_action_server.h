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

    Eigen::Vector3d flange_to_assembly_point_;
    Eigen::Quaterniond flange_to_assembly_quat_;
   
    Eigen::Isometry3d T_7A_, T_WA_;
    
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

    int connector_type_;
    double insertion_depth_;
    int count_;
    int bolting_stop_count_;
    double bolting_minimum_depth_;
    double bolting_vel_threshold_;  

    std::ofstream save_insert_pose_data {"insert_pose_data.txt"};
    std::ofstream save_insertion_vel{"insertion_vel_data.txt"};
    std::ofstream save_rotation_error{"insertion_rotation_error.txt"};

    struct wiggle_z_axis{
      int a;
      double b;
      double t_offset;
    }wiggle_z_axis_param_;

    Eigen::Vector7d q_init_;
    Eigen::Vector7d q_null_target_;
    Eigen::Vector7d tau_null_cmd_;

    double total_action_start_time_;
    
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