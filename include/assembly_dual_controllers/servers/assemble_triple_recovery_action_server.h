#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleTripleRecoveryAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleTripleRecoveryActionServer : public ActionServerBase
{
    enum ASSEMBLY_STATE
    {
      ESCAPE,
      MOVE,
      APPROACH
    };

    actionlib::SimpleActionServer<assembly_msgs::AssembleTripleRecoveryAction> as_;

    assembly_msgs::AssembleTripleRecoveryFeedback feedback_;
    assembly_msgs::AssembleTripleRecoveryResult result_;
    assembly_msgs::AssembleTripleRecoveryGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Matrix<double, 6, 1> f_measured_;
    Eigen::Matrix<double, 6, 1> desired_xd_;

    Eigen::Vector3d init_pos_;    
    Eigen::Matrix3d init_rot_;

    Eigen::Isometry3d origin_;
    Eigen::Isometry3d current_;
    Eigen::Isometry3d target_;
    Eigen::Isometry3d T_EA_, T_WA_;

    Eigen::Vector3d ee_to_assembly_point_;
    Eigen::Quaterniond ee_to_assembly_quat_;

    Eigen::Vector3d target_pose_position_;
    Eigen::Quaterniond target_pose_quat_;

    double recover_angle_; //5*M_PI/180
    double duration_;
    
    
    ASSEMBLY_STATE state_;

    bool is_escape_first_;
    bool is_move_first_;
    bool is_approach_first_;

    Eigen::Vector3d tilt_axis_;


public:
  AssembleTripleRecoveryActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot

  double getDistance(const Eigen::Vector3d target, const Eigen::Vector3d start, const int mode, const int dir);

private:
  void setSucceeded();
  void setAborted();

};