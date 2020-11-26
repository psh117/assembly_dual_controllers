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
      MOVE,
      APPROACH
    };

    actionlib::SimpleActionServer<assembly_msgs::AssembleTripleRecoveryAction> as_;

    assembly_msgs::AssembleTripleRecoveryFeedback feedback_;
    assembly_msgs::AssembleTripleRecoveryResult result_;
    assembly_msgs::AssembleTripleRecoveryGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Isometry3d origin_, current_, target_;
    Eigen::Isometry3d T_7A_, T_7P_, T_WA_;

    Eigen::Vector3d flange_to_assembly_point_;
    Eigen::Quaterniond flange_to_assembly_quat_;
    
    Eigen::Vector3d target_pose_position_;
    Eigen::Quaterniond target_pose_quat_;
    Eigen::Vector6d accumulated_wrench_;

    double tilt_back_threshold_;
    double recovery_angle_;
    double duration_;
    
    ASSEMBLY_STATE state_;

    bool is_escape_first_;
    bool is_move_first_;
    bool is_approach_first_;

    Eigen::Vector3d tilt_axis_;
    int count_;

    std::ofstream triple_recovery_fm_data {"triple_recovery_fm_data.txt"};
    std::ofstream triple_recovery_pr_data {"triple_recovery_pr_data.txt"};
public:
  AssembleTripleRecoveryActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot

  double getDistance(const Eigen::Vector3d target, const Eigen::Vector3d start, const int mode, const int dir);

protected:
  void setSucceeded() override;
  void setAborted() override;

};