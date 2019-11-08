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
    actionlib::SimpleActionServer<assembly_msgs::AssembleTripleRecoveryAction> as_;

    assembly_msgs::AssembleTripleRecoveryFeedback feedback_;
    assembly_msgs::AssembleTripleRecoveryResult result_;
    assembly_msgs::AssembleTripleRecoveryGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Matrix<double, 6, 1> f_measured_;
    Eigen::Matrix<double, 6, 1> desired_xd_;
        
    Eigen::Matrix<double, 3, 3> init_rot_;
    Eigen::Matrix<double, 3, 3> state_init_rot_1_;
    Eigen::Matrix<double, 3, 3> state_init_rot_2_;
    Eigen::Matrix<double, 3, 3> rot_1_;
    Eigen::Matrix<double, 3, 3> rot_2_;
    Eigen::Vector3d origin_;

    Eigen::Vector3d target_force_;

    Eigen::Vector3d target_;
    int dir_;
    int option_;

    double f_threshold_;
    double range_; //5*M_PI/180
    double range_1_;
    double range_2_;
    int mode_;
    int swing_dir_;

    bool is_first_;
    double duration_;
    int assemble_dir_;

    double state_;

    ros::Time task_end_time_2_;
    ros::Time task_end_time_3_;

public:
  AssembleTripleRecoveryActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot

  double getDistance(const Eigen::Vector3d target, const Eigen::Vector3d start, const int mode, const int dir);
};