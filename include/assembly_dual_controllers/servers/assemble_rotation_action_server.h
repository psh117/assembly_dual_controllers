#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleRotationAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleRotationActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::AssembleRotationAction> as_;

    assembly_msgs::AssembleRotationFeedback feedback_;
    assembly_msgs::AssembleRotationResult result_;
    assembly_msgs::AssembleRotationGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Matrix<double, 6, 1> f_measured_;
    
    Eigen::Isometry3d origin_;
    Eigen::Isometry3d current_;

    Eigen::Vector3d flange_to_assembly_point_;
    Eigen::Quaterniond flange_to_assembly_quat_;

    Eigen::Vector3d flange_to_pivot_point_;
    Eigen::Quaterniond flange_to_pivot_quat_;

    Eigen::Isometry3d T_7A_, T_WA_, T_7P_, T_WP_;

    double f_threshold_;
    double range_; //5*M_PI/180

    bool is_first_;

    double pressing_force_;
    Eigen::Vector3d asm_dir_;
    double duration_;

    int count_;
    Eigen::Vector6d accumulated_wrench_, accumulated_wrench_a_;

    std::ofstream fm_rotation_search {"fm_rotation_search.txt"};
    std::ofstream pr_rotation_search {"pr_rotation_search.txt"};

public:
  AssembleRotationActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);

  private:
    void setSucceeded();
    void setAborted();
};