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
    Eigen::Matrix<double, 6, 1> desired_xd_;
        
    Eigen::Vector3d init_pos_;
    Eigen::Matrix3d init_rot_;

    Eigen::Isometry3d origin_;
    Eigen::Isometry3d current_;

    Eigen::Vector3d ee_to_assembly_point_;
    Eigen::Quaterniond ee_to_assembly_quat_;

    Eigen::Isometry3d T_EA_, T_WA_;

    double f_threshold_;
    double range_; //5*M_PI/180

    bool is_first_;

    double pressing_force_;
    Eigen::Vector3d asm_dir_;
    double duration_;

    std::ofstream fm_rotation_search;
    std::ofstream pr_rotation_search;

public:
  AssembleRotationActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);

  private:
    void setSucceeded();
    void setAborted();
};