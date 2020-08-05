#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleVerifyCompletionAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleVerifyActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::AssembleVerifyCompletionAction> as_;

    assembly_msgs::AssembleVerifyCompletionFeedback feedback_;
    assembly_msgs::AssembleVerifyCompletionResult result_;
    assembly_msgs::AssembleVerifyCompletionGoalConstPtr goal_;

    enum STATE
    {
      READY,
      FORWARD,
      BACKWARD
    };

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Isometry3d origin_, current_;
    Eigen::Isometry3d T_EA_, T_WA_;
    
    Eigen::Vector3d ee_to_assembly_point_;
    Eigen::Quaterniond ee_to_assembly_quat_;
    Eigen::Vector3d spiral_origin_;
    
    bool is_mode_changed_;
    int search_index_;
    int cnt_;
    
    Eigen::Vector3d target_;    
    Eigen::Vector2d search_dir_;
    Eigen::Vector2d detect_object_;
    Eigen::Vector3d f_init_;

    STATE state_;
    
    //--------------------------------------    
    double threshold_; //0.8
    double search_range_;
    double search_duration_;
    double angle_range_; //5*M_PI/180
    int mode_;
    int swing_dir_;

    // !!! DO NOT USE RAW POINTER OF FILE !!!
    // FILE *save_data_fm;
    std::ofstream verify_pr_data;
    std::ofstream verify_ft_data;

public:
  AssembleVerifyActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);
  bool compute(ros::Time time) override;
  Eigen::Vector2d setSearchDirection(const Eigen::Vector3d &spiral_origin, const Eigen::Vector3d &start_point, const Eigen::Isometry3d &T_WA);

private:
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  void setSucceeded();
  void setAborted();
};