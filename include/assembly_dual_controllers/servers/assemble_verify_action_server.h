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
    Eigen::Isometry3d T_7A_, T_WA_;
    
    Eigen::Vector3d flange_to_assembly_point_;
    Eigen::Quaterniond flange_to_assembly_quat_;
    Eigen::Vector3d spiral_origin_;
    
    bool is_mode_changed_;
    int search_index_;
    int cnt_;
    
    Eigen::Vector3d verify_origin_;
    Eigen::Vector3d target_;    
    Eigen::Vector2d search_dir_;
    Eigen::Vector2d detect_object_;
    Eigen::Vector6d accumulated_wrench_;

    STATE state_;
    STATE prev_state_;
    
    //--------------------------------------    
    double threshold_; //0.8
    double search_range_;
    double search_duration_;
    
    Eigen::Vector2d max_f_detection_;

    // !!! DO NOT USE RAW POINTER OF FILE !!!
    // FILE *save_data_fm;
    std::ofstream verify_pr_data {"verify_pr_data.txt"};
    std::ofstream verify_ft_data {"verify_ft_data.txt"};

public:
  AssembleVerifyActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);
  bool compute(ros::Time time) override;
  bool printDebugState(ros::Time timt);

protected:
  void setSucceeded() override;
  void setAborted() override;
  void detectObjectLocation();
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
};