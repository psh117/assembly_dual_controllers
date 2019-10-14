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

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Matrix<double, 6, 1> f_measured_;
    Eigen::Matrix<double, 6, 1> desired_xd_;
        
    Eigen::Matrix<double, 3, 3> init_rot_;
    Eigen::Vector3d origin_;

    std::vector<double> mx_;
    std::vector<double> my_;
    std::vector<double> mz_;

    double threshold_; //0.8
    double range_; //5*M_PI/180
    int mode_;
    int swing_dir_;

    

public:
  AssembleVerifyActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot

  void saveMoment(const Eigen::Vector3d m);
};