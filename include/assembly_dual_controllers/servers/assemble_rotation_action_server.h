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
        
    Eigen::Matrix<double, 3, 3> init_rot_;
    Eigen::Vector3d origin_;

    Eigen::Vector3d target_force_;

    Eigen::Vector3d target_;
    int dir_;
    int option_;

    double f_threshold_;
    double range_; //5*M_PI/180
    int mode_;
    int swing_dir_;

    bool is_first_;
    double duration_;
    int assemble_dir_;

public:
  AssembleRotationActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot

  double getDistance(const Eigen::Vector3d target, const Eigen::Vector3d start, const int mode, const int dir);
};