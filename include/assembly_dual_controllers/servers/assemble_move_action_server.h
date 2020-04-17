#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleMoveAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleMoveActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::AssembleMoveAction> as_;

    assembly_msgs::AssembleMoveFeedback feedback_;
    assembly_msgs::AssembleMoveResult result_;
    assembly_msgs::AssembleMoveGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Matrix<double, 6, 1> f_measured_;
    Eigen::Matrix<double, 6, 1> desired_xd_;
        
    Eigen::Matrix<double, 3, 3> init_rot_;
    Eigen::Vector3d origin_;

    Eigen::Vector3d target_;
    int type_;
    int dir_;
    int option_;
    double target_distance_;

    bool is_first_;
    double duration_;

public:
  AssembleMoveActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot

  double getDistance(const Eigen::Vector3d target, const Eigen::Vector3d start, const int type, const int dir);
};