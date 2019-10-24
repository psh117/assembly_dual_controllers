#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleParallelAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleParallelActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::AssembleParallelAction> as_;

    assembly_msgs::AssembleParallelFeedback feedback_;
    assembly_msgs::AssembleParallelResult result_;
    assembly_msgs::AssembleParallelGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Matrix<double, 6, 1> f_measured_;
        
    Eigen::Matrix<double, 3, 3> init_rot_;
    Eigen::Vector3d origin_;

    bool is_first_;
    
    double duration_;
    double roll_, pitch_, yaw_;
    Eigen::Vector3d p1_;
    Eigen::Vector3d p2_;
    Eigen::Vector3d p3_;

  
public:
  AssembleParallelActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
};