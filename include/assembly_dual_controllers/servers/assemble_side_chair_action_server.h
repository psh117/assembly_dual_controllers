#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleSideChairAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleSideChairActionServer : public ActionServerBase
{
    actionlib::SimpleActionServer<assembly_msgs::AssembleSideChairAction> as_;

    assembly_msgs::AssembleSideChairFeedback feedback_;
    assembly_msgs::AssembleSideChairResult result_;
    assembly_msgs::AssembleSideChairGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Matrix<double, 6, 1> f_measured_;
    Eigen::Matrix<double, 6, 1> desired_xd_;
        
    Eigen::Matrix<double, 3, 3> init_rot_pusher_;
    Eigen::Vector3d origin_pusher_;

    Eigen::Matrix<double, 3, 3> init_rot_mover_;
    Eigen::Vector3d origin_mover_;

    Eigen::Vector3d target_force_;

    double linear_vel_;
    double pitch_;
    int assemble_dir_;
    double duration_;
    
    FILE *save_data_fm_mover_;
    FILE *save_data_pv_mover_;
    FILE *save_data_fm_pusher_;
    FILE *save_data_pv_pusher_;
    
public:
  AssembleSideChairActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeMover(ros::Time time, FrankaModelUpdater &arm);
  bool computePusher(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};