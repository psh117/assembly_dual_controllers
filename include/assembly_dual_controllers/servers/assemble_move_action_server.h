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
  enum STATE : int
  {
    LIFT_UP,  //0
    MOVE,     //1
    ROTATE,   //2
    COMPLETE  //3
  };

    actionlib::SimpleActionServer<assembly_msgs::AssembleMoveAction> as_;

    assembly_msgs::AssembleMoveFeedback feedback_;
    assembly_msgs::AssembleMoveResult result_;
    assembly_msgs::AssembleMoveGoalConstPtr goal_;

    void goalCallback() override;
    void preemptCallback() override;

    Eigen::Matrix<double, 6, 1> f_measured_;
    Eigen::Matrix<double, 6, 1> desired_xd_;
        
    Eigen::Isometry3d origin_, current_, target_;
    Eigen::Isometry3d T_EA_, T_WA_;
    Eigen::Vector3d target_pos_, ee_to_assembly_point_;
    Eigen::Quaterniond target_quat_, ee_to_assembly_quat_;
  
    int type_;
    int dir_;
    int option_;
    double target_distance_;

    bool is_mode_changed_;
    STATE state_;
    double duration_;
    double lift_up_distance_;
    double move_distance_;
    double speed_;
    int litf_dir_;


public:
  AssembleMoveActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;

private:
  void setSucceeded();
  void setAborted();
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot

  double getDistance(const Eigen::Vector3d target, const Eigen::Vector3d start, const int type, const int dir);
};