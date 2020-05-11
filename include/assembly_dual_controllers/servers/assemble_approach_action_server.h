#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleApproachAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <assembly_dual_controllers/utils/control/criteria_2.h>
#include <fstream>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleApproachActionServer : public ActionServerBase
{
  enum ASSEMBLY_STATE
  {
    READY,
    EXEC,
    TILT_BACK,
    IGNORE
  };
  actionlib::SimpleActionServer<assembly_msgs::AssembleApproachAction> as_;

  assembly_msgs::AssembleApproachFeedback feedback_;
  assembly_msgs::AssembleApproachResult result_;
  assembly_msgs::AssembleApproachGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

  Eigen::Matrix<double, 6, 1> f_measured_;
  Eigen::Matrix<double, 6, 1> desired_xd_;
  Eigen::Vector3d desired_x_;
  Eigen::Matrix<double, 7, 6> j_inverse_;

  Eigen::Vector3d init_pos_;
  Eigen::Matrix3d init_rot_;

  Eigen::Isometry3d origin_;
  Eigen::Isometry3d current_;

  Eigen::Vector3d ee_to_assembly_point_;
  Eigen::Quaterniond ee_to_assembly_quat_;

  Eigen::Isometry3d T_EA_, T_WA_;

  Eigen::Vector3d tilt_axis_;

  Eigen::Vector3d approach_origin_;

  double descent_speed_;
  double contact_force_;
  int assemble_dir_;
  int mode_;
  double time_limit_;
  bool set_tilt_;
  bool set_tilt_back_;
  double tilt_angle_;
  double tilt_duration_;

  double tilt_start_time_;
  double approach_star_time_;

  ASSEMBLY_STATE state_;

  bool is_ready_first_;
  bool is_approach_first_;
  bool is_tilt_back_first_;

  // !!! DO NOT USE RAW POINTER OF FILE !!!
  // FILE *force_moment_ee;

  std::ofstream force_moment;

public:
  AssembleApproachActionServer(std::string name, ros::NodeHandle &nh,
                               std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu);

  bool compute(ros::Time time) override;

private:
  void setSucceeded();
  void setAborted();
  
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};