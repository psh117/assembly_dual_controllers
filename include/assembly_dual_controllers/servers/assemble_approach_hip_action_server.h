#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleApproachHipAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

#include <fstream>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleApproachHipActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<assembly_msgs::AssembleApproachHipAction> as_;

  assembly_msgs::AssembleApproachHipFeedback feedback_;
  assembly_msgs::AssembleApproachHipResult result_;
  assembly_msgs::AssembleApproachHipGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

  Eigen::Isometry3d origin_;
  Eigen::Isometry3d current_;

  Eigen::Vector3d flange_to_assembly_point_;
  Eigen::Quaterniond flange_to_assembly_quat_;

  Eigen::Isometry3d T_7A_, T_WA_;
  
  double contact_force_rotate_;
  double contact_force_translate_;
  double time_limit_;
  double rotation_vel_;
  double translation_vel_;
  int rotate_dir_; // w.r.t global frame
  int translate_dir_; // w.r.t global frame
  int action_type_;

  Eigen::Vector3d translation_dir_;
  Eigen::Vector3d compliant_rotation_axis_;
  Eigen::Vector3d compliant_translation_axis_;
  Eigen::Matrix3d compliant_rotation_select_mtx_;
  Eigen::Matrix3d compliant_translation_select_mtx_;
  
  
  bool is_mode_changed_;
  
  int count_;

  Eigen::Vector6d accumulated_wrench_;
  Eigen::Vector6d accumulated_wrench_a_;
  
  std::ofstream rotate_fm_data {"rotate_fm_data.txt"};
  std::ofstream translate_fm_data {"translate_fm_data.txt"};
  
public:
  AssembleApproachHipActionServer(std::string name, ros::NodeHandle &nh,
                               std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu);

  bool compute(ros::Time time) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};