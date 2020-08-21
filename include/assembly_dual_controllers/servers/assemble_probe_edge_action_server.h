#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleProbeEdgeAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <fstream>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;


class AssembleProbeEdgeActionServer : public ActionServerBase
{
  enum PROBE_DIRECTION : int
  {
    UP = 0,
    RIGHT = 1,
    LEFT = 2,
    DOWN = 3
  };

  enum PROBE_STATE : int
  {
    READY,
    PROBE_ON_THE_EDGE,
    LOST_CONTACT,
    RECOVER_CONTACT,
    COMPLETE,
    FAIL
  };

  actionlib::SimpleActionServer<assembly_msgs::AssembleProbeEdgeAction> as_;
  assembly_msgs::AssembleProbeEdgeFeedback feedback_;
  assembly_msgs::AssembleProbeEdgeResult result_;
  assembly_msgs::AssembleProbeEdgeGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;
  
  PROBE_DIRECTION probe_direction_;
  PROBE_STATE state_;
  
  Eigen::Isometry3d origin_;
  Eigen::Isometry3d current_;

  Eigen::Vector3d ee_to_assembly_point_;
  Eigen::Quaterniond ee_to_assembly_quat_;

  Eigen::Isometry3d T_EA_, T_WA_;

  double contact_force_;
  double contact_loss_threshold_;
  double attraction_force_;
  double probing_speed_;

  int probe_index_;
  int last_probe_index_;
  int dir_index_;
  int count_;

  Eigen::Vector2d object_location_;
  Eigen::Vector3d probe_origin_;
  Eigen::Vector5d probing_sequence_;
  Eigen::Vector6d f_star_zero_;
  Eigen::Vector6d accumulated_wrench_;
  Eigen::Vector6d init_force_ext_;
  Eigen::Vector3d normal_vector_;
  Eigen::Matrix3d selection_matrix_; // 1 = probing force direction, 0 = attraction force direction

  // !!! DO NOT USE RAW POINTER OF FILE !!!
  // FILE *force_moment_ee;

  std::ofstream probe_ft_data;
  std::ofstream probe_pr_data;
  std::ofstream probe_vel_data;
  std::ofstream probe_ft_cmd_data;

  bool is_mode_changed_;

public:
  AssembleProbeEdgeActionServer(std::string name, ros::NodeHandle &nh,
                               std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu);

  bool compute(ros::Time time) override;  
  void generateProbingSequence(const Eigen::Vector2d &object_location);
  Eigen::Vector2d updateNormalVector(PROBE_DIRECTION probing_direction);
  Eigen::Vector3d generateNormalForce(const Eigen::Vector2d &normal_vector, const double f);
  Eigen::Vector3d generateProbingForce(const Eigen::Isometry3d &origin,
                                       const Eigen::Isometry3d &current,
                                       const Eigen::Ref<const Eigen::Vector6d> &xd,
                                       const Eigen::Isometry3d &T_ea,
                                       const int probing_dir,
                                       const double probing_speed, //set positive
                                       const double t,
                                       const double t_0);

private:
  void setSucceeded();
  void setAborted();
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};