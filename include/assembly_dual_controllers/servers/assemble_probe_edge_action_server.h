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
    SEARCH_OBJECT,
    BACK_TO_ORIGIN,
    PROBE_ON_THE_EDGE,
    LOST_CONTACT,
    COMPLETE,
    FAIL,
    DETECT_HOLE
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
  double contact_loss_;
  double blocking_force_;

  int probe_index_;
  int last_probe_index_;
  int search_index_;

  Eigen::Vector2d detect_object_;
  Eigen::Vector2d search_dir_;
  Eigen::Vector3d search_origin_; //the origin of spiral search
  Eigen::Vector3d probe_origine_;
  Eigen::Vector5d probing_sequence_;

  // !!! DO NOT USE RAW POINTER OF FILE !!!
  // FILE *force_moment_ee;

  std::ofstream probe_ft_data;
  std::ofstream probe_ft_data_lpf;

  bool is_mode_changed_;

public:
  AssembleProbeEdgeActionServer(std::string name, ros::NodeHandle &nh,
                               std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu);

  bool compute(ros::Time time) override;

private:
  void setSucceeded();
  void setAborted();
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  void generateProbingSequence(const Eigen::Vector2d approach_dir);
  Eigen::Vector2d setSearchDirection(const Eigen::Vector3d &search_origin, const Eigen::Vector3d &position);
  Eigen::Vector2d updateNormalVector(PROBE_DIRECTION probing_direction);
  Eigen::Vector3d generateNormalForce(const Eigen::Vector2d &normal_vector, const double f);
  Eigen::VEctor3d generateProbingForce(const Vector3d &origin,
                                       const Vector3d &current,
                                       const Vector3d &current_velocity,
                                       PROBE_DIRECTION probing_dir,
                                       const double probing_speed, //set positive
                                       const double t,
                                       const double t_0);
  //bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
};