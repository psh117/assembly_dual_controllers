#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleTripleMoveAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#define TEST_PRINT

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleTripleMoveActionServer : public ActionServerBase
{
  enum MOVE_STATE : int
  {
    KEEPCURRENT = 0,
    EXEC = 1,
    KEEPSTOP = 2
  };
  struct a_state_
  {
    int count_{0};
    double motion_start_time_;
    double contact_force_;
    double speed_;
    bool heavy_mass_;
    bool is_mode_changed_{true};
    MOVE_STATE move_state_;
    Eigen::Isometry3d origin_;
    Eigen::Vector6d accumulated_wrench_;
  };

  actionlib::SimpleActionServer<assembly_msgs::AssembleTripleMoveAction> as_;
  assembly_msgs::AssembleTripleMoveFeedback feedback_;
  assembly_msgs::AssembleTripleMoveResult result_;
  assembly_msgs::AssembleTripleMoveGoalConstPtr goal_;
  void goalCallback() override;
  void preemptCallback() override;

  std::map<std::string, std::shared_ptr<a_state_>> hc_;
  a_state_ left_states;
  a_state_ right_states;
  a_state_ top_states;
  std::string upper_arm_;
  std::string stop_arm_1_;
  std::string stop_arm_2_;
  Eigen::Vector3d asm_dir_;
  double duration_;
  double m_star_limit_;
  bool control_running_;
  int succeed_flag{0};

  std::ofstream contact_force {"stefan_placement_contact_force.txt"}; 

public:
  AssembleTripleMoveActionServer(std::string name, ros::NodeHandle &nh,
                                 std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm, a_state_ &khc);

protected:
  void setSucceeded() override;
  void setAborted() override;
};