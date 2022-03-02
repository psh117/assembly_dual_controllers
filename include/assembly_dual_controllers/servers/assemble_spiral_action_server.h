#pragma once

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/AssembleSpiralAction.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/control/criteria.h>
#include <assembly_msgs/SetSpiralGain.h>
#include <assembly_dual_controllers/utils/control/EM_GMM_estimator.h>

using namespace dyros_math;
using namespace Criteria;
using namespace PegInHole;

class AssembleSpiralActionServer : public ActionServerBase
{
  enum ASSEMBLY_STATE : int
  {
    READY = 0,
    EXEC = 1,
    RETURN = 2
  };

  actionlib::SimpleActionServer<assembly_msgs::AssembleSpiralAction> as_;

  assembly_msgs::AssembleSpiralFeedback feedback_;
  assembly_msgs::AssembleSpiralResult result_;
  assembly_msgs::AssembleSpiralGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

  Eigen::Matrix<double, 6, 1> f_measured_;

  Eigen::Vector3d init_pos_;
  Eigen::Isometry3d origin_;
  Eigen::Isometry3d current_;

  // Eigen::Vector3d force_compensation_;
  // Eigen::Vector3d moment_compensation_;

  Eigen::Vector6d twist_pos_save_;
  Eigen::Vector6d spiral_pos_save_;
  // parameters from .action file ---
  double lin_vel_;
  double pitch_;
  int mode_;
  int assemble_dir_;
  int partial_search_dir_;
  double friction_;
  double pressing_force_;
  double spiral_duration_;
  double range_;
  double twist_duration_;
  double minimum_depth_;

  double position_change_;
  double flange_to_assembly_point_distance_;
  double flange_to_drill_distance_;

  double global_target_;
  bool use_global_depth_;

  double force_error_sum_;

  Eigen::Vector3d flange_to_assembly_point_;
  Eigen::Quaterniond flange_to_assembly_quat_;

  Eigen::Isometry3d T_7A_, T_WA_;
  //--- parameters from .action file
  
  bool is_first_;
  bool is_mode_changed_;
  // bool heavy_mass_;
  bool set_tilt_;
  ASSEMBLY_STATE state_;

  int count_;
  Eigen::Vector6d accumulated_wrench_;
  Eigen::Vector6d accumulated_wrench_a_;

  Eigen::Vector3d return_to_origin_;
  // !!! DO NOT USE RAW POINTER OF FILE !!!
  // FILE *spiral_data;
  double init_dist_;

  // GMM estimators
  Estimator::GMM_model torque_model_small_;
  Estimator::GMM_model torque_model_large_;
  Estimator::GMM_model pos_vel_model_surface_;
  Estimator::GMM_model pos_vel_model_shallow_;
  Estimator::GMM_model pos_vel_model_floating_;
  Estimator::GMM_model pos_vel_model_deep_;


  // GMM estimators for triple case
  Estimator::GMM_model t_small_;
  Estimator::GMM_model t_medium_;
  Estimator::GMM_model t_large_;
  Estimator::GMM_model pv_surface_;
  Estimator::GMM_model pv_shallow_;
  Estimator::GMM_model pv_floating_;
  Estimator::GMM_model pv_moderate_;
  Estimator::GMM_model pv_deep_;

  int torque_data_dimension_ {21};//{14};
  int pos_vel_data_dimension_ {2};
  
  double spiral_elapsed_time_;
  double ready_elapsed_time_;

  Eigen::VectorXd torque_captured_, torque_init_;
  Eigen::VectorXd pos_vel_captured_, pos_vel_init_;

  Estimator::MultiSampling ms_;
  
  int cs_print_count_;
  
  std::ofstream spiral_search {"spiral_search.txt"};
  std::ofstream twist_search {"twist_search.txt"};
  std::ofstream spiral_arm_position {"spiral_arm_position.txt"};
  std::ofstream force_spiral_cmd {"force_spiral_cmd.txt"};
  std::ofstream contact_force{"contact_force.txt"};
  std::ofstream controller_debug {"controller_debug.txt"};
  std::ofstream save_torque{"save_joint_torque.txt"};    
  std::ofstream save_captured_torque{"spiral_captured_torque.txt"};
  std::ofstream save_captured_position{"spiral_captured_position.txt"};
  std::ofstream save_captured_velocity{"spiral_captured_velocity.txt"};
  std::ofstream save_captured_force{"spiral_captured_force.txt"};
  std::ofstream save_task_pfv{"spiral_captured_task_arm_pvf.txt"};
  std::ofstream save_contact_estimation{"save_contact_estimation.txt"};
  std::ofstream save_torque_label{"save_torque_label.txt"};
  std::ofstream save_pos_vel_label{"save_pos_vel_label.txt"};

  std::ofstream simple_test{"simple_test.txt"};
  
public:
  AssembleSpiralActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);
  std::map<std::string, Eigen::VectorXd> arm_gain_map_;
  ros::ServiceServer server_;

protected:
  void setSucceeded() override;
  void setAborted() override;

  bool setTarget(assembly_msgs::SetSpiralGain::Request &req, assembly_msgs::SetSpiralGain::Response &res);  
  void saveRobotState();
  double computeRelativeLocation();
  void getInitialTransformation();
  
  void estimateContactState();
  void initializeGMMModels();

  void estimateContactStateTriple();
  void initializeGMMModelsTriple();

};