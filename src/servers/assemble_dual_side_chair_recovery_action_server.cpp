#include <assembly_dual_controllers/servers/assemble_dual_side_chair_recovery_action_server.h>

AssembleDualSideChairRecoveryActionServer::AssembleDualSideChairRecoveryActionServer(std::string name, ros::NodeHandle &nh,
                                                       std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleDualSideChairRecoveryActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleDualSideChairRecoveryActionServer::preemptCallback, this));
  as_.start();
}

void AssembleDualSideChairRecoveryActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->task_arm) != mu_.end() && mu_.find(goal_->assist_arm) != mu_.end() ) 
  {
    ROS_INFO("AssembleDualSideChairRecoveryAction goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleDualSideChairRecoveryActionServer::goalCallback] the name %s and %s are not in the arm list.", goal_->assist_arm.c_str(), goal_ ->task_arm.c_str());
    return ;
  }

  task_arm_origin_ = mu_[goal_->task_arm]->transform_;
  assist_arm_origin_ = mu_[goal_->assist_arm]->transform_;

  tilt_angle_ = goal_->tilt_angle;
  tilt_duration_ = goal_->tilt_duration;

  mu_[goal_->task_arm]->task_start_time_ = ros::Time::now();
  mu_[goal_->assist_arm]->task_start_time_ = ros::Time::now();
  
  flange_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  flange_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  flange_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  flange_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  flange_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  flange_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  flange_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;

  flange_to_pivot_point_(0) = goal_->ee_to_pivot.position.x;
  flange_to_pivot_point_(1) = goal_->ee_to_pivot.position.y;
  flange_to_pivot_point_(2) = goal_->ee_to_pivot.position.z;
  flange_to_pivot_quat_.x() = goal_->ee_to_pivot.orientation.x;
  flange_to_pivot_quat_.y() = goal_->ee_to_pivot.orientation.y;
  flange_to_pivot_quat_.z() = goal_->ee_to_pivot.orientation.z;
  flange_to_pivot_quat_.w() = goal_->ee_to_pivot.orientation.w;

  T_7A_.linear() = flange_to_assembly_quat_.toRotationMatrix();
  T_7A_.translation() = flange_to_assembly_point_;

  T_7p_.linear() = flange_to_pivot_quat_.toRotationMatrix();
  T_7p_.translation() = flange_to_pivot_point_;

  T_WA_ = task_arm_origin_ * T_7A_;
  T_wp_ = assist_arm_origin_*T_7p_;

  tilt_axis_ = getTiltDirection(T_7p_);

  control_running_ = true;

  task_state_ = TASK_ARM_STATE::READY;
  assist_state_ = ASSIST_ARM_STATE::READY;

  is_task_mode_changed_ = true;
  is_assist_mode_changed_ = true;
}

void AssembleDualSideChairRecoveryActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleDualSideChairRecoveryActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
    return false;

  if (mu_.find(goal_->task_arm) != mu_.end() && mu_.find(goal_->assist_arm) != mu_.end() ) {
    computeTaskArm(time, *mu_[goal_->task_arm]);
    computeAssistArm(time, *mu_[goal_->assist_arm]);
    return true;
  }
  else
  {
    ROS_ERROR("[AssembleSideChairActionServer::compute] the name %s and %s are not in the arm list.", goal_->assist_arm.c_str(), goal_ ->task_arm.c_str());
  }
  
  
  return false;
}

bool AssembleDualSideChairRecoveryActionServer::computeTaskArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
    return false;

  auto &mass = arm.mass_matrix_;
  auto &rotation = arm.rotation_;
  auto &position = arm.position_;
  auto &jacobian = arm.jacobian_;
  auto &tau_measured = arm.tau_measured_;
  auto &gravity = arm.gravity_;
  auto &xd = arm.xd_; //velocity

  Eigen::Vector3d f_star, m_star;
  Eigen::Vector6d f_star_zero, f_ext;
  double duration;

  f_ext = arm.f_ext_;
  task_arm_current_ = arm.transform_;
  duration = 1.0;

  if(is_task_mode_changed_)
  {
    arm.task_start_time_ = time;
    task_arm_origin_ = arm.transform_;
    is_task_mode_changed_ = false;
  }

  stateTransferFromAssisArm();

  switch (task_state_)
  {
  case READY:
    f_star = PegInHole::keepCurrentPosition(task_arm_origin_, task_arm_current_, xd);
    m_star = PegInHole::keepCurrentOrientation(task_arm_origin_, task_arm_current_, xd);
    break;

  case MOVE:
    if (timeOut(time.toSec(), arm.task_start_time_.toSec(), duration + 0.05))
    {
      if(reachGoal2D(task_arm_current_.translation(), T_WA_.translation(), 0.005, T_WA_)) task_state_ = TASK_ARM_STATE::SUCCESS;
      else task_state_ = TASK_ARM_STATE::FAIL;
    }

    f_star = PegInHole::threeDofMoveEE(task_arm_origin_, task_arm_current_, T_WA_.translation(), xd, T_7A_, time.toSec(), arm.task_start_time_.toSec(), duration);
    f_star = T_WA_.linear()*f_star;
    m_star = PegInHole::rotateWithMat(task_arm_origin_, task_arm_current_, xd, T_WA_.linear()*T_7A_.linear().inverse(), time.toSec(), arm.task_start_time_.toSec(), duration);
    break;
  }
  // f_star.setZero();
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  // save_sprial_data << f_ext.transpose() << std::endl;

  return true;
}

bool AssembleDualSideChairRecoveryActionServer::computeAssistArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
    return false;

  auto &mass = arm.mass_matrix_;
  auto &rotation = arm.rotation_;
  auto &position = arm.position_;
  auto &jacobian = arm.jacobian_;
  auto &tau_measured = arm.tau_measured_;
  auto &gravity = arm.gravity_;
  auto &xd = arm.xd_; //velocity

  Eigen::Vector3d f_star, m_star;
  Eigen::Vector6d f_star_zero, f_ext;
  Eigen::Vector3d f_contact;

  f_ext = arm.f_ext_;
  assist_arm_current_ = arm.transform_;

  f_contact = T_WA_.linear().inverse()*f_ext.head<3>() - accumulated_wrench_.head<3>();

  if(is_assist_mode_changed_)
  {
    arm.task_start_time_ = time;
    assist_arm_origin_ = arm.transform_;
    is_assist_mode_changed_ = false;
  }

  stateTransferFromTaskArm();

  switch (assist_state_)
  {
  case HOLD:
    f_star = PegInHole::keepCurrentPosition(assist_arm_origin_, assist_arm_current_, xd);
    m_star = PegInHole::keepCurrentOrientation(assist_arm_origin_, assist_arm_current_, xd);
    break;

  case LIFT_UP:
    if (timeOut(time.toSec(), arm.task_start_time_.toSec(), duration + 0.05))
    {
      is_assist_mode_changed_ = true;
      assist_state_ =  ASSIST_TASK_STATE::HOLD;
    }

      f_star = PegInHole::tiltMotion(assist_arm_origin_, assist_arm_current_, xd, T_7p_, tilt_axis_, tilt_angle_, time.toSec(), task_start_time_.toSec(), tilt_duration_).head<3>();
      m_star = PegInHole::tiltMotion(assist_arm_origin_, assist_arm_current_, xd, T_7p_, tilt_axis_, tilt_angle_, time.toSec(), task_start_time_.toSec(), tilt_duration_).tail<3>();
      break;
  }

    case LIFT_DOWN:

      if (checkForceLimit(f_contact, 10.0))
      {
        std::cout << "TILT BACK REACHED FORCE LIMIT" << std::endl;
        std::cout<<"f_contact: "<<f_contact.transpose()<<std::endl;
        assist_state_ =  ASSIST_TASK_STATE::FINISH;
      }

      f_star = PegInHole::tiltMotion(assist_arm_origin_, assist_arm_current_, xd, T_7p_, -10*tilt_axis_, tilt_angle_, time.toSec(), task_start_time_.toSec(), 10*tilt_duration_).head<3>();
      m_star = PegInHole::tiltMotion(assist_arm_origin_, assist_arm_current_, xd, T_7p_, -10*tilt_axis_, tilt_angle_, time.toSec(), task_start_time_.toSec(), 10*tilt_duration_).tail<3>();
      break;

    case FINISH:
      setSucceeded();
  }

  // f_star.setZero();
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  save_sprial_data << f_ext.transpose() << std::endl;

  return true;
}

void AssembleDualSideChairRecoveryActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleDualSideChairRecoveryActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}

void AssembleDualSideChairRecoveryActionServer::stateTransferFromTaskArm()
{
  if(task_state_ == TASK_ARM_STATE::MOVE) assist_state_ = ASSIST_ARM_STATE::HOLD;
  else if(task_state_ == TASK_ARM_STATE::FAIL) assist_state_ = ASSIST_ARM_STATE::LIFT_UP;
  else if(task_state_ == TASK_ARM_STATE::SUCCESS) assist_state_ = ASSIST_ARM_STATE::LIFT_DOWN;
}

void AssembleDualSideChairRecoveryActionServer::stateTransferFromAssisArm()
{
  if(assist_state_ == ASSIST_ARM_STATE::HOLD) task_state_ = task_state_;
  else if(assist_state_ == ASSIST_ARM_STATE::LIFT_UP) task_state_ = TASK_ARM_STATE::READY;
  else if(assist_state_ == ASSIST_ARM_STATE::LIFT_DOWN) task_state_ = TASK_ARM_STATE::READY;
}

