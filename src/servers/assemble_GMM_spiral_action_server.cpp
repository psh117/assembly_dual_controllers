#include <assembly_dual_controllers/servers/assemble_GMM_spiral_action_server.h>

AssembleGMMSpiralActionServer::AssembleGMMSpiralActionServer(std::string name, ros::NodeHandle &nh,
                                                             std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleGMMSpiralActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleGMMSpiralActionServer::preemptCallback, this));
  as_.start();
}

void AssembleGMMSpiralActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if (mu_.find(goal_->task_arm) != mu_.end() && mu_.find(goal_->assist_arm) != mu_.end())
  {
    ROS_INFO("AssembleGMMSpiralAction goal has been received.");
  }
  else
  {
    ROS_ERROR("[AssembleGMMSpiralActionServer::goalCallback] the name %s and %s are not in the arm list.", goal_->assist_arm.c_str(), goal_->task_arm.c_str());
    return;
  }

  task_arm_origin_ = mu_[goal_->task_arm]->transform_;
  assist_arm_origin_ = mu_[goal_->assist_arm]->transform_;

  lin_vel_ = goal_->linear_vel;
  pitch_ = goal_->pitch;
  mode_ = goal_->mode;
  depth_ = goal_->depth;
  friction_ = goal_->friction;
  spiral_duration_ = goal_->spiral_duration;
  pressing_force_ = goal_->pressing_force;
  range_ = goal_->range;
  twist_duration_ = goal_->twist_duration;
  assist_arm_action_ = goal_->assist_arm_action;

  count_  = 0;
  is_mode_changed_ = true;
  state_ = READY;

  mu_[goal_->task_arm]->task_start_time_ = ros::Time::now();
  mu_[goal_->assist_arm]->task_start_time_ = ros::Time::now();
  mu_[goal_->task_arm]->task_end_time_ = ros::Time(mu_[goal_->task_arm]->task_start_time_.toSec() + spiral_duration_);

  flange_to_assembly_point_task_(0) = goal_->ee_to_assemble_task.position.x;
  flange_to_assembly_point_task_(1) = goal_->ee_to_assemble_task.position.y;
  flange_to_assembly_point_task_(2) = goal_->ee_to_assemble_task.position.z;
  flange_to_assembly_quat_task_.x() = goal_->ee_to_assemble_task.orientation.x;
  flange_to_assembly_quat_task_.y() = goal_->ee_to_assemble_task.orientation.y;
  flange_to_assembly_quat_task_.z() = goal_->ee_to_assemble_task.orientation.z;
  flange_to_assembly_quat_task_.w() = goal_->ee_to_assemble_task.orientation.w;

  flange_to_assembly_point_assist_(0) = goal_->ee_to_assemble_assist.position.x;
  flange_to_assembly_point_assist_(1) = goal_->ee_to_assemble_assist.position.y;
  flange_to_assembly_point_assist_(2) = goal_->ee_to_assemble_assist.position.z;
  flange_to_assembly_quat_assist_.x() = goal_->ee_to_assemble_assist.orientation.x;
  flange_to_assembly_quat_assist_.y() = goal_->ee_to_assemble_assist.orientation.y;
  flange_to_assembly_quat_assist_.z() = goal_->ee_to_assemble_assist.orientation.z;
  flange_to_assembly_quat_assist_.w() = goal_->ee_to_assemble_assist.orientation.w;

  T_7A_task_.linear() = flange_to_assembly_quat_task_.toRotationMatrix();
  T_7A_task_.translation() = flange_to_assembly_point_task_;
  T_WA_task_ = task_arm_origin_ * T_7A_task_;

  T_7A_assist_.linear() = flange_to_assembly_quat_assist_.toRotationMatrix();
  T_7A_assist_.translation() = flange_to_assembly_point_assist_;
  T_WA_assist_ = assist_arm_origin_ * T_7A_assist_;

  std::cout << "sprial origin: " << task_arm_origin_.translation().transpose() << std::endl;
  if (mode_ == 1)
    std::cout << "single peg in hole" << std::endl;
  if (mode_ == 2)
    std::cout << "dual peg in hole" << std::endl;

  result_.spiral_origin.position.x = T_WA_task_.translation()(0);
  result_.spiral_origin.position.y = T_WA_task_.translation()(1);
  result_.spiral_origin.position.z = T_WA_task_.translation()(2);
  Eigen::Quaterniond temp(T_WA_task_.linear());
  result_.spiral_origin.orientation.x = temp.x();
  result_.spiral_origin.orientation.y = temp.y();
  result_.spiral_origin.orientation.z = temp.z();
  result_.spiral_origin.orientation.w = temp.w();

  std::cout << "pressing_force: " << pressing_force_ << std::endl;
  std::cout << "spiral pitch : " << pitch_ << std::endl;
  std::cout << "save the spiral origin: \n"
            << result_.spiral_origin << std::endl;
  std::cout << "T_7A_task : \n"
            << T_7A_task_.matrix() << std::endl;

  control_running_ = true;
}

void AssembleGMMSpiralActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleGMMSpiralActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
    return false;

  if (mu_.find(goal_->task_arm) != mu_.end() && mu_.find(goal_->assist_arm) != mu_.end())
  {
    computeTaskArm(time, *mu_[goal_->task_arm]);
    computeAssistArm(time, *mu_[goal_->assist_arm]);
    return true;
  }
  else
  {
    ROS_ERROR("[AssembleGMMSpiralActionServer::compute] the name %s and %s are not in the arm list.", goal_->assist_arm.c_str(), goal_->task_arm.c_str());
  }

  return false;
}

bool AssembleGMMSpiralActionServer::computeTaskArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
    return false;

  // auto &mass = arm.mass_matrix_;
  // auto &rotation = arm.rotation_;
  // auto &position = arm.position_;
  auto &jacobian = arm.jacobian_;
  // auto &tau_measured = arm.tau_measured_;
  // auto &gravity = arm.gravity_;
  auto &xd = arm.xd_; //velocity

  Eigen::Vector6d f_star;
  Eigen::Vector3d f_star_motion, f_star_active_force;
  Eigen::Vector3d m_star;
  Eigen::Vector6d f_star_zero, f_d;
  Eigen::Vector6d f_ext;
  double run_time;
  int cnt_max = 150;
  int cnt_start = 50;
  Eigen::Vector3d reaction_force;
  Eigen::Vector3d pos, rot;

  f_ext = arm.f_ext_;
  task_arm_current_ = arm.transform_;

  rot = dyros_math::getPhi(assist_arm_current_.linear(), assist_arm_origin_.linear());
  rot = assist_arm_origin_.linear().inverse()*rot;
  pos = assist_arm_current_.translation() - assist_arm_origin_.translation();
  pos = assist_arm_origin_.linear().inverse()*pos;

  reaction_force = assist_arm_current_.linear().inverse() * f_ext.head<3>();

  if(is_mode_changed_)
  {
    arm.task_start_time_ = time;
    task_arm_origin_ = arm.transform_;
    is_mode_changed_ = false;    
  }

  switch (state_)
  {
  case READY:
    if (count_ > cnt_start)
      PegInHole::getCompensationWrench(accumulated_wrench_, f_ext, cnt_start, count_, cnt_max);
    count_++;
    if (count_ >= cnt_max)
    {
      state_ = EXEC;
      is_mode_changed_ = true;
      count_ = cnt_max;
      accumulated_wrench_a_.head<3>() = T_WA_task_.linear().inverse() * accumulated_wrench_.head<3>();
      accumulated_wrench_a_.tail<3>() = T_WA_task_.linear().inverse() * accumulated_wrench_.tail<3>();
      std::cout << "output for compensation: " << accumulated_wrench_.transpose() << std::endl;
      std::cout << "start spiral search" << std::endl;
    }

    f_star_motion = PegInHole::keepCurrentPose(task_arm_origin_, task_arm_current_, xd, 1000, 10, 2000, 20).head<3>(); //w.r.t {W}
    f_star_active_force.setZero();

    m_star = PegInHole::rotateWithMat(task_arm_origin_, task_arm_current_, xd, task_arm_origin_.linear(), time.toSec(), arm.task_start_time_.toSec(), cnt_max / 1000, 3000, 20);
    break;

  case EXEC:
    if (timeOut(time.toSec(), arm.task_start_time_.toSec(), spiral_duration_)) //duration wrong??
    {
      std::cout << "Time out" << std::endl;
      setAborted();
    }

    run_time = time.toSec() - arm.task_start_time_.toSec();
    if (run_time > 0.5 && detectHole(task_arm_origin_, task_arm_current_, f_ext.head<3>() - accumulated_wrench_.head<3>(), T_WA_task_, friction_))
    {
      std::cout << "HOLE IS DETECTED" << std::endl;
      setSucceeded();
      break;
    }

    f_star = PegInHole::generateSpiralEE2(task_arm_origin_, task_arm_current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_task_, time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec());
    f_star_motion = f_star.head<3>();
    f_star_active_force = f_star.tail<3>();

    f_star_motion = T_WA_task_.linear() * f_star_motion;
    f_star_active_force = T_WA_task_.linear() * f_star_active_force;

    //signle peg in hole
    if (mode_ == 1)
      m_star = PegInHole::keepCurrentOrientation(task_arm_origin_, task_arm_current_, xd, 2000., 15.0);

    //dual peg in hole
    // if (mode_ == 2)
    // {
    //   Eigen::Vector6d f_spiral;
    //   f_spiral = PegInHole::generateTwistSpiralEE(task_arm_origin_, task_arm_current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_task_, range_, time.toSec(), arm.task_start_time_.toSec(), spiral_duration_, twist_duration_);
    //   f_star = T_WA_task_.linear() * (f_spiral.head<3>());
    //   m_star = T_WA_task_.linear() * f_spiral.tail<3>();
    // }
  }

  f_star_zero.head<3>() = f_star_motion;
  f_star_zero.tail<3>() = m_star;

  f_d.setZero();
  f_d.tail<3>() = f_star_active_force;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * (arm.modified_lambda_matrix_ * f_star_zero + f_d);
  arm.setTorque(desired_torque);

  save_task_arm_force << reaction_force.transpose() << std::endl; // w.r.t {7}
  save_task_arm_pose << pos.transpose()<<" "<< rot.transpose()<<std::endl; // w.r.t {7}
  save_task_arm_torque << arm.tau_ext_filtered_.transpose() << std::endl; 

  return true;
}

bool AssembleGMMSpiralActionServer::computeAssistArm(ros::Time time, FrankaModelUpdater &arm)
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
  Eigen::Isometry3d T_7A; // transform of assembly point wrt assist arm
  Eigen::Vector3d reaction_force;
  Eigen::Vector3d pos, rot;
  double reaction_force_sum;

  f_ext = arm.f_ext_;
  assist_arm_current_ = arm.transform_;
  T_7A = assist_arm_current_.inverse() * T_WA_task_;

  rot = dyros_math::getPhi(assist_arm_current_.linear(), assist_arm_origin_.linear());
  rot = assist_arm_origin_.linear().inverse()*rot;
  pos = assist_arm_current_.translation() - assist_arm_origin_.translation();
  pos = assist_arm_origin_.linear().inverse()*pos;

  if (assist_arm_action_ == 1) // hold
  {
    f_star = PegInHole::keepCurrentPosition(assist_arm_origin_, assist_arm_current_, xd, 1000, 20);
  }
  else if (assist_arm_action_ == 2) // press
  {
    f_star = PegInHole::pressCubicEE(assist_arm_origin_, assist_arm_current_, xd, T_WA_assist_, 3.0, time.toSec(), arm.task_start_time_.toSec(), 2.0);
    f_star = T_WA_assist_.linear() * f_star;
  }

  m_star = PegInHole::keepCurrentOrientation(assist_arm_origin_, assist_arm_current_, xd, 3000, 15);

  reaction_force = assist_arm_current_.linear().inverse() * f_ext.head<3>();

  reaction_force_sum = sqrt(pow(reaction_force(0), 2) + pow(reaction_force(1), 2));
  // if (time.toSec() > arm.task_start_time_.toSec() + 5.0)
  // {
  //   if (checkForceLimit(reaction_force_sum, friction_))
  //   {
  //     setSucceeded();
  //     std::cout << "HOLE IS DETECTED!" << std::endl;
  //     std::cout << "reaction_force: " << reaction_force.transpose() << std::endl;
  //   }
  // }

  // f_star.setZero();
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_ * f_star_zero;
  arm.setTorque(desired_torque);

  save_assist_arm_force << reaction_force.transpose() << std::endl; // w.r.t {7}
  save_assist_arm_pose << pos.transpose()<<" "<< rot.transpose()<<std::endl; // w.r.t {7}
  save_assist_arm_torque << arm.tau_ext_filtered_.transpose() << std::endl; 

  return true;
}

void AssembleGMMSpiralActionServer::setSucceeded()
{
  result_.is_completed = true;
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleGMMSpiralActionServer::setAborted()
{
  // result_.is_completed = true;
  as_.setAborted(result_);
  control_running_ = false;
}