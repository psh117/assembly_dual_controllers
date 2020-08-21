#include <assembly_dual_controllers/servers/assemble_dual_approach_action_server.h>

AssembleDualApproachActionServer::AssembleDualApproachActionServer(std::string name, ros::NodeHandle &nh,
                                                                   std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleDualApproachActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleDualApproachActionServer::preemptCallback, this));
  as_.start();
}

void AssembleDualApproachActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->task_arm) != mu_.end() && mu_.find(goal_->assist_arm) != mu_.end() ) 
  {
    ROS_INFO("AssembleDualApproachAction goal has been received.");
  } 
  else
  {
    ROS_ERROR("[AssembleDualApproachActionServer::goalCallback] the name %s and %s are not in the arm list.", goal_->assist_arm.c_str(), goal_ ->task_arm.c_str());
    return;
  }

  task_arm_origin_ = mu_[goal_->task_arm]->transform_;
  assist_arm_origin_ = mu_[goal_->assist_arm]->transform_;

  mu_[goal_->task_arm]->task_start_time_ = ros::Time::now();
  mu_[goal_->assist_arm]->task_start_time_ = ros::Time::now();

  f_measured_.setZero();

  contact_force_ = goal_->contact_force;
  descent_speed_ = goal_->descent_speed;
  time_limit_ = goal_->time_limit;
  tilt_angle_ = goal_->tilt_angle;
  tilt_duration_ = goal_->tilt_duration;
  state_ = (ASSEMBLY_STATE)goal_->state;
  set_tilt_ = goal_->set_tilt;

  task_arm_to_assembly_point_(0) = goal_->task_arm_to_assemble.position.x;
  task_arm_to_assembly_point_(1) = goal_->task_arm_to_assemble.position.y;
  task_arm_to_assembly_point_(2) = goal_->task_arm_to_assemble.position.z;
  task_arm_to_assembly_quat_.x() = goal_->task_arm_to_assemble.orientation.x;
  task_arm_to_assembly_quat_.y() = goal_->task_arm_to_assemble.orientation.y;
  task_arm_to_assembly_quat_.z() = goal_->task_arm_to_assemble.orientation.z;
  task_arm_to_assembly_quat_.w() = goal_->task_arm_to_assemble.orientation.w;

  asssist_to_target_point_(0) = goal_->assist_arm_to_target.position.x;
  asssist_to_target_point_(1) = goal_->assist_arm_to_target.position.y;
  asssist_to_target_point_(2) = goal_->assist_arm_to_target.position.z;
  asssist_to_target_quat_.x() = goal_->assist_arm_to_target.orientation.x;
  asssist_to_target_quat_.y() = goal_->assist_arm_to_target.orientation.y;
  asssist_to_target_quat_.z() = goal_->assist_arm_to_target.orientation.z;
  asssist_to_target_quat_.w() = goal_->assist_arm_to_target.orientation.w;
  
  std::string panda_left;
  panda_left = "panda_left";
  if(goal_->task_arm.c_str() == panda_left) task_arm_to_assist_arm_point_(1) = -0.6;
  else                                    task_arm_to_assist_arm_point_(1) = 0.6;
  task_arm_to_assist_arm_point_(2) = 0.0;
  task_arm_to_assist_arm_quat_.x() = 0.0;
  task_arm_to_assist_arm_quat_.y() = 0.0;
  task_arm_to_assist_arm_quat_.z() = 0.0;
  task_arm_to_assist_arm_quat_.w() = 1.0;

  T_7A_.linear() = task_arm_to_assembly_quat_.toRotationMatrix();
  T_7A_.translation() = task_arm_to_assembly_point_;

  T_7D_.linear() = asssist_to_target_quat_.toRotationMatrix();
  T_7D_.translation() = asssist_to_target_point_;

  T_task_assist_.linear() = task_arm_to_assist_arm_quat_.toRotationMatrix();
  T_task_assist_.translation() = task_arm_to_assist_arm_point_;

  tilt_axis_ = getTiltDirection(T_7A_);

  T_task_A_ = task_arm_origin_ * T_7A_;
  T_assist_D_ = assist_arm_origin_*T_7D_;

  T_AD_ = T_task_A_.inverse()* T_task_assist_ * T_assist_D_;

  is_ready_first_ = true;
  is_approach_first_ = true;
  is_tilt_back_first_ = true;

  if (force_moment.is_open())
    force_moment.close();
  if (force_moment_lpf.is_open())
    force_moment_lpf.close();

  force_moment.open("fm_dual_approach_data.txt");
  force_moment_lpf.open("fm_dual_approach_data_lfp.txt");

  control_running_ = true;

  std::cout<<"Approach using dual arm"<<std::endl;
  std::cout << "set tilt: " << set_tilt_ << std::endl;
  std::cout << "state_: " << state_ << std::endl;

  std::cout << "T_7A_: \n" << T_7A_.matrix() << std::endl;
  std::cout << "task_arm_origin_: \n" << task_arm_origin_.matrix() << std::endl;
  std::cout << "T_task_A_: \n"<< T_task_A_.matrix() << std::endl;
  std::cout << "T_7D_: \n"<< T_7D_.matrix() << std::endl;
  std::cout<<"assist_arm_origin_: \n"<<assist_arm_origin_.matrix()<<std::endl;
  std::cout << "T_asisst_D: \n"<< T_assist_D_.matrix() << std::endl;
  std::cout << "T_AD_: \n"<< T_AD_.matrix() << std::endl;
  std::cout<<"T_task_assist_: \n"<<T_task_assist_.matrix()<<std::endl;
  
  std::cout << "contact_threshold: " << contact_force_ << std::endl;
  std::cout << "descent speed: " << descent_speed_ << std::endl;
  
}

void AssembleDualApproachActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleDualApproachActionServer::compute(ros::Time time)
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
    ROS_ERROR("[AssembleDualApproachActionServer::goalCallback] the name %s and %s are not in the arm list.", goal_->assist_arm.c_str(), goal_ ->task_arm.c_str());
  }

  return false;
}

bool AssembleDualApproachActionServer::computeTaskArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
    return false;

  auto &mass = arm.mass_matrix_;
  auto &rotation = arm.rotation_;
  auto &position = arm.position_;
  auto &jacobian = arm.jacobian_;
  auto &tau_measured = arm.tau_measured_;
  auto &gravity = arm.gravity_;
  auto &xd = arm.xd_;

  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Matrix<double, 6, 1> f_star_zero;
  Eigen::Vector6d f_ext;
  double run_time;

  f_ext = arm.f_ext_;
  task_arm_current_ = arm.transform_;

  switch (state_)
  {
  case READY:
    if (is_ready_first_)
    {
      tilt_start_time_ = time.toSec();
      is_ready_first_ = false;
      std::cout << "TILT" << std::endl;
    }

    if (timeOut(time.toSec(), tilt_start_time_, tilt_duration_ + 1.0))
    {
      state_ = EXEC;
      std::cout << "TILT DONE" << std::endl;
      // setSucceeded();
    }

    f_star = PegInHole::tiltMotion(task_arm_origin_, task_arm_current_, xd, T_7A_, tilt_axis_, tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).head<3>();
    m_star = PegInHole::tiltMotion(task_arm_origin_, task_arm_current_, xd, T_7A_, tilt_axis_, tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).tail<3>();

    break;

  case EXEC:
    if (is_approach_first_)
    {
      task_arm_origin_ = arm.transform_;
      approach_star_time_ = time.toSec();
      is_approach_first_ = false;
      std::cout << "APPROACH" << std::endl;
    }

    run_time = time.toSec() - approach_star_time_;

    if (run_time > 0.05 && Criteria::checkContact(f_ext.head<3>(), T_task_A_, contact_force_))
    {
      if (set_tilt_back_)
        state_ = TILT_BACK;
      else
        state_ = IGNORE;
      // std::cout<<"running time: "<< run_time<<std::endl;
      std::cout << "CHECK CONTATCT!!!!!" << std::endl;
      setSucceeded();
    }

    if (timeOut(time.toSec(), arm.task_start_time_.toSec(), time_limit_)) //duration wrong??
    {
      std::cout << "Time out" << std::endl;
      setAborted();
    }


    f_star = PegInHole::pinMoveEE(task_arm_origin_, task_arm_current_, xd, T_7A_, T_AD_, descent_speed_, time.toSec(), approach_star_time_);
    f_star = T_task_A_.linear() * f_star;

    m_star = PegInHole::keepCurrentOrientation(task_arm_origin_, task_arm_current_, xd, 200, 5);
    break;

  case TILT_BACK:
    if (is_tilt_back_first_)
    {
      tilt_start_time_ = time.toSec();
      is_tilt_back_first_ = false;
      std::cout << "TILT BACK" << std::endl;
    }

    if (timeOut(time.toSec(), tilt_start_time_, tilt_duration_))
    {
      std::cout << "TILT BACK DONE" << std::endl;
      setSucceeded();
    }

    f_star = PegInHole::tiltMotion(task_arm_origin_, task_arm_current_, xd, T_7A_, tilt_axis_, -tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).head<3>();
    m_star = PegInHole::tiltMotion(task_arm_origin_, task_arm_current_, xd, T_7A_, tilt_axis_, -tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).tail<3>();

    break;

  case IGNORE:
    std::cout << "move to the next step" << std::endl;
    setSucceeded();
    break;
  }

  // if(state_ == (ASSEMBLY_STATE) EXEC)
  // {
  //   Eigen::Vector6d f_contact;
  //   f_contact.head<3>() = T_WA_.linear().inverse()*f_ext.head<3>() - accumulated_wrench_a_.head<3>();
  //   f_contact.tail<3>() = T_WA_.linear().inverse()*f_ext.tail<3>() - accumulated_wrench_a_.tail<3>();
  //   force_moment<< f_contact.transpose()<<std::endl;
  // }

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;

  // f_star_zero.setZero();
  // std::cout<<"f_measured: "<<f_measured_.transpose()<<std::endl;
  // std::cout<<"f_filtered: "<<f_ext.transpose()<<std::endl;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  return true;
}

bool AssembleDualApproachActionServer::computeAssistArm(ros::Time time, FrankaModelUpdater &arm)
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

  f_ext = arm.f_ext_;
  assist_arm_current_ = arm.transform_;

  f_star = PegInHole::keepCurrentPosition(assist_arm_origin_, assist_arm_current_, xd, 5000, 100);
  m_star = PegInHole::keepCurrentOrientation(assist_arm_origin_, assist_arm_current_, xd, 200, 5);
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  return true;
}

void AssembleDualApproachActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleDualApproachActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}