#include <assembly_dual_controllers/servers/assemble_back_forth_action_server.h>

AssembleBackForthActionServer::AssembleBackForthActionServer(std::string name, ros::NodeHandle &nh,
                                                               std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleBackForthActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleBackForthActionServer::preemptCallback, this));
  as_.start();
}

void AssembleBackForthActionServer::goalCallback()
{
  goal_ = as_.acceptNewGoal();

  contact_force_ = goal_ -> contact_force;
  linear_vel_ = goal_ -> linear_vel;
  duration_ = goal_ -> duration;
  arm_name_ = goal_ -> arm_name;
  pitch_ = goal_ -> pitch;
  axis_ = goal_ -> axis;

  flange_to_assembly_point_(0) = goal_->T_7A.position.x;
  flange_to_assembly_point_(1) = goal_->T_7A.position.y;
  flange_to_assembly_point_(2) = goal_->T_7A.position.z;
  flange_to_assembly_quat_.x() = goal_->T_7A.orientation.x;
  flange_to_assembly_quat_.y() = goal_->T_7A.orientation.y;
  flange_to_assembly_quat_.z() = goal_->T_7A.orientation.z;
  flange_to_assembly_quat_.w() = goal_->T_7A.orientation.w;

  mu_[arm_name_]->task_start_time_ = ros::Time::now();
  count_ = 0;
  is_mode_changed_ = true;
  control_running_ = true;
  move_state_ = EXEC;
  origin_ = mu_[arm_name_] -> transform_;
  accumulated_wrench_.setZero();
  T_7A_.linear() = flange_to_assembly_quat_.toRotationMatrix(); // it means T_7A_
  T_7A_.translation() = flange_to_assembly_point_;
  T_WA_ = origin_ * T_7A_;

#ifdef TEST_PRINT
  std::cout<<"TESTING BACK AND FORTH ACTION"<<std::endl;
  std::cout<<"contact_force: "<<contact_force_<<std::endl;
  std::cout<<"linear_vel: "<<linear_vel_<<std::endl;
  std::cout<<"duration: "<<duration_<<std::endl;
  std::cout<<"pitch: "<<pitch_<<std::endl;
  std::cout<<"axis: "<<axis_<<std::endl;
#endif
  if (mu_.find(arm_name_) != mu_.end())
  {
    ROS_INFO("AssembleBackForthAction goal has been received.");
  }
  else
  {
    ROS_ERROR("[AssembleBackForthActionServer::goalCallback] the name are not in the arm list.");
    return;
  }
}

bool AssembleBackForthActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;
  if (!as_.isActive())
    return false;
  if (mu_.find(arm_name_) != mu_.end())
  {
    computeArm(time, *mu_[arm_name_]);
    return true;
  }
  else
  {
    ROS_ERROR("[AssembleBackForthActionServer::goalCallback] the name are not in the arm list.");
    return false;
  }
}

bool AssembleBackForthActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
    return false;

  Eigen::Vector3d f_star, m_star, force, keepPose;
  Eigen::Matrix<double, 6, 1> f_star_zero, f_ext;
  auto &current_ = arm.transform_;
  auto &jacobian = arm.jacobian_;
  auto &xd = arm.xd_;
  int cnt_start = 400;
  int cnt_max = 500;
  double run_time;
  f_ext = arm.f_ext_;

  if(is_mode_changed_)
  {
    motion_start_time_ = time.toSec();
    origin_ = arm.transform_;
    is_mode_changed_ = false;
  }
  if (count_ < cnt_max)
  {
    f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 5000, 200, 200, 5).head<3>();
    m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 5000, 200, 200, 5).tail<3>();
    if (count_ > cnt_start)
      PegInHole::getCompensationWrench(accumulated_wrench_, f_ext, cnt_start, count_, cnt_max);
    if(count_ == cnt_max - 1)
    {
      heavy_mass_ = Criteria::holdHeavyMass(accumulated_wrench_.head<3>(), 8.0);
      is_mode_changed_ = true;
#ifdef TEST_PRINT
      std::cout<<"arm_name: "<<arm.arm_name_<<std::endl;
      std::cout<<"is_heavy_mass: "<<heavy_mass_<<std::endl;
      std::cout<<"accumulated_wrench: "<<accumulated_wrench_.transpose()<<std::endl;
#endif
    }
  }

  else
  {
    switch (move_state_)
    {
      case EXEC :
        run_time = time.toSec() - motion_start_time_;
        force(axis_) = f_ext.head<3>()(axis_) - accumulated_wrench_.head<3>()(axis_);
        f_star.setZero();
        f_star = PegInHole::generateSpiralEE(origin_, current_, xd, pitch_, linear_vel_, 0.4, T_7A_, time.toSec(), motion_start_time_, motion_start_time_+duration_, 5.0, false);
        if (axis_ == 0){f_star(1)=0.0;}
        else if (axis_ == 1){f_star(0)=0.0;}
        f_star = T_WA_.linear() * f_star;
        keepPose = PegInHole::keepCurrentPose(origin_, current_, xd, 5000, 200, 200, 5).head<3>();
        keepPose(axis_) = 0.0;
        f_star += keepPose;
        m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 200, 5);

        if (run_time > 0.1 && abs(force(axis_)) >= abs(contact_force_))
        {
          std::cout << "\n!!CHECK CONTATCT!!\n" << std::endl;
          std::cout<<"arm_name: "<<arm.arm_name_<<std::endl;
          std::cout<<"force: "<<force(axis_)<<std::endl;
          setSucceeded();
        }
        else if (timeOut(time.toSec(), motion_start_time_, duration_ + 3.0))
        {
          std::cout << "\n!!!!!TIME OUT!!!!!\n" << std::endl;
          setAborted();
        }
        
        break;
    }
    
#ifdef TEST_PRINT
    if (count_ % 200 == 1)
    {
      // std::cout<<"\narm_name: "<<arm.arm_name_<<" // run_time: "<<run_time<<std::endl;
      // std::cout<<"[f_star] [m_star]: "<<f_star.transpose()<<m_star.transpose()<<std::endl;
      // std::cout<<"force: "<<force(axis_)<<std::endl;
    }
#endif
    if(heavy_mass_)
    {
      f_star += accumulated_wrench_.head<3>();
      m_star += 1.2 * accumulated_wrench_.tail<3>();
    }
  }
  count_++;

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);
  return true;
}

void AssembleBackForthActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  result_.is_completed = false;
  control_running_ = false;
  as_.setPreempted(result_);
}
void AssembleBackForthActionServer::setSucceeded()
{
  ROS_INFO("[%s] Succeeded", action_name_.c_str());
  result_.is_completed = true;
  control_running_ = false;
  as_.setSucceeded(result_);
}
void AssembleBackForthActionServer::setAborted()
{
  ROS_INFO("[%s] Aborted", action_name_.c_str());
  result_.is_completed = false;
  control_running_ = false;
  as_.setAborted(result_);
}