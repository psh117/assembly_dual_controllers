#include <assembly_dual_controllers/servers/assemble_triple_move_action_server.h>

AssembleTripleMoveActionServer::AssembleTripleMoveActionServer(std::string name, ros::NodeHandle &nh,
                                                               std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleTripleMoveActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleTripleMoveActionServer::preemptCallback, this));
  as_.start();
  hc_.insert(std::pair<std::string,std::shared_ptr<a_state_>>("panda_left",&left_states));
  hc_.insert(std::pair<std::string,std::shared_ptr<a_state_>>("panda_right",&right_states));
  hc_.insert(std::pair<std::string,std::shared_ptr<a_state_>>("panda_top",&top_states));
}

void AssembleTripleMoveActionServer::goalCallback()
{
  goal_ = as_.acceptNewGoal();

  if (mu_.find("panda_left") != mu_.end() && mu_.find("panda_right") != mu_.end())
  {
    ROS_INFO("AssembleTripleMoveAction goal has been received.");
  }
  else
  {
    ROS_ERROR("[AssembleTripleMoveActionServer::goalCallback] the name are not in the arm list.");
    return;
  }

  control_running_ = true;
  succeed_flag = 0;

  mu_["panda_left"]->task_start_time_ = ros::Time::now();
  mu_["panda_right"]->task_start_time_ = ros::Time::now();
  mu_["panda_top"]->task_start_time_ = ros::Time::now();
  asm_dir_ = Eigen::Vector3d::UnitZ();

  upper_arm_ = goal_ -> upper_arm;
  stop_arm_1_ = goal_ -> stop_arm_1;
  stop_arm_2_ = goal_ -> stop_arm_2;
  duration_ = goal_ -> duration;
  m_star_limit_ = goal_ -> m_star_limit;

  hc_["panda_left"] -> count_ = 0;
  hc_["panda_left"] -> is_mode_changed_ = true;
  hc_["panda_left"] -> move_state_ = EXEC;
  hc_["panda_left"] -> origin_ = mu_["panda_left"] -> transform_;
  hc_["panda_left"] -> accumulated_wrench_.setZero();
  hc_["panda_left"] -> contact_force_ = goal_ -> contact_force_left;
  hc_["panda_left"] -> speed_ = goal_ -> speed;

  hc_["panda_right"] -> count_ = 0;
  hc_["panda_right"] -> is_mode_changed_ = true;
  hc_["panda_right"] -> move_state_ = EXEC;
  hc_["panda_right"] -> origin_ = mu_["panda_right"] -> transform_;
  hc_["panda_right"] -> accumulated_wrench_.setZero();
  hc_["panda_right"] -> contact_force_ = goal_ -> contact_force_right;
  hc_["panda_right"] -> speed_ = goal_ -> speed;

  hc_["panda_top"] -> count_ = 0;
  hc_["panda_top"] -> is_mode_changed_ = true;
  hc_["panda_top"] -> move_state_ = EXEC;
  hc_["panda_top"] -> origin_ = mu_["panda_top"] -> transform_;
  hc_["panda_top"] -> accumulated_wrench_.setZero();
  hc_["panda_top"] -> contact_force_ = goal_ -> contact_force_top;
  hc_["panda_top"] -> speed_ = goal_ -> speed;

  if (stop_arm_1_.empty() == false)
  {
    succeed_flag++;
    hc_[stop_arm_1_] -> move_state_ = KEEPSTOP;
  }
  if (stop_arm_2_.empty() == false)
  {
    succeed_flag++;
    hc_[stop_arm_2_] -> move_state_ = KEEPSTOP;
  }

#ifdef TEST_PRINT
  std::cout << "TRIPLE MOVE ACTION" << std::endl;
  std::cout << "duration: " << duration_ << std::endl;
  std::cout << "target speed: " << goal_ -> speed;
#endif
  if (upper_arm_ != "")
  {
    hc_[upper_arm_] -> speed_ = goal_ -> upper_speed;
    std::cout << upper_arm_ << " is upper arm!!!" << std::endl;
    std::cout << "upper arm target speed: " << hc_[upper_arm_] -> speed_;
  }
}

bool AssembleTripleMoveActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;
  if (!as_.isActive())
    return false;

  for (auto & model : mu_)
    computeArm(time, *model.second, *hc_[model.first]);

  if (succeed_flag >= 3)
    setSucceeded();
}

bool AssembleTripleMoveActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm, a_state_ &khc)
{
  if (!as_.isActive())
    return false;

  Eigen::Vector3d f_star, m_star, force;
  Eigen::Matrix<double, 6, 1> f_star_zero, f_ext;
  auto &current_ = arm.transform_;
  auto &jacobian = arm.jacobian_;
  auto &xd = arm.xd_;
  int cnt_start = 400;
  int cnt_max = 500;
  double run_time;
  f_ext = arm.f_ext_;

  if(khc.is_mode_changed_)
  {
    khc.motion_start_time_ = time.toSec();
    khc.origin_ = arm.transform_;
    khc.is_mode_changed_ = false;
#ifdef TEST_PRINT
    std::cout<<"\nchanging motion of "<<arm.arm_name_<<std::endl;
    std::cout<<"motion_start_time: "<<khc.motion_start_time_<<std::endl;
    std::cout<<"executing motion: "<<khc.move_state_<<std::endl;
#endif
  }
  if (khc.count_ > 100000)
  {
    f_star = PegInHole::keepCurrentPosition(khc.origin_, current_, xd, 1000, 15);
    m_star = PegInHole::keepCurrentOrientation(khc.origin_, current_, xd, 2000, 15);
    if (khc.count_ > cnt_start)
      PegInHole::getCompensationWrench(khc.accumulated_wrench_, f_ext, cnt_start, khc.count_, cnt_max);
    if(khc.count_ == cnt_max - 1)
    {
      khc.heavy_mass_ = Criteria::holdHeavyMass(khc.accumulated_wrench_.head<3>(), 8.0);
      khc.is_mode_changed_ = true;
      khc.origin_ = arm.transform_;
#ifdef TEST_PRINT
      std::cout<<"\n!STARTING TRIPLE MOVE!"<<std::endl;
      std::cout<<"arm_name: "<<arm.arm_name_<<std::endl;
      std::cout<<"is_heavy_mass: "<<khc.heavy_mass_<<std::endl;
      std::cout<<"accumulated_wrench: "<<khc.accumulated_wrench_.transpose()<<std::endl;
#endif
    }
  }

  else
  {
    Eigen::Matrix3d K_p, K_v;
    Eigen::Vector3d p_desired, v_desired;
    run_time = time.toSec() - khc.motion_start_time_;
    force = f_ext.head<3>() - khc.accumulated_wrench_.head<3>();
    switch (khc.move_state_)
    {
      case EXEC :
        K_p = Eigen::Matrix3d::Identity(); K_v = Eigen::Matrix3d::Identity();

        K_p(0,0) = 600.; K_p(1,1) = 600.; K_p(2,2) = 800.;
        K_v(0,0) = 15.; K_v(1,1) = 15.; K_v(2,2) = 20.; 

        v_desired = khc.speed_ * asm_dir_;
        p_desired = khc.origin_.translation() + khc.speed_ * (time.toSec() - khc.motion_start_time_) * asm_dir_;

        f_star = K_p * (p_desired - current_.translation()) + K_v * (v_desired - xd.head<3>());
        m_star = PegInHole::keepCurrentOrientation(khc.origin_, current_, xd, 1000, 15);

        if (run_time > 1.5 && Criteria::checkContact(force, Eigen::Isometry3d::Identity(), khc.contact_force_))
        {
          std::cout << "\n!!CHECK CONTATCT IN Z DIRECTION!!\n" << std::endl;
          std::cout<<"arm_name: "<<arm.arm_name_<<std::endl;
          std::cout<<"force(2): "<<force(2)<<std::endl;

          khc.move_state_ = KEEPCURRENT;
          succeed_flag++;
          khc.is_mode_changed_ = true;
        }

        else if (run_time > 1.5 && m_star.norm() > m_star_limit_)
        {
          std::cout << "\n!!CHECK MSTAR LIMIT!!\n" << std::endl;
          std::cout<<"arm_name: "<<arm.arm_name_<<std::endl;
          std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
          std::cout<<"m_star norm: "<<m_star.norm() <<std::endl;

          khc.move_state_ = KEEPCURRENT;
          succeed_flag++;
          khc.is_mode_changed_ = true;
        }

        else if (timeOut(time.toSec(), khc.motion_start_time_, duration_ + 3.0))
        {
          std::cout << "\n!!!!!TIME OUT!!!!!\n" << std::endl;
          khc.move_state_ = KEEPCURRENT;
          succeed_flag++;
          setAborted();
        }

        break;

      case KEEPCURRENT:
        f_star = PegInHole::keepCurrentPosition(khc.origin_, current_, xd, 600, 15);
        // f_star = PegInHole::keepCurrentPosition(khc.origin_, current_, xd, 300, 15);
        break;

      case KEEPSTOP:
        f_star = PegInHole::keepCurrentPosition(khc.origin_, current_, xd, 800, 15);
        m_star = PegInHole::keepCurrentOrientation(khc.origin_, current_, xd, 2000, 15);
        break;
    }
#ifdef TEST_PRINT
    if (khc.count_ % 1000 == 1)
    {
      if(arm.arm_name_=="panda_left"){std::cout<<"\n";}
      std::cout<<"\narm_name: "<<arm.arm_name_<<" // run_time: "<<run_time<<std::endl;
      std::cout<<"[f_star] [m_star]: "<<f_star.transpose()<<" "<<m_star.transpose()<<std::endl;
      std::cout<<"force: "<<force.transpose()<<std::endl;
      std::cout<<"m_star.norm(): "<<m_star.norm()<<std::endl;
      // std::cout<<"distance: "<<(khc.origin_.translation() - current_.translation()).norm()<<std::endl;
      // std::cout<<"speed: "<<xd.head<3>().norm()<<std::endl;
    }
#endif
    // if(khc.heavy_mass_)
    // {
    //   f_star += khc.accumulated_wrench_.head<3>();
    //   m_star += 1.0 * khc.accumulated_wrench_.tail<3>();
    // }
  }
  khc.count_++;
  
  contact_force << arm.f_ext_.transpose()<<std::endl;

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_ * f_star_zero;
  arm.setTorque(desired_torque);
  return true;
}

void AssembleTripleMoveActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  result_.is_completed = false;
  control_running_ = false;
  as_.setPreempted(result_);
}
void AssembleTripleMoveActionServer::setSucceeded()
{
  ROS_INFO("[%s] Succeeded", action_name_.c_str());
  result_.is_completed = true;
  control_running_ = false;
  as_.setSucceeded(result_);
}
void AssembleTripleMoveActionServer::setAborted()
{
  ROS_INFO("[%s] Aborted", action_name_.c_str());
  result_.is_completed = false;
  control_running_ = false;
  as_.setAborted(result_);
}