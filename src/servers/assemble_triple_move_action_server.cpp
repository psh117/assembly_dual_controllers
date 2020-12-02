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

  dir_ <<  goal_ -> direction.x, goal_ -> direction.y, goal_ -> direction.z;
  upper_arm_ = goal_ -> upper_arm;
  stop_arm_1_ = goal_ -> stop_arm_1;
  stop_arm_2_ = goal_ -> stop_arm_2;
  duration_ = goal_ -> duration;
  is_test_ = goal_ -> is_test;
  upper_more_ = goal_ -> upper_more;
  stop_speed_ = goal_ -> stop_speed;
  max_force = goal_ -> force_limit;

  hc_["panda_left"] -> count_ = 0;
  hc_["panda_left"] -> wait_ = 0;
  hc_["panda_left"] -> is_upper_arm_ = false;
  hc_["panda_left"] -> is_mode_changed_ = true;
  hc_["panda_left"] -> move_state_ = EXEC;
  hc_["panda_left"] -> origin_ = mu_["panda_left"] -> transform_;
  hc_["panda_left"] -> target_ = hc_["panda_left"] -> origin_.translation() + dir_;
  hc_["panda_left"] -> accumulated_wrench_.setZero();
  hc_["panda_left"] -> contact_force_ = goal_ -> contact_force_left;

  hc_["panda_right"] -> count_ = 0;
  hc_["panda_right"] -> wait_ = 0;
  hc_["panda_right"] -> is_upper_arm_ = false;
  hc_["panda_right"] -> is_mode_changed_ = true;
  hc_["panda_right"] -> move_state_ = EXEC;
  hc_["panda_right"] -> origin_ = mu_["panda_right"] -> transform_;
  hc_["panda_right"] -> target_ = hc_["panda_right"] -> origin_.translation() + dir_;
  hc_["panda_right"] -> accumulated_wrench_.setZero();
  hc_["panda_right"] -> contact_force_ = goal_ -> contact_force_right;

  top_arm_rot_.setIdentity();
  top_arm_rot_(0,0) = -1.0;
  top_arm_rot_(1,1) = -1.0;

  hc_["panda_top"] -> count_ = 0;
  hc_["panda_top"] -> wait_ = 0;
  hc_["panda_top"] -> is_upper_arm_ = false;
  hc_["panda_top"] -> is_mode_changed_ = true;
  hc_["panda_top"] -> move_state_ = EXEC;
  hc_["panda_top"] -> origin_ = mu_["panda_top"] -> transform_;
  hc_["panda_top"] -> target_ = hc_["panda_top"] -> origin_.translation() + top_arm_rot_ *dir_;
  hc_["panda_top"] -> accumulated_wrench_.setZero();
  hc_["panda_top"] -> contact_force_ = goal_ -> contact_force_top;

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

  if (is_test_)
  {
    std::cout<<"TESTING TRIPLE MOVE ACTION"<<std::endl;
    hc_["panda_left"] -> move_state_ = KEEPSTOP;
    hc_["panda_right"] -> move_state_ = KEEPSTOP;
    hc_["panda_top"] -> move_state_ = KEEPSTOP;
    succeed_flag = 2;
  }

#ifdef TEST_PRINT
  std::cout<<"TRIPLE MOVE ACTION"<<std::endl;
  std::cout<<"duration: "<<duration_<<std::endl;
  std::cout<<"l&r target direction: "<<dir_.transpose()<<std::endl;
  std::cout<<"top target direction: "<<(top_arm_rot_*dir_).transpose()<<std::endl;
#endif

  if (upper_arm_ != "")
  {
    hc_[upper_arm_] -> is_upper_arm_ = true;
    if (upper_arm_ == "panda_top") {hc_[upper_arm_] -> target_ += top_arm_rot_ * dir_ * upper_more_;}
    else {hc_[upper_arm_] -> target_ += dir_;}
    std::cout<<upper_arm_<<" is upper arm!!!"<<std::endl;
    std::cout<<"target direction: "<<(hc_[upper_arm_] -> origin_.translation() - hc_[upper_arm_] -> target_).transpose() <<std::endl;
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
  if (khc.count_ < cnt_max)
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
    run_time = time.toSec() - khc.motion_start_time_;
    force = f_ext.head<3>() - khc.accumulated_wrench_.head<3>();
    switch (khc.move_state_)
    {
      case EXEC :
        f_star = PegInHole::threeDofMove(khc.origin_, current_, khc.target_, xd, time.toSec(), khc.motion_start_time_, duration_, 500, 20);
        // m_star = PegInHole::keepCurrentOrientation(khc.origin_, current_, xd, 1200, 15);

        if (run_time > 0.05 && Criteria::checkContact(force, Eigen::Isometry3d::Identity(), khc.contact_force_))
        {
          std::cout << "\n!!CHECK CONTATCT IN Z DIRECTION!!\n" << std::endl;
          std::cout<<"arm_name: "<<arm.arm_name_<<std::endl;
          std::cout<<"force(2): "<<force(2)<<std::endl;

          khc.move_state_ = KEEPCURRENT;
          succeed_flag++;
          khc.is_mode_changed_ = true;
        }
        else if (run_time > (duration_*0.7) && xd.head<3>().norm() < stop_speed_)
        {
          khc.wait_++;
          if (khc.wait_ > 1000)
          {
            std::cout << "\n!SPEED ZERO!\n" << std::endl;
            std::cout<<"arm_name: "<<arm.arm_name_<<std::endl;
            std::cout<<"speed: "<<xd.head<3>().norm()<<std::endl;
            
            khc.move_state_ = KEEPCURRENT;
            succeed_flag++;
            khc.is_mode_changed_ = true;
          }
        }

        // else if (run_time > 0.5 && (khc.target_ - current_.translation()).norm() < 0.00)
        // {
        //   std::cout << "\n!DISTANCE REACHED!\n" << std::endl;
        //   std::cout<<"arm_name: "<<arm.arm_name_<<std::endl;
        //   std::cout<<"speed: "<<xd.head<3>().norm()<<std::endl;
          
        //   khc.move_state_ = KEEPCURRENT;
        //   succeed_flag++;
        //   khc.is_mode_changed_ = true;
        // }

        else if (timeOut(time.toSec(), khc.motion_start_time_, duration_ + 3.0))
        {
          std::cout << "\n!!!!!TIME OUT!!!!!\n" << std::endl;
          khc.move_state_ = KEEPCURRENT;
          succeed_flag++;
          setAborted();
        }

        else
        {
          khc.wait_ = 0;
        }
        break;

      case KEEPCURRENT:
        f_star = PegInHole::keepCurrentPosition(khc.origin_, current_, xd, 1000, 15);
        break;

      case KEEPSTOP:
        f_star = PegInHole::keepCurrentPosition(khc.origin_, current_, xd, 1000, 15);
        m_star = PegInHole::keepCurrentOrientation(khc.origin_, current_, xd, 2000, 15);
        break;
    }
#ifdef TEST_PRINT
    if (khc.count_ % 500 == 1)
    {
      if(arm.arm_name_=="panda_left"){std::cout<<"\n";}
      std::cout<<"\narm_name: "<<arm.arm_name_<<" // run_time: "<<run_time<<std::endl;
      std::cout<<"[f_star] [m_star]: "<<f_star.transpose()<<m_star.transpose()<<std::endl;
      std::cout<<"force: "<<force.transpose()<<std::endl;
      std::cout<<"distance: "<<(khc.origin_.translation() - current_.translation()).norm()<<std::endl;
      std::cout<<"speed: "<<xd.head<3>().norm()<<std::endl;
    }
#endif
    for (int i=0; i<3; i++){if (f_star(i) < max_force) {f_star(i) = max_force;}}
    if(khc.heavy_mass_)
    {
      f_star += khc.accumulated_wrench_.head<3>();
      m_star += 1.0 * khc.accumulated_wrench_.tail<3>();
    }
  }
  khc.count_++;

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