#include <assembly_dual_controllers/servers/assemble_triple_move_action_server.h>

AssembleTripleMoveActionServer::AssembleTripleMoveActionServer(std::string name, ros::NodeHandle &nh,
                                                               std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleTripleMoveActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleTripleMoveActionServer::preemptCallback, this));
  as_.start();
}

void AssembleTripleMoveActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if (mu_.find("panda_left") != mu_.end() && mu_.find("panda_right") != mu_.end())
  {
    ROS_INFO("AssembleDualApproachAction goal has been received.");
  }
  else
  {
    ROS_ERROR("[AssembleTripleMoveActionServer::goalCallback] the name are not in the arm list.");
    return;
  }

  left_arm_origin_ = mu_["panda_left"]->transform_;
  right_arm_origin_ = mu_["panda_right"]->transform_;
  // top_arm_origin_ = mu_["panda_top"]->transform_;

  mu_["panda_left"]->task_start_time_ = ros::Time::now();
  mu_["panda_right"]->task_start_time_ = ros::Time::now();
  // mu_["panda_top"]->task_start_time_ = ros::Time::now();
  f_measured_.setZero();

  duration_ = goal_->duration;
  contact_force_ = goal_->contact_force;
  left_target = left_arm_origin_.translation();
  right_target = right_arm_origin_.translation();
  // top_target = top_arm_origin_.translation();

  left_target[2] += goal_->target_position.position.z;
  right_target[2] += goal_->target_position.position.z;
  // top_target[2] += goal_->target_position.position.z;

  control_running_ = true;
  count_ = 0;
  left_state_ = EXEC;
  right_state_ = EXEC;

  std::cout << "Move simultaneously using dual arm" << std::endl;
  std::cout << "left_arm_origin_: \n"
            << left_arm_origin_.matrix() << std::endl;
  std::cout << "right_arm_origin_: \n"
            << right_arm_origin_.matrix() << std::endl;
}

void AssembleTripleMoveActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleTripleMoveActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
    return false;

  if (mu_.find("panda_left") != mu_.end() && mu_.find("panda_right") != mu_.end())
  {
    if (first_)
    {
      motion_start_time_ = time.toSec();
      first_ = false;
    }
    computeArm(time, *mu_["panda_left"], left_arm_origin_, left_target, left_state_);
    computeArm(time, *mu_["panda_right"], right_arm_origin_, right_target, right_state_);
    // computeArm(time, *mu_["panda_top"], top_arm_origin_, top_target);
    if (succeed_flag == 2)
      setSucceeded();
    return true;
  }
  else
  {
    ROS_ERROR("[AssembleTripleMoveActionServer::goalCallback] the name are not in the arm list.");
  }

  return false;
}

bool AssembleTripleMoveActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm, 
                                                Eigen::Isometry3d origin, Eigen::Vector3d target_pos, MOVE_STATE state_)
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
  auto &current_ = arm.transform_;

  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Matrix<double, 6, 1> f_star_zero, f_ext;
  double run_time;
  Eigen::Vector3d force;
  f_ext = arm.f_ext_;  

  int cnt_max = 500;
  int cnt_start = 400;
  if (count_ < cnt_max)
  {
    f_star = PegInHole::keepCurrentPose(origin, current_, xd, 5000, 200, 200, 5).head<3>(); //w.r.t {W}
    m_star = PegInHole::keepCurrentPose(origin, current_, xd, 5000, 200, 200, 5).tail<3>();
    if (count_ == 0)
    {
      std::cout << "command force: " << f_star.transpose() << std::endl;
      std::cout << "command moment: " << m_star.transpose() << std::endl;
    }
    if (count_ > cnt_start)
      PegInHole::getCompensationWrench(accumulated_wrench_, f_ext, cnt_start, count_, cnt_max);
    count_++;
    if (count_ >= cnt_max)
    {
      count_ = cnt_max;
      std::cout << "output for compensation: " << accumulated_wrench_.transpose() << std::endl;
      wrench_rtn_.force.x = accumulated_wrench_(0);
      wrench_rtn_.force.y = accumulated_wrench_(1);
      wrench_rtn_.force.z = accumulated_wrench_(2);
      wrench_rtn_.torque.x = accumulated_wrench_(3);
      wrench_rtn_.torque.y = accumulated_wrench_(4);
      wrench_rtn_.torque.z = accumulated_wrench_(5);
      // result_.compensation = wrench_rtn_;
    }
  }

  else
  {
    switch (state_)
    {
      case EXEC :
        run_time = time.toSec() - motion_start_time_;
        force = f_ext.head<3>() - accumulated_wrench_.head<3>();
        if (run_time > 0.05 && Criteria::checkContact(force, contact_force_))
        {
          // std::cout<<"running time: "<< run_time<<std::endl;
          std::cout << "CHECK CONTATCT!!!!!" << std::endl;
          succeed_flag++;
          state_ = KEEPCURRENT;
        }
        f_star = PegInHole::threeDofMove(origin, current_, target_pos, xd, time.toSec(), motion_start_time_, duration_);
        m_star = PegInHole::keepCurrentOrientation(origin, current_, xd, 200, 5);
        break;

      case KEEPCURRENT:
        f_star = PegInHole::keepCurrentPose(origin, current_, xd, 5000, 200, 200, 5).head<3>();
        m_star = PegInHole::keepCurrentPose(origin, current_, xd, 5000, 200, 200, 5).tail<3>();
        break;
    }
  }
    
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  // std::cout << "target: " << target_pos.transpose() << std::endl;
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  return true;
}

void AssembleTripleMoveActionServer::setSucceeded()
{
  result_.is_completed = true;
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleTripleMoveActionServer::setAborted()
{
  result_.is_completed = false;
  as_.setAborted();
  control_running_ = false;
}