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

  if (mu_.find("panda_left") != mu_.end() && mu_.find("panda_top") != mu_.end() && mu_.find("panada_right") != mu_.end())
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
  top_arm_origin_ = mu_["panda_top"]->transform_;

  mu_["panda_left"]->task_start_time_ = ros::Time::now();
  mu_["panda_right"]->task_start_time_ = ros::Time::now();
  mu_["panda_top"]->task_start_time_ = ros::Time::now();
  f_measured_.setZero();

  duration_ = goal_->duration;


  target_pos[0] = goal_->target_position.position.x;
  target_pos[1] = goal_->target_position.position.y;
  target_pos[2] = goal_->target_position.position.z;
  first_ = true;

  control_running_ = true;

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

  if (mu_.find("panda_left") != mu_.end() && mu_.find("panda_top") != mu_.end() && mu_.find("panada_right") != mu_.end())
  {
    computeArm(time, *mu_["panda_left"], left_arm_origin_);
    computeArm(time, *mu_["panda_right"], right_arm_origin_);
    computeArm(time, *mu_["panda_top"], top_arm_origin_);
    return true;
  }
  else
  {
    ROS_ERROR("[AssembleTripleMoveActionServer::goalCallback] the name are not in the arm list.");
  }

  return false;
}

bool AssembleTripleMoveActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm, Eigen::Isometry3d origin)
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
  Eigen::Matrix<double, 6, 1> f_star_zero;

  f_measured_ = arm.f_measured_;
  

  if (first_)
  {
    motion_start_time_ = time.toSec();
    first_ = false;
  }
  f_star = PegInHole::threeDofMove(origin, current_, target_pos, xd, time.toSec(), motion_start_time_, duration_);
  m_star = PegInHole::keepCurrentOrientation(origin, current_, xd, 200, 5);
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  return true;
}

void AssembleTripleMoveActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleTripleMoveActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}