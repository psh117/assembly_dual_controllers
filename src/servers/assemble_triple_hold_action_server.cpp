#include <assembly_dual_controllers/servers/assemble_triple_hold_action_server.h>

AssembleTripleHoldActionServer::AssembleTripleHoldActionServer(std::string name, ros::NodeHandle &nh,
                                                               std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleTripleHoldActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleTripleHoldActionServer::preemptCallback, this));
  as_.start();
}

void AssembleTripleHoldActionServer::goalCallback()
{
  goal_ = as_.acceptNewGoal();

  if (mu_.find("panda_left") != mu_.end() && mu_.find("panda_right") != mu_.end() && mu_.find("panda_top") != mu_.end())
  {
    ROS_INFO("AssembleTripleHoldAction goal has been received.");
  }
  else
  {
    ROS_ERROR("[AssembleTripleHoldActionServer::goalCallback] the name are not in the arm list.");
    return;
  }

  mu_["panda_left"]->task_start_time_ = ros::Time::now();
  mu_["panda_right"]->task_start_time_ = ros::Time::now();
  mu_["panda_top"]->task_start_time_ = ros::Time::now();

  target_force_left_(0) = goal_ -> target_force_left.force.x;
  target_force_left_(1) = goal_ -> target_force_left.force.y;
  target_force_left_(2) = goal_ -> target_force_left.force.z;
  target_force_left_(3) = goal_ -> target_force_left.torque.x;
  target_force_left_(4) = goal_ -> target_force_left.torque.y;
  target_force_left_(5) = goal_ -> target_force_left.torque.z;

  target_force_right_(0) = goal_ -> target_force_right.force.x;
  target_force_right_(1) = goal_ -> target_force_right.force.y;
  target_force_right_(2) = goal_ -> target_force_right.force.z;
  target_force_right_(3) = goal_ -> target_force_right.torque.x;
  target_force_right_(4) = goal_ -> target_force_right.torque.y;
  target_force_right_(5) = goal_ -> target_force_right.torque.z;

  target_force_top_(0) = goal_ -> target_force_top.force.x;
  target_force_top_(1) = goal_ -> target_force_top.force.y;
  target_force_top_(2) = goal_ -> target_force_top.force.z;
  target_force_top_(3) = goal_ -> target_force_top.torque.x;
  target_force_top_(4) = goal_ -> target_force_top.torque.y;
  target_force_top_(5) = goal_ -> target_force_top.torque.z;

  // duration_ = goal_ -> duration;
  control_running_ = true;
}

bool AssembleTripleHoldActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;
  if (!as_.isActive())
    return false;

  if (mu_.find("panda_left") != mu_.end() && mu_.find("panda_right") != mu_.end() && mu_.find("panda_top") != mu_.end())
  {
    for (auto & model : mu_)
        computeArm(time, *model.second, model.first);
    return true;
  }
  else
  {
    ROS_ERROR("[AssembleTripleHoldAction::compute] arm is not in the list.");
  }
  return false;
}

bool AssembleTripleHoldActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm,const std::string &arm_name)
{
  if (!as_.isActive())
    return false;

  Eigen::Matrix<double, 6, 1> target_force;
  Eigen::Vector3d f_star, m_star, force;
  Eigen::Matrix<double, 6, 1> f_star_zero, f_ext;
  auto &current_ = arm.transform_;
  auto &jacobian = arm.jacobian_;
  auto &xd = arm.xd_;
  int cnt_start = 400;
  int cnt_max = 500;
  double run_time;
  f_ext = arm.f_ext_;
  
  if (arm_name == "panda_left")
  {
      target_force = target_force_left_;
  }
  else if (arm_name == "panda_right")
  {
      target_force = target_force_right_;
  }
  else if (arm_name == "panda_top")
  {
      target_force = target_force_top_;
  }

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_ * target_force;
  arm.setTorque(desired_torque);
  return true;
}

void AssembleTripleHoldActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  result_.is_completed = false;
  control_running_ = false;
  as_.setPreempted(result_);
}
void AssembleTripleHoldActionServer::setSucceeded()
{
  ROS_INFO("[%s] Succeeded", action_name_.c_str());
  result_.is_completed = true;
  control_running_ = false;
  as_.setSucceeded(result_);
}
void AssembleTripleHoldActionServer::setAborted()
{
  ROS_INFO("[%s] Aborted", action_name_.c_str());
  result_.is_completed = false;
  control_running_ = false;
  as_.setAborted(result_);
}