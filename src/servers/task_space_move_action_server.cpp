#include <assembly_dual_controllers/servers/task_space_move_action_server.h>

TaskSpaceMoveActionServer::TaskSpaceMoveActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
  as_.registerGoalCallback(boost::bind(&TaskSpaceMoveActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&TaskSpaceMoveActionServer::preemptCallback, this));
  as_.start();
}

void TaskSpaceMoveActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  ROS_INFO("[TaskSpaceMoveActionServer::goalCallback] Joint trajectory goal has been received.");
  Eigen::Isometry3d target_relative_pose;
  tf::poseMsgToEigen(goal_->target_pose, target_relative_pose);

  if (target_relative_pose.translation().norm() > 0.1)
  {
    ROS_WARN("[TaskSpaceMoveActionServer::goalCallback] DO NOT USE THIS ACTION with over 10 cm movement goal. Just passing it.");
    as_.setAborted();
    return; 
  }
  if (Eigen::Quaterniond(target_relative_pose.linear()).angularDistance(Eigen::Quaterniond::Identity()) > 1.560796)
  {
    ROS_WARN("[TaskSpaceMoveActionServer::goalCallback] DO NOT USE THIS ACTION with over 90 degree rotation goal. Just passing it.");
    as_.setAborted();
    return; 
  }

  start_time_ = ros::Time::now();

  bool find_arm = false;
  for (auto iter = mu_.begin(); iter != mu_.end(); iter++)
  {
    // Find 'panda_left' in 'panda_left_joint1' 
    // ROS_INFO("%s ",goal_->trajectory.joint_names[0].c_str());
    // ROS_INFO("%s ",iter->first.c_str());
    // ROS_INFO("%d ", goal_->trajectory.joint_names[0].find(iter->first));
    if (goal_->arm_name == iter->first)
    {
      ROS_INFO("[TaskSpaceMoveActionServer::goalCallback] target arm: %s.", iter->first.c_str());
      active_arm_ = iter->first;
      q_desired_ = mu_[active_arm_]->initial_q_;
      qd_desired_.setZero(q_desired_.size());
      qdd_desired_.setZero(q_desired_.size());
      target_pose_ = mu_[active_arm_]->initial_transform_ * target_relative_pose;
      control_running_ = true;
      find_arm = true;
    }
  }
  // trajectory_data_.resize(goal_->trajectory.joint_names.size(), goal_->trajectory.points.size());

  if (find_arm == false)
  {
    ROS_ERROR("[TaskSpaceMoveActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    as_.setAborted();
    return;
  }
}

void TaskSpaceMoveActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool TaskSpaceMoveActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  if(!control_running_)  // wait for acceptance of the goal, active but not accepted
    return false; 
  
  computeArm(time, *mu_[active_arm_]);
  return false;
}


bool TaskSpaceMoveActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  Eigen::Matrix<double, 7, 1> desired_torque;
  ros::Duration passed_time = time - start_time_;

  Eigen::Vector3d position_now;

  bool value_updated = false;

  Eigen::Vector3d p, p_dot, p_ddot;
  for(int i=0; i<3; i++)
  {
    auto result = dyros_math::quinticSpline(passed_time.toSec(), 0, goal_->execution_time,  
          arm.initial_transform_.translation()(i), 0, 0, 
          target_pose_.translation()(i), 0, 0);

    p(i) = result(0);
    p_dot(i) = result(1);
    p_ddot(i) = result(2);
  }

  double kp = 1000, kv = 20;
  double kp_o = 4000, kv_o = 30;

  Eigen::Matrix3d desired_rotation;
  Eigen::Vector3d r_error, omega, omega_dot;
  dyros_math::rotationQuinticZero(passed_time.toSec(), 0, goal_->execution_time, arm.rotation_, target_pose_.linear(), desired_rotation, omega, omega_dot);

  r_error = - 0.5 * dyros_math::getPhi(arm.rotation_, desired_rotation);
  // std::cout << "p: " << p.transpose() << std::endl
  //           << "arm.position_: " << arm.position_.transpose() << std::endl
  //           << "r_error: " << r_error.transpose() << std::endl
  //           << "arm.xd_: " << arm.xd_.transpose() << std::endl
  //           << "omega: " << omega.transpose() << std::endl
  //           << "omega_dot: " << omega_dot.transpose() << std::endl;
  Eigen::Vector3d f_star = p_ddot + kp * (p - arm.position_) + kv * (p_dot - arm.xd_.head<3>());
  Eigen::Vector3d m_star = omega_dot + kp_o * (r_error) + kv_o * (omega - arm.xd_.tail<3>());

  // std::cout << "f_star: " << f_star.transpose() << std::endl
  //           << "m_star: " << m_star.transpose() << std::endl;
  
  Eigen::Vector6d fm_star;
  fm_star << f_star, m_star;

  desired_torque = arm.jacobian_.transpose() * arm.modified_lambda_matrix_ * fm_star;

  if(time.toSec() > (start_time_.toSec() +  goal_->execution_time + 0.5))
  {
    ROS_INFO("[TaskSpaceMoveActionServer::goalCallback] arriving to the goal");
    std::cout << "position error: " << (target_pose_.translation() -  arm.position_).transpose() << std::endl;
    std::cout << "rotational error: " << r_error.transpose() << std::endl;
    as_.setSucceeded();
    control_running_ = false;
    return true;
  }

  arm.setTorque(desired_torque);
  return true;
}
