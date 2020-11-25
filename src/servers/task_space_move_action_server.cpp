#include <assembly_dual_controllers/servers/task_space_move_action_server.h>

#define EYE(X) Eigen::Matrix<double, X, X>::Identity()

TaskSpaceMoveActionServer::TaskSpaceMoveActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
  as_.registerGoalCallback(boost::bind(&TaskSpaceMoveActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&TaskSpaceMoveActionServer::preemptCallback, this));
  as_.start();
  
  _task1.resize(2,dof);
  _task2.resize(6,dof);
}

void TaskSpaceMoveActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  ROS_INFO("[TaskSpaceMoveActionServer::goalCallback] Joint trajectory goal has been received.");
  Eigen::Isometry3d target_relative_pose;
  tf::poseMsgToEigen(goal_->target_pose, target_relative_pose);
  std::cout<<goal_->target_pose.orientation.x<<std::endl;
  std::cout<<goal_->target_pose.orientation.y<<std::endl;
  std::cout<<goal_->target_pose.orientation.z<<std::endl;
  std::cout<<goal_->target_pose.orientation.w<<std::endl;
  if (target_relative_pose.translation().norm() > 0.4)
  {
    ROS_WARN("[TaskSpaceMoveActionServer::goalCallback] DO NOT USE THIS ACTION with over 40 cm movement goal. Just passing it.");
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

  ////////////////////////////////// Task Transition /////////////////////////////////////////
  Eigen::MatrixXd A_inv_ = arm.modified_mass_matrix_.inverse();

  _task1.J.setZero();
  _task1.J(0, 0) = 1.0;
  _task1.J(1, 1) = 1.0;
  _task1.JT = _task1.J.transpose();

  _task1.lambda_inv = _task1.J * A_inv_ * _task1.JT;
  _task1.lambda = _task1.lambda_inv.inverse();

  _task1.J_barT = _task1.lambda * _task1.J * A_inv_;
  _task1.J_bar = _task1.J_barT.transpose();

  _task1.NT = EYE(dof) - _task1.JT * _task1.J_barT;
  _task1.N = _task1.NT.transpose();

  
  _task1.setActive2(-166.0/180.0*M_PI, 166.0/180.0*M_PI, arm.q_(0), true);
  _task1.setPotential_3(-166.0/180.0*M_PI, 166.0/180.0*M_PI, arm.q_(0), arm.qd_(0), true);

  _task1.setActive2(-101.0/180.0*M_PI, 101.0/180.0*M_PI, arm.q_(1), false);
  _task1.setPotential_3(-101.0/180.0*M_PI, 101.0/180.0*M_PI, arm.q_(1), arm.qd_(1), false);

  _task1.h = std::max(_task1.h1, _task1.h2);
  if (_task1.h > 0)
  {
    // std::cout << "Joint limit task" << std::endl;
  }

  _task2.J = arm.jacobian_;
  _task2.JT = _task2.J.transpose();
  _task2.lambda_inv = _task2.J * A_inv_ * _task2.JT;
  _task2.lambda = _task2.lambda_inv.inverse();
  //_limit_task.h = 0.0;
  _task2.f.head(3) = f_star;
  _task2.f.tail(3) = m_star;

  //std::cout <<  _task1.fi.transpose() << std::endl;

  _task1.fi = _task1.h * _task1.f +
              (1.0 - _task1.h) * _task1.J * A_inv_ *
                  (_task2.JT * _task2.lambda * _task2.f);
  //_task1.fi = _task1.h * _task1.f;
  //std::cout <<  _task1.fi.transpose() << std::endl;

  _task2.fi = _task2.f;

  _task1.T = _task1.JT * _task1.lambda * _task1.fi;

  Eigen::Matrix6d lambda_12;
  Eigen::Matrix6d lambda_12_inv;

  lambda_12_inv = _task2.J * _task1.N * A_inv_ * _task1.NT * _task2.JT + 0.001*EYE(6);
  lambda_12 = lambda_12_inv.inverse();

  desired_torque = _task1.T  + _task1.NT * (_task2.JT * lambda_12 * _task2.fi);

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  //std::cout << "desired_torque" <<arm.q_(0)/M_PI*180.0 << std::endl;

  // std::cout<<"desried_torque: "<< desired_torque.transpose() <<std::endl;

  //desired_torque.setZero();

  if(time.toSec() > (start_time_.toSec() +  goal_->execution_time + 0.8))
  {
    ROS_INFO("[TaskSpaceMoveActionServer::goalCallback] arriving to the goal");
    std::cout << "position error: " << (target_pose_.translation() -  arm.position_).transpose() << std::endl;
    std::cout << "rotational error: " << r_error.transpose() << std::endl;
    as_.setSucceeded();
    arm.setInitialValues(target_pose_);
    arm.idle_controlled_ = true;
    arm.setTorque(desired_torque, true);
    control_running_ = false;
    return true;
  }

  arm.setTorque(desired_torque);
  return true;
}
