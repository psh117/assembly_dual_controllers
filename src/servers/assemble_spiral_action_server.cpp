#include <assembly_dual_controllers/servers/assemble_spiral_action_server.h>

AssembleSpiralActionServer::AssembleSpiralActionServer(std::string name, ros::NodeHandle &nh,
                                                       std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleSpiralActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleSpiralActionServer::preemptCallback, this));
  as_.start();

  // spiral_data = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/spiral_data.txt","w");
}

void AssembleSpiralActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if (mu_.find(goal_->arm_name) != mu_.end())
  {
    ROS_INFO("AssembleSpiral goal has been received.");
  }
  else
  {
    ROS_ERROR("[AssembleSpiralActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return;
  }

  lin_vel_ = goal_->linear_vel;
  pitch_ = goal_->pitch;
  mode_ = goal_->mode;
  // assemble_dir_ = goal_->assemble_dir;
  depth_ = goal_->depth;
  friction_ = goal_->friction;
  spiral_duration_ = goal_->spiral_duration;
  pressing_force_ = goal_->pressing_force;
  range_ = goal_->range;
  twist_duration_ = goal_->twist_duration;

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  mu_[goal_->arm_name]->task_end_time_ = ros::Time(mu_[goal_->arm_name]->task_start_time_.toSec() + spiral_duration_);

  f_measured_.setZero();
  desired_xd_.setZero();

  init_pos_ = mu_[goal_->arm_name]->position_;
  init_rot_ = mu_[goal_->arm_name]->rotation_;

  origin_ = mu_[goal_->arm_name]->transform_;

  ee_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  ee_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  ee_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  ee_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  ee_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  ee_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  ee_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;

  T_EA_.linear() = ee_to_assembly_quat_.toRotationMatrix();
  T_EA_.translation() = ee_to_assembly_point_;

  T_WA_ = origin_ * T_EA_;

  ori_change_dir_ = 0;
  is_first_ = true;
  ori_duration_ = 0.5; //1.0

  std::cout << "sprial origin: " << init_pos_.transpose() << std::endl;
  if (mode_ == 1)
    std::cout << "Single peg in hole" << std::endl;
  if (mode_ == 2)
    std::cout << "dual peg in hole" << std::endl;

  if (save_sprial_data.is_open())
    save_sprial_data.close();
  save_sprial_data.open("save_sprial_data.txt");

  control_running_ = true;

  std::cout<<"pressing_force: "<<pressing_force_<<std::endl;
  std::cout<<"spiral pitch : "<<pitch_<<std::endl;
}

void AssembleSpiralActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleSpiralActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
    return false;

  if (mu_.find(goal_->arm_name) != mu_.end())
  {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  }
  else
  {
    ROS_ERROR("[AssembleSpiralActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}

bool AssembleSpiralActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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

  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Vector6d f_star_zero;
  Eigen::Vector6d f_lpf;

  f_measured_ = arm.f_measured_;
  f_lpf = arm.f_measured_filtered_;
  current_ = arm.transform_;

  if (timeOut(time.toSec(), arm.task_start_time_.toSec(), spiral_duration_)) //duration wrong??
  {
    std::cout << "Time out" << std::endl;
    setAborted();
  }

  f_star = PegInHole::generateSpiralEE(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_EA_, time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec());
  f_star = T_WA_.linear() * f_star;

  //signle peg in hole
  if (mode_ == 1)
    m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 250., 5.0);

  //dual peg in hole
  if (mode_ == 2)
  {
    m_star = PegInHole::generateTwistSearchMotionEE(origin_, current_, T_EA_, xd, range_, time.toSec(), arm.task_start_time_.toSec(), twist_duration_);
    m_star = T_WA_.linear() * m_star;
  }

  if (detectHole(origin_, current_, f_lpf.head<3>(), T_WA_, friction_))
  // if(detectHole(init_pos_(assemble_dir_),position(assemble_dir_),f,depth_,friction_))
  {
    std::cout << "HOLE IS DETECTED" << std::endl;
    setSucceeded();
  }
  // f_star.setZero();
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  // std::cout<<"pressing_force: "<<pressing_force_<<std::endl;
  // f_star_zero.setZero();

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  save_sprial_data << f_lpf.transpose() << std::endl;

  // Eigen::Vector3d euler;
  // euler = dyros_math::rot2Euler(current_.linear()); 
  // save_sprial_data << euler.transpose()<<std::endl;
  return true;
}

// Eigen::Vector3d AssembleSpiralActionServer::motionForDual(ros::Time time, Eigen::Matrix3d rot, Eigen::Vector3d angular)
// {

//   Eigen::Vector3d m_star;

//   if (ori_change_dir_ == 0)
//   {
//     if (is_first_ == true)
//     {
//       ori_start_time_ = time.toSec();
//       is_first_ = false;
//       // ori_init_ = desired_rotation_M;
//     }

//     m_star = generateSpiralWithRotation(init_rot_, rot, angular, time.toSec(), ori_start_time_, ori_duration_, ori_change_dir_, 2);

//     if (time.toSec() > ori_start_time_ + ori_duration_ / 2)
//     {
//       ori_change_dir_ = 1;
//       is_first_ = true;
//     }
//   }

//   if (ori_change_dir_ == 1)
//   {
//     if (is_first_ == true)
//     {
//       ori_start_time_ = time.toSec();
//       is_first_ = false;
//     }

//     m_star = generateSpiralWithRotation(init_rot_, rot, angular, time.toSec(), ori_start_time_, ori_duration_, ori_change_dir_, 2);

//     if (time.toSec() > ori_start_time_ + ori_duration_)
//     {
//       ori_change_dir_ = 2;
//       is_first_ = true;
//     }
//   }

//   if (ori_change_dir_ == 2)
//   {
//     if (is_first_ == true)
//     {
//       ori_start_time_ = time.toSec();
//       is_first_ = false;
//     }

//     m_star = generateSpiralWithRotation(init_rot_, rot, angular, time.toSec(), ori_start_time_, ori_duration_, ori_change_dir_, 2);

//     if (time.toSec() > ori_start_time_ + ori_duration_)
//     {
//       ori_change_dir_ = 1;
//       is_first_ = true;
//     }
//   }

//   return m_star;
// }

void AssembleSpiralActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleSpiralActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}