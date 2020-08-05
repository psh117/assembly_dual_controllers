#include <assembly_dual_controllers/servers/assemble_dual_spiral_action_server.h>

AssembleDualSpiralActionServer::AssembleDualSpiralActionServer(std::string name, ros::NodeHandle &nh,
                                                       std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleDualSpiralActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleDualSpiralActionServer::preemptCallback, this));
  as_.start();
}

void AssembleDualSpiralActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->task_arm) != mu_.end() && mu_.find(goal_->assist_arm) != mu_.end() ) 
  {
    ROS_INFO("AssembleDualSpiralAction goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleDualSpiralActionServer::goalCallback] the name %s and %s are not in the arm list.", goal_->assist_arm.c_str(), goal_ ->task_arm.c_str());
    return ;
  }

  task_arm_origin_ = mu_[goal_->task_arm]->transform_;
  assist_arm_origin_ = mu_[goal_->assist_arm]->transform_;

  lin_vel_ = goal_->linear_vel;
  pitch_ = goal_->pitch;
  mode_ = goal_->mode;
  depth_ = goal_->depth;
  friction_ = goal_->friction;
  spiral_duration_ = goal_->spiral_duration;
  pressing_force_ = goal_->pressing_force;
  range_ = goal_->range;
  twist_duration_ = goal_->twist_duration;
  
  mu_[goal_->task_arm]->task_start_time_ = ros::Time::now();
  mu_[goal_->assist_arm]->task_start_time_ = ros::Time::now();
  mu_[goal_->task_arm]->task_end_time_ = ros::Time(mu_[goal_->task_arm]->task_start_time_.toSec() + spiral_duration_);  

  ee_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  ee_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  ee_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  ee_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  ee_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  ee_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  ee_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;

  T_EA_.linear() = ee_to_assembly_quat_.toRotationMatrix();
  T_EA_.translation() = ee_to_assembly_point_;

  T_WA_ = task_arm_origin_ * T_EA_;

  std::cout << "sprial origin: " << task_arm_origin_.translation().transpose() << std::endl;
  if (mode_ == 1)
    std::cout << "single peg in hole" << std::endl;
  if (mode_ == 2)
    std::cout << "dual peg in hole" << std::endl;

  if (save_sprial_data.is_open())
    save_sprial_data.close();
  save_sprial_data.open("save_sprial_data.txt");

  control_running_ = true;
  
  result_.spiral_origin.position.x = T_WA_.translation()(0);
  result_.spiral_origin.position.y = T_WA_.translation()(1);
  result_.spiral_origin.position.z = T_WA_.translation()(2);
  Eigen::Quaterniond temp(T_WA_.linear());
  result_.spiral_origin.orientation.x = temp.x();
  result_.spiral_origin.orientation.y = temp.y();
  result_.spiral_origin.orientation.z = temp.z();
  result_.spiral_origin.orientation.w = temp.w();

  std::cout<<"pressing_force: "<<pressing_force_<<std::endl;
  std::cout<<"spiral pitch : "<<pitch_<<std::endl;
  std::cout<<"save the spiral origin: \n"<<result_.spiral_origin<<std::endl;
}

void AssembleDualSpiralActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleDualSpiralActionServer::compute(ros::Time time)
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
    ROS_ERROR("[AssembleSideChairActionServer::compute] the name %s and %s are not in the arm list.", goal_->assist_arm.c_str(), goal_ ->task_arm.c_str());
  }
  
  
  return false;
}

bool AssembleDualSpiralActionServer::computeTaskArm(ros::Time time, FrankaModelUpdater &arm)
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
  task_arm_current_ = arm.transform_;

  if (timeOut(time.toSec(), arm.task_start_time_.toSec(), spiral_duration_)) //duration wrong??
  {
    std::cout << "Time out" << std::endl;
    setAborted();
  }

  f_star = PegInHole::generateSpiralEE(task_arm_origin_, task_arm_current_, xd, pitch_, lin_vel_, pressing_force_, T_EA_, time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec());
  f_star = T_WA_.linear() * f_star;

  //signle peg in hole
  if (mode_ == 1)
    m_star = PegInHole::keepCurrentOrientation(task_arm_origin_, task_arm_current_, xd, 250., 5.0);

  //dual peg in hole
  if (mode_ == 2)
  {
    m_star = PegInHole::generateTwistSearchMotionEE(task_arm_origin_, task_arm_current_, T_EA_, xd, range_, time.toSec(), arm.task_start_time_.toSec(), twist_duration_);
    m_star = T_WA_.linear() * m_star;
  }

  // f_star.setZero();
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  // save_sprial_data << f_lpf.transpose() << std::endl;

  return true;
}

bool AssembleDualSpiralActionServer::computeAssistArm(ros::Time time, FrankaModelUpdater &arm)
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
  Eigen::Vector6d f_star_zero, f_lpf;
  Eigen::Isometry3d T_AA; // transform of assembly point wrt assist arm
  Eigen::Vector3d reaction_force;
  double reaction_force_sum;

  f_measured_ = arm.f_measured_;
  f_lpf = arm.f_measured_filtered_;
  assist_arm_current_ = arm.transform_;

  f_star = PegInHole::keepCurrentPosition(assist_arm_origin_, assist_arm_current_, xd, 5000, 100);
  m_star = PegInHole::keepCurrentOrientation(assist_arm_origin_, assist_arm_current_, xd, 200, 5);

  T_AA = assist_arm_current_.inverse()*T_WA_;
  
  reaction_force = assist_arm_current_.linear().inverse()*f_lpf.head<3>();

  reaction_force_sum = sqrt(pow(reaction_force(0),2) + pow(reaction_force(1),2));
  if(time.toSec() > arm.task_start_time_.toSec() + 5.0)
  {
    if(checkForceLimit(reaction_force_sum, friction_))
    {
      setSucceeded();
      std::cout<<"HOLE IS DETECTED!"<<std::endl;
      std::cout<<"reaction_force: "<<reaction_force.transpose()<<std::endl;  
    }
  }

  
  // f_star.setZero();
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  save_sprial_data << f_lpf.transpose() << std::endl;

  return true;
}

void AssembleDualSpiralActionServer::setSucceeded()
{
  result_.is_completed = true;
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleDualSpiralActionServer::setAborted()
{
  // result_.is_completed = true;
  as_.setAborted(result_);
  control_running_ = false;
}