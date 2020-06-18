#include <assembly_dual_controllers/servers/assemble_rotation_action_server.h>

AssembleRotationActionServer::AssembleRotationActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleRotationActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleRotationActionServer::preemptCallback, this));
  as_.start();
}

void AssembleRotationActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleRotation goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleRotationActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  mu_[goal_->arm_name]->task_end_time_ = ros::Time(mu_[goal_->arm_name]->task_start_time_.toSec() + 3.0);

  f_measured_.setZero();

  pressing_force_ = goal_->pressing_force; 
  range_ = goal_ ->range*M_PI/180;
  f_threshold_ = goal_ ->f_threshold;
  duration_ = goal_->duration;

  origin_ = mu_[goal_->arm_name]->transform_;  

  is_first_ = true;

  ee_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  ee_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  ee_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  ee_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  ee_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  ee_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  ee_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;
  
  T_EA_.linear() = ee_to_assembly_quat_.toRotationMatrix();
  T_EA_.translation() = ee_to_assembly_point_;

  T_WA_ = origin_*T_EA_;

  asm_dir_ = T_EA_.linear().col(2);

  if(fm_rotation_search.is_open())  fm_rotation_search.close();
  if(pr_rotation_search.is_open())  pr_rotation_search.close();

  fm_rotation_search.open("fm_rotation_search.txt");
  pr_rotation_search.open("pr_rotation_search.txt");


  control_running_ = true;
}

void AssembleRotationActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleRotationActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleRotationActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleRotationActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
      return false; 

  auto & mass = arm.mass_matrix_;
  auto & rotation = arm.rotation_;
  auto & position = arm.position_;
  auto & jacobian = arm.jacobian_;
  auto & tau_measured = arm.tau_measured_;
  auto & gravity = arm.gravity_;
  auto & xd = arm.xd_; //velocity
  
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Vector6d f_lpf;
  double dis;
  double run_time;  

  Eigen::Matrix<double, 6, 1> f_star_zero;
  
  f_measured_ = arm.f_measured_;
  f_lpf = arm.f_measured_filtered_;
  current_ = arm.transform_;

  dis = origin_.translation()(2) - current_.translation()(2);
  run_time = time.toSec() - arm.task_start_time_.toSec();

  if(is_first_ == true)
  {
    arm.task_start_time_ = time;
    is_first_ = false;
    std::cout<<"ROTATE TO SEARCH"<<std::endl;
  }

  if(abs(f_lpf(5)) >= f_threshold_)
  {
    std::cout<<"HOLE IS DETECTED"<<std::endl;
    std::cout<<"f_measured_(5): "<<f_lpf(5)<<std::endl;
    setSucceeded();
  }

  if(dis >= 0.005 && abs(f_lpf(5)) < f_threshold_)
  { 
    std::cout<<"dis: "<<dis<<std::endl;
    std::cout<<"Deviating"<<std::endl;
    setAborted();
  }

  if(timeOut(time.toSec(), arm.task_start_time_.toSec(), duration_)) //duration wrong??
  { 
    std::cout<<"No holes"<<std::endl;
    setAborted();
  } 

  // f_star = PegInHole::pressEE(origin_, current_, xd, pressing_force_, T_EA_);
  f_star = PegInHole::generateRotationSearchMotionEE(origin_, current_, T_EA_, xd, range_, time.toSec(), arm.task_start_time_.toSec(), duration_); 
  f_star = T_WA_.linear()*f_star;

  // m_star = PegInHole::generateRotationSearchMotionEE(origin_, current_, T_EA_, xd, range_, time.toSec(), arm.task_start_time_.toSec(), duration_);   
  m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 200, 5);
  // m_star = T_WA_.linear()*m_star;

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  // std::cout<<"dis: "<<dis<<std::endl;
  // std::cout<<"f_reaction: "<<f_measured_(5)<<std::endl;
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  // f_star_zero.setZero();

  arm.setTorque(desired_torque);
  
  fm_rotation_search << f_measured_.transpose() << std::endl;
  pr_rotation_search << position.transpose() << std::endl;  


  return true;
}


void AssembleRotationActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleRotationActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}
