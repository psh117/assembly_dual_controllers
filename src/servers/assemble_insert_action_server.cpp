#include <assembly_dual_controllers/servers/assemble_insert_action_server.h>

AssembleInsertActionServer::AssembleInsertActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleInsertActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleInsertActionServer::preemptCallback, this));
  as_.start();
}

void AssembleInsertActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleInsert goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleInsertActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  
  f_measured_.setZero();
  desired_xd_.setZero();

  init_pos_ = mu_[goal_->arm_name]->position_;  
  init_rot_ = mu_[goal_->arm_name]->rotation_;

  origin_.linear() = init_rot_;
  origin_.translation() = init_pos_;
  
  duration_ = goal_->duration;
  insertion_force_ = goal_->insertion_force;
  wiggle_motion_ = goal_->wiggle_motion;

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

  wiggle_angle_ = 5.0*DEG2RAD;
  wiggle_angular_vel_ = 4*M_PI; // 2pi / s

  if(wiggle_motion_) std::cout<<"wiggle motion is activated"<<std::endl;
  else  std::cout<<"wiggle motion is not activated"<<std::endl;
  std::cout<<"initial_pos at insertion stage: "<< origin_.translation().transpose()<<std::endl;

  control_running_ = true;
}

void AssembleInsertActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleInsertActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleInsertActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleInsertActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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
  
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Matrix<double, 6, 1> f_star_zero;

  current_.linear() = rotation;
  current_.translation() = position;
  
  if(timeOut(time.toSec(), arm.task_start_time_.toSec(), duration_))
  {
    std::cout<<"INSERTION IS DONE"<<std::endl;
    setSucceeded();
  }

  f_star = PegInHole::pressEE(origin_, current_, xd, insertion_force_, T_EA_);
  f_star = T_WA_.linear()*f_star;

  if(wiggle_motion_)
  {
    m_star = PegInHole::generateWiggleMotionEE(origin_, current_, T_EA_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec());
    m_star = T_WA_.linear()*m_star;
  } 
  else  m_star = keepCurrentOrientation(init_rot_, rotation, xd, 200, 5);

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;

  // f_star_zero.setZero();
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);
  
  return true;
}

void AssembleInsertActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleInsertActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}