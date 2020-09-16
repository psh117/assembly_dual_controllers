#include <assembly_dual_controllers/servers/assemble_retreat_bolt_action_server.h>

AssembleRetreatBoltActionServer::AssembleRetreatBoltActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleRetreatBoltActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleRetreatBoltActionServer::preemptCallback, this));
  as_.start();
}

void AssembleRetreatBoltActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleInsert goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleRetreatBoltActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();

  origin_ = mu_[goal_->arm_name]->transform_;
  
  retreat_force_ = goal_->retreat_force;
  retreat_distance_ = goal_->retreat_distance;
  time_limit_ = goal_->time_limit;

  wiggle_motion_ = goal_->wiggle_motion;
  wiggle_motion_z_axis_ = goal_->wiggle_motion_z_axis;

  wiggle_angle_ = goal_->wiggle_angle * DEG2RAD; //radian
  
  std::cout<<goal_->arm_name<<std::endl;
  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"init_yaw_angle: "<<init_yaw_angle_<<std::endl;
  // std::cout<<"init_yaw_goal: "<<yawing_angle_<<std::endl;
  // std::cout<<"sum: "<<yawing_angle_ + init_yaw_angle_<<std::endl;
  
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

  wiggle_angular_vel_ = 4*M_PI; // 2pi / s

  if(wiggle_motion_ && wiggle_motion_z_axis_) std::cout<<"wiggle motions along all axes are activated"<<std::endl;
  else if(wiggle_motion_) std::cout<<"wiggle motion along x/y axis are activated"<<std::endl;
  else if(wiggle_motion_z_axis_) std::cout<<"wiggle motion along z axis is activated"<<std::endl;
  else  std::cout<<"no optional motions"<<std::endl;
  
  control_running_ = true;

}

void AssembleRetreatBoltActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleRetreatBoltActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleRetreatBoltActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleRetreatBoltActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
      return false; 

  // auto & mass = arm.mass_matrix_;
  // auto & rotation = arm.rotation_;
  // auto & position = arm.position_;
  auto & jacobian = arm.jacobian_;
  // auto & tau_measured = arm.tau_measured_;
  // auto & gravity = arm.gravity_;
  auto & xd = arm.xd_; //velocity

  Eigen::Vector3d f_star, m_star, m_wig, m_wig_z;
  Eigen::Vector6d f_star_zero;
  Eigen::Vector3d dp;

  current_ = arm.transform_;
  
  dp = T_WA_.inverse()*(current_*T_EA_).translation(); // displacement w.r.t {A} 
  
  if(-dp(2) > retreat_distance_) // dp(2) is always negative 
  {
    std::cout<<"Success retreat action"<<std::endl;
    setSucceeded();
    return true;
  }

  if(Criteria::timeOut(time.toSec(), arm.task_start_time_.toSec(), time_limit_))
  {
    std::cout<<"Fail retreat action"<<std::endl;
    setAborted();
    return true;
  }

  f_star = -PegInHole::pressCubicEE(origin_, current_, xd, T_WA_, retreat_force_, time.toSec(), arm.task_start_time_.toSec(), time_limit_);
  f_star = T_WA_.linear()*f_star;
  
  if(wiggle_motion_ && wiggle_motion_z_axis_)
  {
    m_wig = PegInHole::generateWiggleMotionEE(origin_, current_, T_EA_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec());
    m_wig_z = PegInHole::generateWiggleMotionEE_Zaxis(origin_, current_, T_EA_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec());
    m_star = m_wig + m_wig_z;
    m_star = T_WA_.linear()*m_star;
  }
  else if(wiggle_motion_)
  {
    m_star = PegInHole::generateWiggleMotionEE(origin_, current_, T_EA_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec());
    m_star = T_WA_.linear()*m_star;
  }
  else if(wiggle_motion_z_axis_)
  {
    m_star = PegInHole::generateWiggleMotionEE_Zaxis(origin_, current_, T_EA_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec());
    m_star = T_WA_.linear()*m_star;
  }
  else  m_star = keepCurrentOrientation(origin_, current_ , xd, 200, 5);

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  
  // f_star_zero.setZero();
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);
  
  // std::cout<<"f_star: "<< f_star.transpose()<<std::endl;
  std::cout<<"--------------"<<std::endl;
  std::cout<<"displacement: "<<dp.transpose()<<std::endl;
  return true;
}

void AssembleRetreatBoltActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleRetreatBoltActionServer::setAborted()
{
  as_.setAborted(result_);
  control_running_ = false;
}
