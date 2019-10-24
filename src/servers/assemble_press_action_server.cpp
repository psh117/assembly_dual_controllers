#include <assembly_dual_controllers/servers/assemble_press_action_server.h>

AssemblePressActionServer::AssemblePressActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssemblePressActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssemblePressActionServer::preemptCallback, this));
  as_.start();
}

void AssemblePressActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssemblePress goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssemblePressActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  
  f_measured_.setZero();
  desired_xd_.setZero();

  origin_ = mu_[goal_->arm_name]->position_;  
  init_rot_ = mu_[goal_->arm_name]->rotation_;
  
  target_force_(0) = goal_ ->target_force.force.x;
  target_force_(1) = goal_ ->target_force.force.y;
  target_force_(2) = goal_ ->target_force.force.z;
  
  target_torque_(0) = goal_ ->target_force.torque.x;
  target_torque_(1) = goal_ ->target_force.torque.y;
  target_torque_(2) = goal_ ->target_force.torque.z; 


  assemble_dir_ = goal_ ->assemble_dir;
}

void AssemblePressActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool AssemblePressActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssemblePressActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssemblePressActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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

  if(arm.task_start_time_.toSec() + 3.0 <= time.toSec()) //after pushing for 3 seconds
  {
    std::cout<<"PRESS IS FINISHED"<<std::endl;
    as_.setSucceeded();    
  }

  f_star = keepCurrentState(origin_, init_rot_, position, rotation, xd, 5000, 100).head<3>();
  m_star = keepCurrentState(origin_, init_rot_, position, rotation, xd, 5000, 100).tail<3>();
      
  

  f_star(assemble_dir_) = target_force_(assemble_dir_); 

  

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  return true;
}