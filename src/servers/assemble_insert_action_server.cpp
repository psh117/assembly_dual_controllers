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

  origin_ = mu_[goal_->arm_name]->position_;  
  init_rot_ = mu_[goal_->arm_name]->rotation_;
  
  target_force_(0) = goal_ ->target_force.force.x;
  target_force_(1) = goal_ ->target_force.force.y;
  target_force_(2) = goal_ ->target_force.force.z;
  
  target_torque_(0) = goal_ ->target_force.torque.x;
  target_torque_(1) = goal_ ->target_force.torque.y;
  target_torque_(2) = goal_ ->target_force.torque.z; 


  target_time_ = goal_->time;
  if(target_time_ <= 0.01)
    target_time_ = 3.0;

  task_p_gain_ = goal_->p_gain;
  if(task_p_gain_ <= 0.01)
  {
    task_p_gain_ = 500.;
  }

  rot_p_gain_ = goal_->rot_p_gain;
  rot_d_gain_ = goal_->rot_d_gain;

  assemble_dir_ = goal_ ->assemble_dir;

  rot_compliant_mode_ = goal_->rot_compliant_mode;

}

void AssembleInsertActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
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
  
  if(arm.task_start_time_.toSec() + target_time_ <= time.toSec()) //after pushing for 3 seconds
  {
    std::cout<<"INSERTION IS FINISHED"<<std::endl;
    as_.setSucceeded();    
  }

  f_star = task_p_gain_ * (origin_ - position);
  f_star(assemble_dir_) = target_force_(assemble_dir_); 
    
  // m_star.setZero();
  m_star = keepCurrentOrientation(init_rot_, rotation, xd, rot_p_gain_, rot_d_gain_);
    

  if(rot_compliant_mode_)
  {
    for(size_t i = 0; i < 3; i++)
    {
      if(i != assemble_dir_) m_star(i) = 0.0;
    }
  }  

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);
  
  return true;
}