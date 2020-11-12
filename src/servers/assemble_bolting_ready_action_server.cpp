#include <assembly_dual_controllers/servers/assemble_bolting_ready_action_server.h>

AssembleBoltingReadyActionServer::AssembleBoltingReadyActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleBoltingReadyActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleBoltingReadyActionServer::preemptCallback, this));
  as_.start();
}

void AssembleBoltingReadyActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleBoltingReady goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleBoltingReadyActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  
  origin_ = mu_[goal_->arm_name]->transform_;  
  duration_ = goal_->duration;
  
  target_pos_(0) = goal_->target_pose.position.x;
  target_pos_(1) = goal_->target_pose.position.y;
  target_pos_(2) = goal_->target_pose.position.z;
  target_quat_.x() = goal_->target_pose.orientation.x;
  target_quat_.y() = goal_->target_pose.orientation.y;
  target_quat_.z() = goal_->target_pose.orientation.z;
  target_quat_.w() = goal_->target_pose.orientation.w;

  T_target_.linear() = target_quat_.toRotationMatrix();
  T_target_.translation() = target_pos_;
   
  control_running_ = true;
}

void AssembleBoltingReadyActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleBoltingReadyActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;
    
  if (!as_.isActive())
    return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleBoltingReadyActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleBoltingReadyActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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
  
  Eigen::Vector3d f_star, m_star;  
  Eigen::Vector6d f_star_zero;
  double run_time;

  current_ = arm.transform_;
  run_time = time.toSec() - arm.task_start_time_.toSec();

  if(timeOut(time.toSec(), arm.task_start_time_.toSec(), duration_ +0.01))
  {
    std::cout<<"TF : \n"<< current_.linear()<<std::endl;
    setSucceeded();
  }

  f_star = PegInHole::keepCurrentPosition(origin_, current_, xd, 500, 10);
  m_star = PegInHole::rotateWithMat(origin_, current_, xd, T_target_.linear(), time.toSec(), arm.task_start_time_.toSec(), duration_, 2000, 15);
   

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
    
  // f_star_zero.setZero();

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_*f_star_zero;
  arm.setTorque(desired_torque);
  
  return true;
}

void AssembleBoltingReadyActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleBoltingReadyActionServer::setAborted()
{
  as_.setAborted(result_);
  control_running_ = false;
}
