#include <assembly_dual_controllers/servers/assemble_approach_action_server.h>

AssembleApproachActionServer::AssembleApproachActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleApproachActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleApproachActionServer::preemptCallback, this));
  as_.start();
}

void AssembleApproachActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleApproach goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleApproachActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  f_measured_.setZero();
  desired_xd_.setZero();
  // desired_x_ = mu_[goal_->arm_name]->position_;
  // desired_xd_(2) = descent_speed_;
  
  origin_ = mu_[goal_->arm_name]->position_;  
  init_rot_ = mu_[goal_->arm_name]->rotation_;

  contact_force_ = goal_->contact_force;
  assemble_dir_ = goal_ ->assemble_dir;
  descent_speed_ = goal_ ->descent_speed;
}

void AssembleApproachActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool AssembleApproachActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleApproachActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleApproachActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
      return false; 
  
  auto & mass = arm.mass_matrix_;
  auto & rotation = arm.rotation_;
  auto & position = arm.position_;
  auto & jacobian = arm.jacobian_;
  auto & tau_measured = arm.tau_measured_;
  auto & gravity = arm.gravity_;
  auto & xd = arm.xd_;
  
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Matrix<double, 6, 1> f_star_zero;
  Eigen::Matrix<double, 6, 6> lambda;
  Eigen::Matrix<double, 7, 6> J_bar;

  lambda = (jacobian*mass.inverse()*jacobian.transpose()).inverse();
  J_bar = mass.inverse()*jacobian.transpose()*lambda;
  f_measured_ = J_bar.transpose()*(tau_measured - gravity);
      
  if(checkContact(f_measured_(assemble_dir_), contact_force_))
  {
    std::cout<<"CHECK CONTATCT!!!!!"<<std::endl;
    as_.setSucceeded();
  }
  else
  {
    f_star = straightMove(origin_, position, xd, assemble_dir_, descent_speed_, time.toSec(), arm.task_start_time_.toSec());
    m_star = keepOrientationPerpenticular(init_rot_, rotation, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());
  }   

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  return true;
}