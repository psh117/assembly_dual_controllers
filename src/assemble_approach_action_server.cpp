#include <assembly_dual_controllers/assemble_approach_action_server.h>

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
  desired_x_ = mu_[goal_->arm_name]->position_;
  desired_xd_(2) = descent_speed_;
  ori_init_assembly_ = mu_[goal_->arm_name]->rotation_;
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

  auto & rotation = arm.rotation_;
  auto & position = arm.position_;
  auto & jacobian = arm.jacobian_;
  auto & tau_measured = arm.tau_measured_;
  auto & gravity = arm.gravity_;
  auto & xd = arm.xd_;

  double contact_force = goal_->contact_force;   
      
  Eigen::Matrix3d K_p_asm;
  Eigen::Matrix3d K_v_asm; //control gain for peg in hole task
  Eigen::Vector3d f_star_asm; //linear force
	Eigen::Vector3d m_star_asm; //orientation
	Eigen::Matrix<double, 6, 1> f_star_zero_asm;
  Eigen::Vector3d delphi_delta_asm; // value related with orientation

  
  K_p_asm.setZero(); K_v_asm.setZero();
	for (int i = 0; i < 3; i++)
	{
		K_p_asm(i, i) = 10000.0; K_v_asm(i, i) = 40.; //7000
	}
  //K_p_asm(2, 2) = 3000.0; //5000
  
  f_star_asm = K_p_asm * (desired_x_ - position) + K_v_asm *(desired_xd_.head<3>() - xd.head<3>());  
  //f_star_asm = K_v_asm *(desired_xd_.head(3) - xd.head(3));  
  
  delphi_delta_asm = -0.5 * getPhi(rotation, ori_init_assembly_);
  m_star_asm = (1.0) * 200 * delphi_delta_asm - 20 * xd.tail(3);
  
  
  f_star_zero_asm.head(3) = f_star_asm;
  f_star_zero_asm.tail(3) = m_star_asm;

  
  //f_measured_ = (jacobian * jacobian.transpose()).inverse() * jacobian * (tau_measured - gravity);
      

  // std::cout << f_star_zero_asm.transpose() << std::endl;

  //if (abs(f_measured_(2)) > contact_force)
  //{
  //    as_.setSucceeded();
  //}

  f_star_zero_asm(2) = -1.0;


  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero_asm;
  arm.setTorque(desired_torque);

  desired_x_(2) += descent_speed_ / 1000.;

  std::cout<<f_star_zero_asm.transpose()<<std::endl;
  return true;
}