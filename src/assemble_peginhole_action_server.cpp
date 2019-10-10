#include <assembly_controllers/assemble_peginhole_action_server.h>

AssemblePegInHoleActionServer::AssemblePegInHoleActionServer(std::string name, ros::NodeHandle &nh, 
                                std::shared_ptr<FrankaModelUpdater> &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssemblePegInHoleActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssemblePegInHoleActionServer::preemptCallback, this));
  as_.start();
}

void AssemblePegInHoleActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  ROS_INFO("AssemblePegInHole goal has been received.");

  start_time_ = ros::Time::now();
  spiral_starting_pos_assembly_ = mu_->position_;
  ori_init_assembly_ = mu_->rotation_;

}

void AssemblePegInHoleActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool AssemblePegInHoleActionServer::getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque)
{
  if (!as_.isActive())
    return false; 

  auto & rotation = mu_->rotation_;
  auto & position = mu_->position_;
  auto & jacobian = mu_->jacobian_;
  auto & tau_measured = mu_->tau_measured_;
  auto & gravity = mu_->gravity_;
  auto & xd = mu_->xd_;

  double desired_z_force = goal_->z_force;
  double linear_velocity = goal_->linear_velocity;
  double pitch = goal_->pitch;
  double insert_force = goal_->insert_force;

  //------If you saw a parameter whick looks like ***_asm, the parameter is only used in the "assembleUpdate"
  // if(time - task_start_time_)
  Eigen::Matrix<double, 7, 1> tau_cmd_asm;
  Eigen::Vector3d pos_hole_asm; //the location of a hole(the position at last time of spiral)
  Eigen::Matrix3d K_p_asm; //control gain for peg in hole task
  Eigen::Matrix3d K_v_asm; //control gain for peg in hole task
  Eigen::Vector3d f_star_asm; //linear force
	Eigen::Vector3d m_star_asm; //orientation
	Eigen::Matrix<double, 6, 1> f_star_zero_asm;
  Eigen::Matrix<double, 6, 1> f_sensing_asm;
  Eigen::Vector3d delphi_delta_asm; // value related with orientation

  x_desired_assembly_.setZero();

  if ((position(2) < spiral_starting_pos_assembly_ (2)-0.008)&&(check_stop_assemlby_ == 0)) //To check a pin is inserted correctly
  {
    check_stop_assemlby_ = 1;
    pos_hole_asm = position; // store the location of a hole
    as_.setSucceeded();
  }

  double search_time = 40.0;
  x_desired_assembly_.block<2, 1>(0, 0) = spiral
  (time.toSec(), start_time_.toSec(), start_time_.toSec() + search_time, 
  spiral_starting_pos_assembly_.block<2, 1>(0, 0), linear_velocity, pitch);
  // 0.005, 0.002
  // 0.005, 0.0
	x_desired_assembly_(2) = spiral_starting_pos_assembly_(2);

  //////"for loop" to set gains------------
  K_p_asm.setZero(); K_v_asm.setZero();
	for (int i = 0; i < 3; i++)
	{
		K_p_asm(i, i) = 5000.0; K_v_asm(i, i) = 100.0; //7000
	}
  K_p_asm(2, 2) = 3000.0; //5000
  ///------------------------------------
  
  f_star_asm = K_p_asm * (x_desired_assembly_ - position) - K_v_asm * xd.head(3);  
  f_star_asm(2) = desired_z_force; // -6.0; // setpoint torque by trial

  if(check_stop_assemlby_ == 1)
  {
    f_star_asm(2) = insert_force;
  }

  delphi_delta_asm = -0.5 * getPhi(rotation, ori_init_assembly_);
  m_star_asm = (1.0) * 200 * delphi_delta_asm - 20 * xd.tail(3);
  
 
  f_star_zero_asm.head(3) = f_star_asm;
	f_star_zero_asm.tail(3) = m_star_asm;

  
  f_sensing_asm = (jacobian * jacobian.transpose()).inverse() * jacobian * (tau_measured - gravity);

  torque = jacobian.transpose() * f_star_zero_asm;

  feedback_.percent_complete = (time-start_time_).toSec() / search_time * 100.0;
  // feedback_.header.seq=feedback_header_stamp_;
  // feedback_header_stamp_++;
  as_.publishFeedback(feedback_);


  return true;
}



