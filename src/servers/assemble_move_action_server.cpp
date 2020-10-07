#include <assembly_dual_controllers/servers/assemble_move_action_server.h>

AssembleMoveActionServer::AssembleMoveActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleMoveActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleMoveActionServer::preemptCallback, this));
  as_.start();
}

void AssembleMoveActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleMove goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleMoveActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  
  origin_ = mu_[goal_->arm_name]->transform_;  
  
  target_pos_(0) = goal_->move_to_target.position.x;
  target_pos_(1) = goal_->move_to_target.position.y;
  target_pos_(2) = goal_->move_to_target.position.z;
  target_quat_.x() = goal_->move_to_target.orientation.x;
  target_quat_.y() = goal_->move_to_target.orientation.y;
  target_quat_.z() = goal_->move_to_target.orientation.z;
  target_quat_.w() = goal_->move_to_target.orientation.w;
  
  std::cout<<"target w.r.t assemble frame: "<<target_pos_.transpose()<<std::endl;

  flange_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  flange_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  flange_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  flange_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  flange_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  flange_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  flange_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;

  target_.translation() = target_pos_; //start from {A} frame, T_AD(T_AA')
  target_.linear() = target_quat_.toRotationMatrix();

  T_7A_.linear() = flange_to_assembly_quat_.toRotationMatrix();
  T_7A_.translation() = flange_to_assembly_point_;
  T_WA_ = origin_*T_7A_;

  // target_ = target_*T_7A_.inverse();

  // Eigen::Vector3d relative_pose = target_pos_ - origin_.translation(); //P_we
  // Eigen::Matrix3d relative_rot = T_WA_.linear().inverse()*origin_.linear(); //R_ae

  // target_.translation() = T_WA_.linear().inverse()*relative_pose; // P_ae
  // target_.linear() = target_quat_.toRotationMatrix();
  
  // target_ = T_WA_*target_;
  
  is_mode_changed_ = true;
  state_ = LIFT_UP;
  
}

void AssembleMoveActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool AssembleMoveActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleMoveActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleMoveActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
      return false; 

  // auto & mass = arm.mass_matrix_;
  // auto & rotation = arm.rotation_;
  // auto & position = arm.position_;
  auto & jacobian = arm.jacobian_;
  // auto & tau_measured = arm.tau_measured_;
  auto & gravity = arm.gravity_;
  auto & xd = arm.xd_; //velocity
  
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d f_star,m_star;
  Eigen::Vector6d f_star_zero;
  Eigen::Matrix3d rot_target;
  double duration;
  double lift_up;
  current_ = arm.transform_;
  duration = 1.0;
  lift_up = -0.03;

  if(is_mode_changed_)
  {
    arm.task_start_time_ = time;
    origin_ = arm.transform_;
    is_mode_changed_ = false;
    std::cout<<(STATE)state_<<std::endl;
  }

  switch (state_)
  {
  case LIFT_UP: //move along z-axis w.r.t {A}   
    
    f_star = PegInHole::oneDofMoveEE(origin_, current_, xd, T_7A_, time.toSec(), arm.task_start_time_.toSec(), duration, lift_up, 2);
    f_star = T_WA_.linear()*f_star;
    m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 2000, 15);

    if(timeOut(time.toSec(), arm.task_start_time_.toSec(), duration + 0.1))
    {
      state_ = MOVE;
      is_mode_changed_ = true;
      std::cout<<"LIFT UP! GO TO NEXT MOTION"<<std::endl;
    }

    break;
  case MOVE: // move on X-Y plane w.r.t {A}
    
    f_star = PegInHole::twoDofMoveEE(origin_, current_, xd, T_7A_, time.toSec(), arm.task_start_time_.toSec(), duration*2, target_.translation(), 2);
    f_star = T_WA_.linear()*f_star;    
    m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 2000, 15);
    // m_star = PegInHole::rotateWithMat(origin_, current_, xd,  T_WA_.linear()*target_quat_.toRotationMatrix(), time.toSec(), arm.task_start_time_.toSec(), duration);
    
    if(timeOut(time.toSec(), arm.task_start_time_.toSec(), duration*2 + 0.1))
    {
      state_ = COMPLETE;
      is_mode_changed_ = true;
      std::cout<<"FINITHED"<<std::endl;
    }
    break;

  case ROTATE:
    // duration = 2.0;
   
    // f_star = PegInHole::keepCurrentPosition(origin_, current_, xd, 6000, 200);
    // // m_star = PegInHole::rotateWithMat(origin_, current_, xd,  T_WA_.linear()*target_.linear(), time.toSec(), arm.task_start_time_.toSec(), duration);
    // // rot_target = target_quat_.toRotationMatrix()*T_7A_.linear().inverse();
    // m_star = PegInHole::rotateWithMat(origin_, current_, xd,  target_quat_.toRotationMatrix(), time.toSec(), arm.task_start_time_.toSec(), duration);

    // if (timeOut(time.toSec(), arm.task_start_time_.toSec(), duration + 0.1))
    // {
    //   state_ = COMPLETE;
    //   is_mode_changed_ = true;
    //   std::cout<<"FINITHED"<<std::endl;
    // }
    break;

  case COMPLETE:
      setSucceeded();
  }

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_*f_star_zero;
  arm.setTorque(desired_torque);

  return true;
}

void AssembleMoveActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleMoveActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}

double AssembleMoveActionServer::getDistance(const Eigen::Vector3d target, const Eigen::Vector3d start, 
  const int type, const int dir)
{
  double dis;
  Eigen::Vector3d dx;

  dx << target(0) - start(0), target(1) - start(1), target(2) - start(2); 
  
  if(type == 1)
  {
    dis = sqrt(pow(dx(dir),2));
  }

  if(type == 2)
  {
    dx(dir) = 0;
    dis = sqrt(pow(dx(0),2) + pow(dx(1),2) + pow(dx(2),2));
  }

  return dis;  
}