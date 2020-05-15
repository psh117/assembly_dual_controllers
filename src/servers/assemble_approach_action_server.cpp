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
  
  init_pos_ = mu_[goal_->arm_name]->position_;  
  init_rot_ = mu_[goal_->arm_name]->rotation_;

  origin_ = mu_[goal_->arm_name]->transform_;  

  contact_force_ = goal_->contact_force;
  std::cout<<"contact_threshold: "<<contact_force_<<std::endl;
  // assemble_dir_ = goal_ ->assemble_dir;
  descent_speed_ = goal_ ->descent_speed;
  //mode_ = goal_ ->mode;
  time_limit_ = goal_ ->time_limit;
  tilt_angle_ = goal_ ->tilt_angle;
  tilt_duration_ = goal_->tilt_duration;
  state_ = (ASSEMBLY_STATE)goal_->state;
  set_tilt_ = goal_->set_tilt;

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

  // set_tilt_ = PegInHole::setTilt(T_EA_, 0.1);
  
  // if(state_ != TILT_BACK && state_ != IGNORE)
  // {
  //   if(set_tilt_)
  //   {
  //     set_tilt_back_ = true;
  //     state_ = READY;
  //     tilt_axis_ = PegInHole::getTiltDirection(T_EA_);
  //     std::cout<<"with tilt motion"<<std::endl;
  //   } 
  //   else
  //   {
  //     set_tilt_back_ = false;
  //     state_ = EXEC;
  //     std::cout<<"without tilt motion"<<std::endl;
  //   } 
  // }
  
  is_ready_first_ = true;
  is_approach_first_ = true;
  is_tilt_back_first_ = true;

  if(force_moment.is_open())
    force_moment.close();
  force_moment.open("fm_approach_data.txt");

  control_running_ = true;

  std::cout<<"set tilt: "<<set_tilt_<<std::endl;
  std::cout<<"state_: "<<state_<<std::endl;
}

void AssembleApproachActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleApproachActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

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
  
  f_measured_ = arm.f_measured_;  
  current_ = arm.transform_;
  
  Eigen::Vector6d f_lpf = arm.f_measured_filtered_;

  switch (state_)
  {
    case READY:
      if(is_ready_first_)
      {
        tilt_start_time_ = time.toSec();
        is_ready_first_ = false;
        std::cout<<"TILT"<<std::endl;
      }

      if(timeOut(time.toSec(), tilt_start_time_, tilt_duration_+1.0))
      {
        state_ = EXEC;
        std::cout<<"TILT DONE"<<std::endl;
        // setSucceeded();
      }

      f_star = PegInHole::tiltMotion(origin_, current_, xd, T_EA_, tilt_axis_, tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).head<3>();
      m_star = PegInHole::tiltMotion(origin_, current_, xd, T_EA_, tilt_axis_, tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).tail<3>();
    
      break;

    case EXEC:
      if(is_approach_first_)
      {
        origin_ = arm.transform_;
        approach_star_time_ = time.toSec();        
        is_approach_first_ = false;
        std::cout<<"APPROACH"<<std::endl;
      }
      if(Criteria::checkContact(f_measured_.head<3>(), T_WA_, contact_force_))
      // if(checkContact(f_measured_(assemble_dir_), contact_force_))
      { 
        if(set_tilt_back_) state_ = TILT_BACK;
        else state_ = IGNORE;
        std::cout<<"CHECK CONTATCT!!!!!"<<std::endl;
        setSucceeded();
      }
      
      if(timeOut(time.toSec(), arm.task_start_time_.toSec(), time_limit_)) //duration wrong??
      { 
        std::cout<<"Time out"<<std::endl;
        setAborted();
      } 
      
      f_star = PegInHole::straightMoveEE(origin_, current_, xd, T_EA_, descent_speed_, time.toSec(), approach_star_time_);
      f_star = T_WA_.linear()*f_star;

      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 200, 5);
      // std::cout<<"f_star for approach: "<<f_star.transpose()<<std::endl;
      break;

    case TILT_BACK:
      if(is_tilt_back_first_)
      {
        tilt_start_time_ = time.toSec();
        is_tilt_back_first_ = false;
        std::cout<<"TILT BACK"<<std::endl;
      }

      if(timeOut(time.toSec(), tilt_start_time_, tilt_duration_))
      {
        std::cout<<"TILT BACK DONE"<<std::endl;
        setSucceeded();
      }

      f_star = PegInHole::tiltMotion(origin_, current_, xd, T_EA_, tilt_axis_, -tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).head<3>();
      m_star = PegInHole::tiltMotion(origin_, current_, xd, T_EA_, tilt_axis_, -tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).tail<3>();
    
      break;

    case IGNORE:
      std::cout<<"move to the next step"<<std::endl;
      setSucceeded();
      break;
  }

  
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"f_a: "<<(T_WA_.linear().inverse()*(f_measured_.head<3>())).transpose()<<std::endl;

  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  
  // // f_star_zero.setZero();  
  // std::cout<<"f_measured: "<<f_measured_.transpose()<<std::endl;
  // std::cout<<"f_filtered: "<<f_lpf.transpose()<<std::endl;

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  force_moment<<f_measured_(0) << "\t" << f_measured_(1)<< "\t" << f_measured_(2)<< "\t"<< arm.f_measured_filtered_(0) << "\t" << arm.f_measured_filtered_(1)<< "\t" << arm.f_measured_filtered_(2)<< "\n";

  return true;
}

void AssembleApproachActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleApproachActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}