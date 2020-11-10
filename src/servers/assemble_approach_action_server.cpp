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
  total_action_start_time_ = ros::Time::now().toSec();
  // origin_ = mu_[goal_->arm_name]->transform_;  
  origin_ = mu_[goal_->arm_name]->initial_transform_;
  contact_force_ = goal_->contact_force;
  
  descent_speed_ = goal_ ->descent_speed;  
  time_limit_ = goal_ ->time_limit;
  tilt_angle_ = goal_ ->tilt_angle;
  tilt_duration_ = goal_->tilt_duration;
  state_ = (ASSEMBLY_STATE)goal_->state;
  set_tilt_ = goal_->set_tilt;

  flange_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  flange_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  flange_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  flange_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  flange_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  flange_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  flange_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;

  tilt_back_threshold_ = goal_ -> tilt_back_threshold;
  //TODO : delete T_WD_, T_AD_
  T_7A_.linear() = flange_to_assembly_quat_.toRotationMatrix(); // from the flange frame to the assembly frame
  T_7A_.translation() = flange_to_assembly_point_;

  tilt_axis_ = getTiltDirection(T_7A_);

  T_WA_ = origin_*T_7A_;

  is_ready_first_ = true;
  is_approach_first_ = true;
  is_tilt_back_first_ = true;
  
  control_running_ = true;

  std::cout<<"APPROACH START"<<std::endl;
  std::cout<<"set tilt: "<<set_tilt_<<std::endl;
  std::cout<<"T_7A_: \n"<< T_7A_.matrix()<<std::endl;
  std::cout<<"contact_threshold: "<<contact_force_<<std::endl;
  std::cout<<"tilt_back_threshold: "<<tilt_back_threshold_<<std::endl;
  std::cout<<goal_->arm_name<<std::endl;
  accumulated_wrench_.setZero();
  count_ = 0;

  q_init_ = mu_[goal_->arm_name]->q_;
  q_null_target_ = getNullSpaceJointTarget(rbdl_panda_.getJointLimit(), q_init_);
  
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
  
  // auto & mass = arm.mass_matrix_;
  // auto & rotation = arm.rotation_;
  auto & position = arm.position_;
  auto & jacobian = arm.jacobian_;
  // auto & tau_measured = arm.tau_measured_;
  // auto & gravity = arm.gravity_;
  auto & xd = arm.xd_;
  
  Eigen::Vector3d f_star, m_star;
  Eigen::Vector6d f_star_zero, f_ext, f_tilt;
  double run_time;
  Eigen::Vector3d force;
  
  f_ext = arm.f_ext_;
  current_ = arm.transform_;
  
  int cnt_max = 500;
  int cnt_start = 100;
  if(count_ < cnt_max)
  { 
    f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 700, 10, 2000, 15).head<3>(); //w.r.t {W}
    m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 700, 10, 2000, 15).tail<3>();
    
    if(count_ > cnt_start) PegInHole::getCompensationWrench(accumulated_wrench_, f_ext, cnt_start, count_, cnt_max);    
    count_++;
    if(count_ >= cnt_max)
    {
      count_ = cnt_max;
      accumulated_wrench_a_.head<3>() = T_WA_.linear().inverse() * accumulated_wrench_.head<3>();
      accumulated_wrench_a_.tail<3>() = T_WA_.linear().inverse() * accumulated_wrench_.tail<3>();
      // std::cout << "output for compensation: " << accumulated_wrench_.transpose() << std::endl;
      // wrench_rtn_.force.x = accumulated_wrench_a_(0);
      // wrench_rtn_.force.y = accumulated_wrench_a_(1);
      // wrench_rtn_.force.z = accumulated_wrench_a_(2);
      // wrench_rtn_.torque.x = accumulated_wrench_a_(3);
      // wrench_rtn_.torque.y = accumulated_wrench_a_(4);
      // wrench_rtn_.torque.z = accumulated_wrench_a_(5);
      // result_.compensation = wrench_rtn_;
      // heavy_mass_ = Criteria::holdHeavyMass(accumulated_wrench_.head<3>(), 8.0);
    } 
  }
  else
  {
    switch (state_)
    {
    case READY:
      if (is_ready_first_)
      {
        tilt_start_time_ = time.toSec();
        is_ready_first_ = false;
        std::cout << "time check: " << tilt_start_time_ - time.toSec() << std::endl;
        std::cout << "tilt duration: " << tilt_duration_ << std::endl;
        std::cout << "TILT" << std::endl;
      }

      if (timeOut(time.toSec(), tilt_start_time_, tilt_duration_ + 1.0))
      {
        state_ = EXEC;
        std::cout << "TILT DONE" << std::endl;
      }
      
      f_tilt = PegInHole::tiltMotion(origin_, current_, xd, T_7A_, tilt_axis_, tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_, 600.0, 20.0);
      f_star = f_tilt.head<3>();
      m_star = f_tilt.tail<3>();
      
      break;

    case EXEC:
      if (is_approach_first_)
      {
        origin_ = arm.transform_;
        approach_start_time_ = time.toSec();
        is_approach_first_ = false;
        std::cout << "APPROACH" << std::endl;
      }

      run_time = time.toSec() - approach_start_time_;
      force = f_ext.head<3>() - accumulated_wrench_.head<3>();
      
      if (timeOut(time.toSec(), arm.task_start_time_.toSec(), time_limit_)) //duration wrong??
      {
        std::cout << "Time out" << std::endl;
        setAborted();
        break;
      }

      if (run_time > 0.5 && Criteria::checkContact(force, T_WA_, contact_force_))
      {
        if (set_tilt_back_) state_ = TILT_BACK;
        else                state_ = IGNORE;
        std::cout << "CHECK CONTATCT!!!!!" << std::endl;
        setSucceeded();
        break;
      }

      f_star = PegInHole::approachComponentEE(origin_, current_, xd, T_7A_, descent_speed_, time.toSec(), approach_start_time_, 600, 20);
      f_star = T_WA_.linear() * (f_star);
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 2000, 15);
      
      // if(heavy_mass_)
      // {
      //   f_star = T_WA_.linear() * (f_star);// + accumulated_wrench_.head<3>();
      //   m_star = m_star + 1.2*accumulated_wrench_.tail<3>();
      // }
      // else
      // {
      //   f_star = T_WA_.linear() * (f_star);
      // }
      
      break;

    case TILT_BACK:
      run_time = time.toSec() - tilt_start_time_;
      // f_contact = T_WA_.linear().inverse()*f_ext.head<3>() + accumulated_wrench_.head<3>();
      if (is_tilt_back_first_)
      {
        tilt_start_time_ = time.toSec();
        origin_ = arm.transform_;
        is_tilt_back_first_ = false;
        std::cout << "TILT BACK" << std::endl;
      }
      
      if (timeOut(time.toSec(), tilt_start_time_, tilt_duration_ + 1.5))
      {
        std::cout << "TILT BACK DONE" << std::endl;
        setSucceeded();
      }
      // if (run_time > 0.5 && Criteria::checkForceLimit(f_contact, tilt_back_threshold_))
      if (run_time > 0.5 && Criteria::checkContact(f_ext.head<3>() - accumulated_wrench_.head<3>(), T_WA_, tilt_back_threshold_))
      {
        std::cout << "TILT BACK REACHED FORCE LIMIT" << std::endl;
        std::cout<<"tilt_back_threshold_: "<<tilt_back_threshold_<<std::endl;
        setSucceeded();
      }

      f_tilt = PegInHole::tiltMotion(origin_, current_, xd, T_7A_, tilt_axis_, -(tilt_angle_*1.5), time.toSec(), tilt_start_time_, tilt_duration_+0.5, 700.0, 40.0);
      f_star = f_tilt.head<3>();
      m_star = f_tilt.tail<3>();

      break;

    case IGNORE:
      std::cout << "move to the next step" << std::endl;
      f_star = PegInHole::keepCurrentPosition(origin_, current_, xd);
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd);
      setSucceeded();
      break;
    }
  }

  if(state_ == (ASSEMBLY_STATE) EXEC)
  {
    Eigen::Vector6d f_contact;
    f_contact.head<3>() = T_WA_.linear().inverse()*f_ext.head<3>() - accumulated_wrench_a_.head<3>();
    f_contact.tail<3>() = force;
    force_moment<< f_contact.transpose()<<std::endl;
    approach_pose_data << arm.position_.transpose()<<std::endl;
  }

  if(state_ == (ASSEMBLY_STATE) TILT_BACK)
  {
    Eigen::Vector6d f_contact;
    f_contact.head<3>() = T_WA_.linear().inverse()*f_ext.head<3>() - accumulated_wrench_a_.head<3>();
    f_contact.tail<3>() = T_WA_.linear().inverse()*f_ext.tail<3>() - accumulated_wrench_a_.tail<3>();
    force_moment_tilt_back<< f_contact.transpose()<<std::endl;
  }

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  
  // f_star_zero.setZero();  
  // std::cout<<"f_measured: "<<f_measured_.transpose()<<std::endl;
  // std::cout<<"f_filtered: "<<f_ext.transpose()<<std::endl;

  tau_null_cmd_ = PegInHole::nullSpaceJointTorque(arm.q_, q_init_, q_null_target_, arm.qd_, time.toSec(), total_action_start_time_, 2.5, 75, 1);
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() *arm.modified_lambda_matrix_* f_star_zero;// + arm.null_projector_*tau_null_cmd_;
  // Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;// + arm.null_projector_*tau_null_cmd_;
  // std::cout<<"tau_null_cmd : "<< tau_null_cmd_.transpose()<<std::endl;
  arm.setTorque(desired_torque);

  return true;
}

void AssembleApproachActionServer::setSucceeded()
{
  // result_.is_completed = true;
  // as_.setSucceeded(result_);
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleApproachActionServer::setAborted()
{
  // result_.is_completed = false;
  // as_.setAborted(result_);
  as_.setAborted();
  control_running_ = false;
}