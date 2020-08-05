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
  
  origin_ = mu_[goal_->arm_name]->transform_;  
  contact_force_ = goal_->contact_force;
  
  descent_speed_ = goal_ ->descent_speed;  
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

  tilt_back_threshold_ = goal_ -> tilt_back_threshold;
  //TODO : delete T_WD_, T_AD_
  T_EA_.linear() = ee_to_assembly_quat_.toRotationMatrix();
  T_EA_.translation() = ee_to_assembly_point_;

  tilt_axis_ = getTiltDirection(T_EA_);

  T_WA_ = origin_*T_EA_;

  is_ready_first_ = true;
  is_approach_first_ = true;
  is_tilt_back_first_ = true;

  if(force_moment.is_open())  force_moment.close();
  if(force_moment_lpf.is_open()) force_moment_lpf.close();
  if(contact_force.is_open()) contact_force.close();

  force_moment.open("fm_approach_data.txt");
  force_moment_lpf.open("fm_approach_data_lfp.txt");
  contact_force.open("contact_force.txt");

  control_running_ = true;

  std::cout<<"APPROACH START"<<std::endl;
  std::cout<<"set tilt: "<<set_tilt_<<std::endl;
  // std::cout<<"state_: "<<state_<<std::endl;
  std::cout<<"tilt_angle: "<<tilt_angle_<<std::endl;
  // std::cout<<"tilt_duration: "<<tilt_duration_<<std::endl;
  // std::cout<<"T_EA_: \n"<< T_EA_.matrix()<<std::endl;
  // std::cout<<"T_WA_: \n"<< T_WA_.matrix()<<std::endl;
  // std::cout<<"init_rot: \n"<< origin_.linear()<<std::endl;
  // std::cout<<"ASSEMBLY_FRAM_POSITION: "<<T_WA_.translation().transpose()<<std::endl;
  // std::cout<<"contact_threshold: "<<contact_force_<<std::endl;
  // std::cout<<"tilt_back_threshold: "<<tilt_back_threshold_<<std::endl;
  // std::cout<<"descent speed: "<< descent_speed_<<std::endl;
  // std::cout<<goal_->arm_name<<std::endl;

  accumulated_wrench_.setZero();
  count_ = 0;
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
  
  Eigen::Vector3d f_star, m_star, f_contact;
  Eigen::Vector6d f_star_zero, f_lpf;
  double run_time;

  f_measured_ = arm.f_measured_;  
  f_lpf = arm.f_measured_filtered_;
  current_ = arm.transform_;
  
  int cnt_max = 300;
  int cnt_start = 250;
  if(count_ < cnt_max)
  { 
    f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 7500, 200, 300, 5).head<3>(); //w.r.t {W}
    m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 7500, 200, 300, 5).tail<3>();
    if(count_ > cnt_start) PegInHole::getCompensationWrench(accumulated_wrench_, f_lpf, cnt_start, count_, cnt_max);    
    count_++;
    if(count_ >= cnt_max)
    {
      count_ = cnt_max;
      accumulated_wrench_a_.head<3>() = T_WA_.linear().inverse() * accumulated_wrench_.head<3>();
      accumulated_wrench_a_.tail<3>() = T_WA_.linear().inverse() * accumulated_wrench_.tail<3>();
      std::cout << "output for compensation: " << accumulated_wrench_.transpose() << std::endl;
      wrench_rtn_.force.x = accumulated_wrench_a_(0);
      wrench_rtn_.force.y = accumulated_wrench_a_(1);
      wrench_rtn_.force.z = accumulated_wrench_a_(2);
      wrench_rtn_.torque.x = accumulated_wrench_a_(3);
      wrench_rtn_.torque.y = accumulated_wrench_a_(4);
      wrench_rtn_.torque.z = accumulated_wrench_a_(5);
      result_.compensation = wrench_rtn_;
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
        origin_ = arm.transform_;
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

      f_star = PegInHole::tiltMotion(origin_, current_, xd, T_EA_, tilt_axis_, tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).head<3>();
      m_star = PegInHole::tiltMotion(origin_, current_, xd, T_EA_, tilt_axis_, tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).tail<3>();
      break;

    case EXEC:
      if (is_approach_first_)
      {
        origin_ = arm.transform_;
        approach_star_time_ = time.toSec();
        is_approach_first_ = false;
        std::cout << "APPROACH" << std::endl;
      }

      run_time = time.toSec() - approach_star_time_;

      if (run_time > 0.5 && Criteria::checkContact(f_lpf.head<3>() - accumulated_wrench_.head<3>(), T_WA_, contact_force_))
      {
        if (set_tilt_back_)
          state_ = TILT_BACK;
        else
          state_ = IGNORE;
        // std::cout<<"running time: "<< run_time<<std::endl;
        std::cout << "CHECK CONTATCT!!!!!" << std::endl;
        setSucceeded();
      }

      if (timeOut(time.toSec(), arm.task_start_time_.toSec(), time_limit_)) //duration wrong??
      {
        std::cout << "Time out" << std::endl;
        setAborted();
      }

      f_star = PegInHole::approachComponentEE(origin_, current_, xd, T_EA_, descent_speed_, time.toSec(), approach_star_time_);
      f_star = T_WA_.linear() * f_star;

      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 400, 5);

      force_moment << f_measured_.transpose() << std::endl;
      force_moment_lpf << f_lpf.transpose() << std::endl;

      break;

    case TILT_BACK:

      // f_contact = T_WA_.linear().inverse()*f_lpf.head<3>() + accumulated_wrench_.head<3>();

      if (is_tilt_back_first_)
      {
        tilt_start_time_ = time.toSec();
        origin_ = arm.transform_;
        is_tilt_back_first_ = false;
        std::cout << "TILT BACK" << std::endl;
      }

      run_time = time.toSec() - tilt_start_time_;

      if (timeOut(time.toSec(), tilt_start_time_, tilt_duration_))
      {
        std::cout << "TILT BACK DONE" << std::endl;
        setSucceeded();
      }


      // if (run_time > 0.5 && Criteria::checkForceLimit(f_contact, tilt_back_threshold_))
      if (run_time > 0.5 && Criteria::checkContact(f_lpf.head<3>() - accumulated_wrench_.head<3>(), T_WA_, tilt_back_threshold_))
      {
        std::cout << "TILT BACK REACHED FORCE LIMIT" << std::endl;
        std::cout<<"tilt_back_threshold_: "<<tilt_back_threshold_<<std::endl;
        setSucceeded();
      }

      f_star = PegInHole::tiltMotion(origin_, current_, xd, T_EA_, tilt_axis_, -tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).head<3>();
      m_star = PegInHole::tiltMotion(origin_, current_, xd, T_EA_, tilt_axis_, -tilt_angle_, time.toSec(), tilt_start_time_, tilt_duration_).tail<3>();
      force_moment << f_measured_.transpose() << std::endl;
      force_moment_lpf << f_lpf.transpose() << std::endl;
      contact_force << f_contact.transpose()<<std::endl;

      break;

    case IGNORE:
      std::cout << "move to the next step" << std::endl;
      f_star = PegInHole::keepCurrentPosition(origin_, current_, xd);
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd);
      setSucceeded();
      break;
    }
  }

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  
  // f_star_zero.setZero();  
  // std::cout<<"f_measured: "<<f_measured_.transpose()<<std::endl;
  // std::cout<<"f_filtered: "<<f_lpf.transpose()<<std::endl;

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  

  return true;
}

void AssembleApproachActionServer::setSucceeded()
{
  result_.is_completed = true;
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleApproachActionServer::setAborted()
{
  result_.is_completed = false;
  as_.setAborted(result_);
  control_running_ = false;
}