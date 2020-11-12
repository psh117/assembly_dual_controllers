#include <assembly_dual_controllers/servers/assemble_retreat_bolt_action_server.h>

AssembleRetreatBoltActionServer::AssembleRetreatBoltActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleRetreatBoltActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleRetreatBoltActionServer::preemptCallback, this));
  as_.start();
}

void AssembleRetreatBoltActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleRetreat goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleRetreatBoltActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  changed_task_time_ = mu_[goal_->arm_name]->task_start_time_.toSec();

  origin_ = mu_[goal_->arm_name]->transform_;
  
  retreat_force_ = goal_->retreat_force;
  retreat_distance_ = goal_->retreat_distance;
  duration_ = goal_ ->duration;
  time_limit_ = goal_->time_limit;

  wiggle_motion_ = goal_->wiggle_motion;
  wiggle_motion_z_axis_ = goal_->wiggle_motion_z_axis;
  reverse_moment_ = goal_->reverse_moment;
  wiggle_angle_ = goal_->wiggle_angle * DEG2RAD; //radian
  
  mode_ = goal_->mode;

  std::cout<<goal_->arm_name<<std::endl;
  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"init_yaw_angle: "<<init_yaw_angle_<<std::endl;
  // std::cout<<"init_yaw_goal: "<<yawing_angle_<<std::endl;
  // std::cout<<"sum: "<<yawing_angle_ + init_yaw_angle_<<std::endl;
  
  compliant_translation_ee_select_mtx_ = Eigen::Matrix3d::Identity();
  compliant_translation_ee_select_mtx_(2,2) = 0.0; // z-axis is defined as the assemble direction.
  
  flange_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  flange_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  flange_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  flange_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  flange_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  flange_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  flange_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;

  T_7A_.linear() = flange_to_assembly_quat_.toRotationMatrix();
  T_7A_.translation() = flange_to_assembly_point_;
 
  T_WA_ = origin_*T_7A_;

  wiggle_angular_vel_ = 4*M_PI; // 2pi / s
  
  repeatition_ = 0;

  if(wiggle_motion_ && wiggle_motion_z_axis_) std::cout<<"wiggle motions along all axes are activated"<<std::endl;
  else if(wiggle_motion_) std::cout<<"wiggle motion along x/y axis are activated"<<std::endl;
  else if(wiggle_motion_z_axis_) std::cout<<"wiggle motion along z axis is activated"<<std::endl;
  else  std::cout<<"no optional motions"<<std::endl;

  std::cout<<"ORIGIN: "<<origin_.translation().transpose()<<std::endl;
  std::cout<<"before retreating TF : \n"<< origin_.linear()<<std::endl;
   
   // To define parameters of wiggle motion along z-axis
  
  if(mu_[goal_->arm_name]->q_(6,0) + wiggle_angle_ >= rbdl_panda_.getJointLimit()(6,1))
  {
    wiggle_z_axis_param_.a = 1;
    wiggle_z_axis_param_.b = mu_[goal_->arm_name]->q_(6,0) - wiggle_angle_;
    wiggle_z_axis_param_.t_offset = 2*M_PI/wiggle_angular_vel_/4;
  }
  else if(mu_[goal_->arm_name]->q_(6,0) - wiggle_angle_ <= rbdl_panda_.getJointLimit()(6,0))
  {
    wiggle_z_axis_param_.a = -1;
    wiggle_z_axis_param_.b = mu_[goal_->arm_name]->q_(6,0) + wiggle_angle_;
    wiggle_z_axis_param_.t_offset = 2*M_PI/wiggle_angular_vel_/4;
  }
  else
  {
    wiggle_z_axis_param_.a = 0;
    wiggle_z_axis_param_.b = 0;
    wiggle_z_axis_param_.t_offset = 0;
  }

  control_running_ = true;
}

void AssembleRetreatBoltActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleRetreatBoltActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleRetreatBoltActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}

bool AssembleRetreatBoltActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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

  Eigen::Vector3d f_star, m_star, m_wig, m_wig_z;
  Eigen::Vector6d f_star_zero;
  Eigen::Vector3d d;
  double run_time;
  double holding_time = 5.0;

  current_ = arm.transform_;
  run_time = time.toSec() - arm.task_start_time_.toSec();
 
  d = (T_7A_.inverse()).translation() - (T_WA_.inverse()*current_).translation();

  if(mode_ == 0) // to move backward
  {
    if(Criteria::timeOut(time.toSec(), arm.task_start_time_.toSec(), duration_))
    {
      std::cout<<"Move backward is done"<<std::endl;
      setSucceeded();
      return true;
    }
    Eigen::Vector3d f_a;
    f_star = PegInHole::keepCurrentPosition(origin_, current_, xd, 700.0, 40.0);
    f_a = T_WA_.linear().inverse()*f_star; // transfomr from {W} to {A}
    f_a = compliant_translation_ee_select_mtx_*f_a; // set complianct motion 
    f_star = T_WA_.linear()*f_a;
    // std::cout<<"f_star with compliant motion : "<<f_star.transpose()<<std::endl;
  }
  else if(mode_ == 1) // to retreat drill from a bolt with force control
  {
    if(run_time > 1.0 && abs(d(2)) > retreat_distance_) // dp(2) is always negative 
    {
      std::cout<<"Success retreat action"<<std::endl;
      setSucceeded();
      return true;
    }
    if(Criteria::timeOut(time.toSec(), arm.task_start_time_.toSec(), time_limit_))
    {
      std::cout<<"Fail retreat action"<<std::endl;
      setAborted();
      return true;
    }
    f_star = PegInHole::pressCubicEE(origin_, current_, xd, T_WA_, -retreat_force_, time.toSec(), arm.task_start_time_.toSec() + holding_time, 2.0, 100.0, 0.0);
    Eigen::Vector3d target;
    f_star = T_WA_.linear()*f_star;
    // reaction_force << arm.f_ext_.transpose()<<std::endl;
  }  

  else if(mode_ == 2) // to retreat drill from a bolt with velocity control
  {
    if(run_time > 1.0 && abs(d(2)) > retreat_distance_) // dp(2) is always negative 
    {
      std::cout<<"Success retreat action"<<std::endl;
      setSucceeded();
      return true;
    }
    if(Criteria::timeOut(time.toSec(), arm.task_start_time_.toSec(), time_limit_))
    {
      std::cout<<"Fail retreat action"<<std::endl;
      setAborted();
      return true;
    }
    
    Eigen::Vector3d target;
    target << 0.0, 0.0, -retreat_distance_*1.2;
    f_star = PegInHole::threeDofMoveEE(origin_, current_, target, xd, T_7A_, time.toSec(), arm.task_start_time_.toSec(), duration_, 300.0, 20.0); 
    // f_star.head<2>().setZero();
    f_star = T_WA_.linear()*f_star;
    reaction_force << arm.f_ext_.transpose()<<std::endl;
  } 
  
  else if(mode_ == 3) // to make a vibration on the tip of drill
  {
    double t_e;
    t_e = time.toSec() - changed_task_time_;

    if(run_time > 1.0 && abs(d(2)) > retreat_distance_) // dp(2) is always negative 
    {
      std::cout<<"Success retreat action"<<std::endl;
      setSucceeded();
      return true;
    }
    if(Criteria::timeOut(time.toSec(), arm.task_start_time_.toSec(), time_limit_))
    {
      std::cout<<"Fail retreat action"<<std::endl;
      setAborted();
      return true;
    }

    f_star = PegInHole::generateForceToDisassembleEE(origin_, current_, xd, T_7A_, retreat_force_, time.toSec(), arm.task_start_time_.toSec(), duration_, repeatition_);
    f_star = T_WA_.linear()*f_star;

    if(t_e > (repeatition_ + 1)*duration_)
    {
      changed_task_time_ = time.toSec();
      std::cout<<"t_e: "<<t_e<<std::endl;
      std::cout<<"(repeatition_ + 1)*duration_ : "<< (repeatition_ + 1)*duration_<<std::endl; 
      repeatition_ ++;
    }
  }

  //######## Orientation ########
  
  if(wiggle_motion_ && wiggle_motion_z_axis_)
  {
    m_wig = PegInHole::generateWiggleMotionEE(origin_, current_, T_7A_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec(), 500);
    m_wig_z = PegInHole::generateWiggleMotionEE_Zaxis(origin_, current_, T_7A_, xd, wiggle_angle_, wiggle_angular_vel_,
                                                      wiggle_z_axis_param_.a, wiggle_z_axis_param_.b, wiggle_z_axis_param_.t_offset,
                                                      time.toSec(), arm.task_start_time_.toSec());
    m_star = m_wig + m_wig_z;
    m_star = T_WA_.linear()*m_star;
  }
  else if(wiggle_motion_)
  {
    m_star = PegInHole::generateWiggleMotionEE(origin_, current_, T_7A_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec(), 500);
    m_star = T_WA_.linear()*m_star;
  }
  else if(wiggle_motion_z_axis_)
  {
    m_star = PegInHole::generateWiggleMotionEE_Zaxis(origin_, current_, T_7A_, xd, wiggle_angle_, wiggle_angular_vel_,
                                                    wiggle_z_axis_param_.a, wiggle_z_axis_param_.b, wiggle_z_axis_param_.t_offset, 
                                                    time.toSec(), arm.task_start_time_.toSec());
    m_star = T_WA_.linear()*m_star;
  }
  else if(reverse_moment_)
  {
    Eigen::Vector3d p, r_error;
    p = current_.linear()*T_7A_.translation();
    // m_star = -arm.f_ext_.tail<3>() + 2*p.cross(arm.f_ext_.head<3>()) + 100*r_error;
    m_star = 10*(p.cross(arm.f_ext_.head<3>()) - arm.f_ext_.tail<3>()) - 1.0*xd.tail<3>();
    
    // m_star = -0.8*arm.f_ext_.tail<3>() - 10*xd.tail<3>();
    // m_star.setZero();
    std::cout<<"==================="<<std::endl;
    std::cout<<"relative position :"<<p.transpose()<<std::endl;
    std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
    std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
    std::cout<<"m_ext : "<<arm.f_ext_.tail<3>().transpose()<<std::endl;
    std::cout<<"cross : "<<2*p.cross(arm.f_ext_.head<3>()).transpose()<<std::endl;
    std::cout<<"r_error: "<<r_error.transpose()<<std::endl;
    std::cout<<"run time : "<< run_time<<std::endl;
  }
  else
  {
    m_star = keepCurrentOrientation(origin_, current_ , xd, 2000, 15);    
  }  

  // m_star.setZero();

  if(mode_ != 0 && run_time <= holding_time)
  {
    f_star = -0.3*arm.f_ext_.head<3>() - 20*xd.head<3>();
    // m_star = -0.4*arm.f_ext_.tail<3>() - 10*xd.tail<3>();

    Eigen::Vector3d p;    
    p = current_.linear()*T_7A_.translation();
    // m_star = -arm.f_ext_.tail<3>() + 2*p.cross(arm.f_ext_.head<3>());
    m_star = 10*(p.cross(arm.f_ext_.head<3>()) - arm.f_ext_.tail<3>()) - 1.0*xd.tail<3>();
  }

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  
  tau_measured_ = arm.tau_desired_read_;
  tau_ext_ = arm.tau_ext_filtered_;
  tau_measured << tau_measured_.transpose()<<std::endl;
  tau_ext << tau_ext_.transpose()<<std::endl;
  reaction_force << arm.f_ext_.transpose()<<std::endl;
  command_force << f_star_zero.transpose()<<std::endl;

  // f_star_zero.setZero();  
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_ * f_star_zero;
  arm.setTorque(desired_torque);
  
  return true;
}

void AssembleRetreatBoltActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleRetreatBoltActionServer::setAborted()
{
  as_.setAborted(result_);
  control_running_ = false;
}
