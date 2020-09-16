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

  origin_ = mu_[goal_->arm_name]->transform_;
  
  duration_ = goal_->duration;
  insertion_force_ = goal_->insertion_force;
  std::cout<<"insertion_force: "<<insertion_force_<<std::endl;
  
  wiggle_motion_ = goal_->wiggle_motion;
  wiggle_motion_z_axis_ = goal_->wiggle_motion_z_axis;
  yawing_motion_ = goal_->yawing_motion;

  yawing_angle_ = goal_->yawing_angle * DEG2RAD; //radian
  init_yaw_angle_ = mu_[goal_->arm_name]->q_(6,0); //radian

  std::cout<<goal_->arm_name<<std::endl;
  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"init_yaw_angle: "<<init_yaw_angle_<<std::endl;
  // std::cout<<"init_yaw_goal: "<<yawing_angle_<<std::endl;
  // std::cout<<"sum: "<<yawing_angle_ + init_yaw_angle_<<std::endl;
  
  if (yawing_angle_ + init_yaw_angle_ > yaw_up_limit_) 
  {
    std::cout<<"\nover joint upper limit: "<<yaw_up_limit_<<std::endl;
    yawing_angle_ = yaw_up_limit_ - 0.1 - init_yaw_angle_;
  }
  else if (yawing_angle_ + init_yaw_angle_ < yaw_low_limit_ )
  {
    std::cout<<"\nunder joint lower limit: "<<yaw_low_limit_<<std::endl;
    yawing_angle_ = yaw_low_limit_ + 0.1 - init_yaw_angle_;
  }
  // std::cout<<"\nfinal_yaw_goal: "<<yawing_angle_<<std::endl;
  // std::cout<<"sum: "<<yawing_angle_ + init_yaw_angle_<<std::endl;
  // std::cout<<"-------------------------------------"<<std::endl;

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

  wiggle_angle_ = 5.0*DEG2RAD;
  wiggle_angular_vel_ = 4*M_PI; // 2pi / s

  if(wiggle_motion_ && yawing_motion_) std::cout<<"wiggle & yawing motion is activated"<<std::endl;
  else if(wiggle_motion_) std::cout<<"wiggle motion is activated"<<std::endl;
  else if(yawing_motion_) std::cout<<"yawing motion is activated"<<std::endl;
  else  std::cout<<"no optional motions"<<std::endl;
  
  control_running_ = true;

  Eigen::Isometry3d T_AE;
  Eigen::Vector3d P_ae;
  double dis;
  T_AE = T_EA_.inverse();
  P_ae = T_AE.translation();
  dis = sqrt(P_ae(0)*P_ae(0) + P_ae(1)*P_ae(1));

  if(dis <= 0.005)
  {
    mode_ = SIMPLE;
    std::cout<<"Insert the part in the simple method"<<std::endl;
  } 
  else
  {
    mode_ = COMPLEX;
    std::cout<<"Insert the part in the complex method"<<std::endl;
  } 

  U_EA_ = PegInHole::getNormalVector(T_EA_.translation());
  U_dir_ = T_EA_.linear().col(2);

  std::cout<<"------------------"<<std::endl;

}

void AssembleInsertActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleInsertActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;
    
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

  // auto & mass = arm.mass_matrix_;
  // auto & rotation = arm.rotation_;
  // auto & position = arm.position_;
  auto & jacobian = arm.jacobian_;
  // auto & tau_measured = arm.tau_measured_;
  // auto & gravity = arm.gravity_;
  auto & xd = arm.xd_; //velocity
  
  Eigen::Vector3d f_star, m_star, m_wig, m_yaw;;
  Eigen::Vector3d m_dir; 
  Eigen::Vector6d f_star_zero;
  double run_time;
  
  current_ = arm.transform_;
  run_time = time.toSec() - arm.task_start_time_.toSec();
  if(timeOut(time.toSec(), arm.task_start_time_.toSec(), duration_ +0.01))
  {
    std::cout<<"INSERTION IS DONE"<<std::endl;
    std::cout<<"run time: "<< run_time<<std::endl;
    savePosture();
    setSucceeded();
  }

  // switch (mode_)
  // {
  // case SIMPLE:
  //   f_star = PegInHole::pressEE(insertion_force_); //w.r.t {A}
  //   break;
  
  // case COMPLEX: // Assumption : the location of the end-effector should be higher than the assembly frame.
  //   f_star = PegInHole::keepCurrentPosition(origin_, current_, xd, 5000, 100);
  //   // std::cout<<"f_star w.r.t global frame: \t"<<f_star.transpose()<<std::endl;
  //   f_star = current_.linear().inverse()*f_star;
  //   f_star(2) = 0.0;
  //   f_star = current_.linear()*f_star;
  //   // std::cout<<"f_star w.r.t local frame: \t"<<f_star.transpose()<<std::endl;
  //   m_dir = U_dir_.cross(U_EA_); // w.r.t {E}
  //   m_star = current_.linear()*(insertion_force_*U_dir_);
  //   break;
  // }
  //f_star = PegInHole::pressEE(insertion_force_); //w.r.t {A}

  f_star = PegInHole::pressCubicEE(origin_, current_, xd, T_WA_, insertion_force_, time.toSec(), arm.task_start_time_.toSec(), duration_/2);
  f_star = T_WA_.linear()*f_star;
  
  if(wiggle_motion_ && yawing_motion_)
  {
    m_wig = PegInHole::generateWiggleMotionEE(origin_, current_, T_EA_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec());
    m_yaw = PegInHole::generateYawingMotionEE(origin_, current_, T_EA_, xd, yawing_angle_, duration_, time.toSec(), arm.task_start_time_.toSec());
    m_star = m_wig + m_yaw;
    m_star = T_WA_.linear()*m_star;
  }
  else if(yawing_motion_ )
  {
    m_star = PegInHole::generateYawingMotionEE(origin_, current_, T_EA_, xd, yawing_angle_, duration_, time.toSec(), arm.task_start_time_.toSec());
    m_star = T_WA_.linear()*m_star;
  }
  else if(wiggle_motion_)
  {
    m_star = PegInHole::generateWiggleMotionEE(origin_, current_, T_EA_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec());
    m_star = T_WA_.linear()*m_star;
  }
  else if(wiggle_motion_z_axis_)
  {
    m_star = PegInHole::generateWiggleMotionEE_Zaxis(origin_, current_, T_EA_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec());
    m_star = T_WA_.linear()*m_star;
  }
  else  m_star = keepCurrentOrientation(origin_, current_ , xd, 2000, 15);

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  
  // f_star_zero.setZero();
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_*f_star_zero;
  arm.setTorque(desired_torque);
  
  insert_pose_data << arm.position_<<std::endl;
  // std::cout<<"f_star: "<< f_star.transpose()<<std::endl;
  return true;
}

void AssembleInsertActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleInsertActionServer::setAborted()
{
  as_.setAborted(result_);
  control_running_ = false;
}

void AssembleInsertActionServer::savePosture()
{
  current_;
  
  result_.end_pose.position.x = current_.translation()(0);
  result_.end_pose.position.y = current_.translation()(1);
  result_.end_pose.position.z = current_.translation()(2);
  Eigen::Quaterniond temp(current_.linear());
  result_.end_pose.orientation.x = temp.x();
  result_.end_pose.orientation.y = temp.y();
  result_.end_pose.orientation.z = temp.z();
  result_.end_pose.orientation.w = temp.w();
}