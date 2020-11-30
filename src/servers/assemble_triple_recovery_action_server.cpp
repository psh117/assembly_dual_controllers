#include <assembly_dual_controllers/servers/assemble_triple_recovery_action_server.h>

AssembleTripleRecoveryActionServer::AssembleTripleRecoveryActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleTripleRecoveryActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleTripleRecoveryActionServer::preemptCallback, this));
  as_.start();
}

void AssembleTripleRecoveryActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleTripleRecovery goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleTripleRecoveryActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  mu_[goal_->arm_name]->task_end_time_ = ros::Time(mu_[goal_->arm_name]->task_start_time_.toSec() + 3.0);
  
  flange_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  flange_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  flange_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  flange_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  flange_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  flange_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  flange_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;

  target_pose_position_(0) = goal_->recovery_target.position.x;
  target_pose_position_(1) = goal_->recovery_target.position.y;
  target_pose_position_(2) = goal_->recovery_target.position.z;
  target_pose_quat_.x() = goal_->recovery_target.orientation.x;
  target_pose_quat_.y() = goal_->recovery_target.orientation.y;
  target_pose_quat_.z() = goal_->recovery_target.orientation.z;
  target_pose_quat_.w() = goal_->recovery_target.orientation.w;

  tilt_back_threshold_ = goal_->tilt_back_threshold;
  recovery_angle_ = goal_->recovery_angle;
  origin_ = mu_[goal_->arm_name]->transform_;

  duration_ = goal_->duration;

  T_7A_.linear() = flange_to_assembly_quat_.toRotationMatrix();
  T_7A_.translation() = flange_to_assembly_point_;
  T_WA_ = origin_*T_7A_;

  target_.linear() = target_pose_quat_.toRotationMatrix();
  target_.translation() = target_pose_position_;

  Eigen::Vector3d recover_dir;
  double offset_angle;
  Eigen::Isometry3d T_aa;
  recover_dir = (T_WA_.linear().inverse()*(origin_.translation().cross(target_.translation())));
  recover_dir = recover_dir/recover_dir.norm();
  if(recover_dir(2) < 0) offset_angle = -2.0*DEG2RAD;
  else offset_angle = 2.0*DEG2RAD;
  T_aa.Identity();
  T_aa.linear() = dyros_math::rotateWithZ(offset_angle);
  target_pose_position_ = T_WA_*T_aa*T_WA_.inverse()*target_pose_position_;
  // std::cout<<"origin target position w.r.t {A} : "<<(T_WA_.inverse()*target_pose_position_).transpose()<<std::endl;
  // std::cout<<"renew  target position w.r.t {A} : "<<(T_aa*T_WA_.inverse()*target_pose_position_).transpose()<<std::endl;
  std::cout<<"rotation direction : "<<recover_dir.transpose()<<std::endl;

  state_ = MOVE;

  is_escape_first_ = true;
  is_move_first_ = true;
  is_approach_first_ = true;
 
  tilt_axis_ = PegInHole::getTiltDirection(T_7A_);
  count_ = 0;

  control_running_ = true;

  std::cout<<"TRIPLE RECOVERY START"<<std::endl;
  std::cout<<"return target after: "<<target_pose_position_.transpose()<<std::endl;
}

void AssembleTripleRecoveryActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleTripleRecoveryActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleTripleRecoveryActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleTripleRecoveryActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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
  
  Eigen::Vector3d f_star, m_star, f_contact;    
  Eigen::Vector6d f_star_zero, f_ext, f_tilt;
  double radius;
  double run_time;

  f_ext = arm.f_ext_;
  current_ = arm.transform_;
  
  int cnt_max = 300;
  int cnt_start = 250;
  if(count_ < cnt_max)
  { 
    f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 700, 40, 2000, 15).head<3>(); //w.r.t {W}
    m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 700, 40, 2000, 15).tail<3>();
    if(count_ > cnt_start) PegInHole::getCompensationWrench(accumulated_wrench_, f_ext, cnt_start, count_, cnt_max);    
    count_++;
    if(count_ >= cnt_max)
    {
      count_ = cnt_max;
      accumulated_wrench_.head<3>() = T_WA_.linear().inverse() * accumulated_wrench_.head<3>(); // to convert {A} frame
      accumulated_wrench_.tail<3>() = T_WA_.linear().inverse() * accumulated_wrench_.tail<3>();
      std::cout << "output for compensation: " << accumulated_wrench_.transpose() << std::endl;
    } 
  }
  else
  {
    
  switch(state_)
  {
    case MOVE:
      if(is_move_first_)
      {
        origin_ = arm.transform_;
        arm.task_start_time_ = time;
        is_move_first_ = false;
        std::cout<<"Move"<<std::endl;
      }

      if(timeOut(time.toSec(), arm.task_start_time_.toSec(), duration_+0.25))
      {
        state_ = APPROACH;
        origin_ = arm.transform_;
        count_ = 0;
      } 
      
      radius = T_7A_.translation().norm();
      // f_star = PegInHole::threeDofMove(origin_, current_, target_.translation(), xd, time.toSec(), arm.task_start_time_.toSec(), duration_, 300.0, 10);
      f_star = PegInHole::followSphericalCoordinateEE(origin_, current_, target_, xd, T_7A_, time.toSec(), arm.task_start_time_.toSec(), duration_, radius, 400, 5.0);
      f_star = T_WA_.linear()*f_star;
      m_star = PegInHole::rotateWithMat(origin_, current_, xd, target_.linear(), time.toSec(), arm.task_start_time_.toSec(), duration_);

      break;

    case APPROACH:
      if(is_approach_first_)
      {
        origin_ = arm.transform_;
        arm.task_start_time_ = time;
        is_approach_first_ = false;
        f_contact_init_ = T_WA_.linear().inverse()*f_ext.head<3>();
        std::cout<<"Approach"<<std::endl;
        std::cout<<"recovery_angle: "<<recovery_angle_*RAD2DEG<<std::endl;
        std::cout<<"tilt_axis: "<<tilt_axis_.transpose()<<std::endl;
        std::cout<<"f_contact_init : "<< f_contact_init_.transpose()<<std::endl;
      }
      
      f_contact = T_WA_.linear().inverse()*f_ext.head<3>() - f_contact_init_;
      // f_contact = T_WA_.linear().inverse() * (f_ext.head<3>() - f_star);
      triple_recovery_fm_data << f_contact.transpose()<<std::endl;

      run_time = time.toSec() - arm.task_start_time_.toSec();
      if(timeOut(time.toSec(), arm.task_start_time_.toSec(), duration_+0.25))
      {
        std::cout<<"TILT BACK DONE"<<std::endl;
        setSucceeded();
      }

      if(run_time > 0.5 && Criteria::checkForceLimit(f_contact, tilt_back_threshold_))
      {
        std::cout << "TILT BACK REACHED FORCE LIMIT" << std::endl;
        std::cout<<"tilt_back_threshold_: "<<tilt_back_threshold_<<std::endl;
        std::cout<<"f_contact: "<<f_contact.transpose()<<std::endl;
        setSucceeded();
      }

      f_tilt = PegInHole::tiltMotion(origin_, current_, xd, T_7A_, tilt_axis_, recovery_angle_, time.toSec(), arm.task_start_time_.toSec(), duration_, 400, 10);
      f_star = f_tilt.head<3>();
      m_star = f_tilt.tail<3>();
      break;
    }
  }
  
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;

  // if(state_ == APPROACH) f_star_zero.setZero();
  // f_star_zero.setZero();

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_*f_star_zero;
  arm.setTorque(desired_torque);
  
  return true;
}

void AssembleTripleRecoveryActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleTripleRecoveryActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}