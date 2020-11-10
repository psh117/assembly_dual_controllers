#include <assembly_dual_controllers/servers/assemble_approach_hip_action_server.h>

AssembleApproachHipActionServer::AssembleApproachHipActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleApproachHipActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleApproachHipActionServer::preemptCallback, this));
  as_.start();
}

void AssembleApproachHipActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleApproachHip goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleApproachHipActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  origin_ = mu_[goal_->arm_name]->initial_transform_;
  contact_force_rotate_ = goal_->contact_force_rotate;
  contact_force_translate_ = goal_->contact_force_translate;
  time_limit_ = goal_ ->time_limit;
  rotate_dir_ = goal_->rotate_dir;
  translate_dir_ = goal_->translate_dir;
  translation_dir_.setZero();
  translation_dir_(translate_dir_) = 1;
  rotation_vel_ = goal_->rotation_vel;
  translation_vel_ = goal_ -> translation_vel;
  action_type_ = goal_->action_type;

  for(int i = 0; i < 3; i++)
  {
    compliant_rotation_axis_(i) = goal_->compliant_rotation_axis[i];
    compliant_translation_axis_(i)= goal_->compliant_translation_axis[i];
  }

  compliant_rotation_select_mtx_ = Eigen::Matrix3d::Identity();
  compliant_translation_select_mtx_ = Eigen::Matrix3d::Identity();

  for(int i = 0; i < 3; i ++)
  {
    if(compliant_rotation_axis_(i) == 1)    compliant_rotation_select_mtx_(i,i) = 0.0;
    if(compliant_translation_axis_(i) == 1) compliant_translation_select_mtx_(i,i) = 0.0;
  }
  
  flange_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  flange_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  flange_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  flange_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  flange_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  flange_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  flange_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;

  //TODO : delete T_WD_, T_AD_
  T_7A_.linear() = flange_to_assembly_quat_.toRotationMatrix(); // from the flange frame to the assembly frame
  T_7A_.translation() = flange_to_assembly_point_;

  T_WA_ = origin_*T_7A_;

  is_mode_changed_ = false;
  
  control_running_ = true;

  std::cout<<"APPROACH HIP START"<<std::endl;
  std::cout<<"T_7A_: \n"<< T_7A_.matrix()<<std::endl;
  std::cout<<"contact_force_rotate_       : "<<contact_force_rotate_<<std::endl;
  std::cout<<"contact_force_translate_    : "<<contact_force_translate_<<std::endl;
  std::cout<<"compliant_rotation_axis_    : "<<compliant_rotation_axis_.transpose()<<std::endl;
  std::cout<<"compliant_translation_axis_ : "<<compliant_translation_axis_.transpose()<<std::endl;
  std::cout<<"compliant_translation_select: \n"<<compliant_translation_select_mtx_<<std::endl;
  std::cout<<"compliant_rotation_select   : \n"<<compliant_rotation_select_mtx_<<std::endl;
  accumulated_wrench_.setZero();
  count_ = 0;
  
}

void AssembleApproachHipActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleApproachHipActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleApproachHipActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleApproachHipActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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
  
  Eigen::Vector3d f_star, m_star, force;
  Eigen::Vector3d f_d, m_d;
  Eigen::Vector6d f_star_zero, f_star_zero_d, f_ext;
  double run_time;
  
  f_ext = arm.f_ext_;
  current_ = arm.transform_;

  int cnt_max = 200;
  int cnt_start = 100;
  if(count_ < cnt_max)
  { 
    f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 500, 10, 500, 15).head<3>(); //w.r.t {W}
    m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 500, 10, 500, 15).tail<3>();
    f_d.setZero();
    m_d.setZero();
    if(count_ > cnt_start) PegInHole::getCompensationWrench(accumulated_wrench_, f_ext, cnt_start, count_, cnt_max);    
    count_++;
    if(count_ >= cnt_max)
    {
      count_ = cnt_max;
      accumulated_wrench_a_.head<3>() = T_WA_.linear().inverse() * accumulated_wrench_.head<3>();
      accumulated_wrench_a_.tail<3>() = T_WA_.linear().inverse() * accumulated_wrench_.tail<3>();
      arm.task_start_time_ = time;
      std::cout << "output for compensation: " << accumulated_wrench_.transpose() << std::endl;
    } 
  }
  else
  {
    switch (action_type_)
    {
    case 0: // rotate motion
    
      run_time = time.toSec() - arm.task_start_time_.toSec();
      force = f_ext.head<3>() - accumulated_wrench_.head<3>();
      
      if (timeOut(time.toSec(), arm.task_start_time_.toSec(), time_limit_))
      {
        std::cout<<"TIME OUT!!"<<std::endl;
        setAborted();
        break;
      }

      if (run_time > 0.5 && Criteria::checkForceLimit(force, contact_force_rotate_))
      {
        std::cout << "CHECK CONTATCT!!!!!" << std::endl;
        setSucceeded();
        break;
      }

      f_star = PegInHole::keepCurrentPosition(origin_, current_, xd, 800, 20);
      f_star = compliant_translation_select_mtx_*f_star;
      f_d.setZero();

      m_star = PegInHole::oneDofRotation(origin_, current_, xd, f_ext.tail<3>() ,rotation_vel_, rotate_dir_, time.toSec(), arm.task_start_time_.toSec(), 2000, 20);
      m_star = compliant_rotation_select_mtx_*m_star;
      m_d.setZero();
      
      rotate_fm_data << force.transpose()<<std::endl;
      break;

    case 1: // translation motion
      run_time = time.toSec() - arm.task_start_time_.toSec();
      force = f_ext.head<3>() - accumulated_wrench_.head<3>();

      if (timeOut(time.toSec(), arm.task_start_time_.toSec(), time_limit_))
      {
        std::cout<<"TIME OUT!!"<<std::endl;
        setAborted();
        break;
      }
      if (run_time > 0.5 && Criteria::checkForceLimit(force, contact_force_translate_))
      {
        std::cout << "CHECK CONTATCT!!!!!" << std::endl;
        setSucceeded();
        break;
      }

      f_star = PegInHole::straightMotion(origin_, current_, xd, translation_dir_, translation_vel_, time.toSec(), arm.task_start_time_.toSec(), 800, 20);
      f_d.setZero();
      f_star = compliant_translation_select_mtx_*f_star;

      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 2000, 15);
      m_d.setZero();
      m_star = compliant_rotation_select_mtx_*m_star;

      translate_fm_data << force.transpose()<<std::endl;
    }
  }

  
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  f_star_zero_d.head<3>() = f_d;
  f_star_zero_d.tail<3>() = m_d;

  // std::cout<<"-------------------"  <<std::endl;
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  // std::cout<<"f_d   : "<<f_d.transpose()<<std::endl;
  // std::cout<<"m_d   : "<<m_d.transpose()<<std::endl;
  // std::cout<<"m_ext : "<<f_ext.tail<3>().transpose()<<std::endl;
  // f_star_zero.setZero();    
  
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() *(arm.modified_lambda_matrix_* f_star_zero + f_star_zero_d);
  
  arm.setTorque(desired_torque);

  return true;
}

void AssembleApproachHipActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleApproachHipActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}