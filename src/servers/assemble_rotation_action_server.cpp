#include <assembly_dual_controllers/servers/assemble_rotation_action_server.h>

AssembleRotationActionServer::AssembleRotationActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleRotationActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleRotationActionServer::preemptCallback, this));
  as_.start();
}

void AssembleRotationActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleRotation goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleRotationActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  mu_[goal_->arm_name]->task_end_time_ = ros::Time(mu_[goal_->arm_name]->task_start_time_.toSec() + 3.0);

  f_measured_.setZero();

  pressing_force_ = goal_->pressing_force; 
  range_ = goal_ ->range*M_PI/180;
  f_threshold_ = goal_ ->f_threshold;
  duration_ = goal_->duration;

  origin_ = mu_[goal_->arm_name]->transform_;  

  is_first_ = false;

  flange_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  flange_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  flange_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  flange_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  flange_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  flange_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  flange_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;
  
  flange_to_pivot_point_(0) = goal_->ee_to_pivot.position.x;
  flange_to_pivot_point_(1) = goal_->ee_to_pivot.position.y;
  flange_to_pivot_point_(2) = goal_->ee_to_pivot.position.z;
  flange_to_pivot_quat_.x() = goal_->ee_to_pivot.orientation.x;
  flange_to_pivot_quat_.y() = goal_->ee_to_pivot.orientation.y;
  flange_to_pivot_quat_.z() = goal_->ee_to_pivot.orientation.z;
  flange_to_pivot_quat_.w() = goal_->ee_to_pivot.orientation.w;

  T_7A_.linear() = flange_to_assembly_quat_.toRotationMatrix();
  T_7A_.translation() = flange_to_assembly_point_;
  T_7P_.linear() = flange_to_pivot_quat_.toRotationMatrix();
  T_7P_.translation() = flange_to_pivot_point_;

  T_WA_ = origin_*T_7A_;
  T_WP_ = origin_*T_7P_;

  count_ = 0;
  accumulated_wrench_.setZero();
  accumulated_wrench_a_.setZero();
  control_running_ = true;
}

void AssembleRotationActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleRotationActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleRotationActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleRotationActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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
  
  Eigen::Vector3d delphi_delta, f_star, m_star;
  Eigen::Vector6d f_ext, f_ext_a, f_star_zero;
  double dis;
  double run_time;  
  double friction, m_sum;
  int cnt_max = 150;
  int cnt_start = 50;

  current_ = arm.transform_;

  dis = origin_.translation()(2) - current_.translation()(2);
  run_time = time.toSec() - arm.task_start_time_.toSec();

  if(is_first_ == true)
  {
    arm.task_start_time_ = time;
    is_first_ = false;
    origin_ = arm.transform_;
    std::cout<<"ROTATE TO SEARCH"<<std::endl;
  }

  if(count_ < cnt_max)
  { 
    f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 700, 40, 2000, 15).head<3>(); //w.r.t {W}
    m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 700, 40, 2000, 15).tail<3>();

    if(count_ > cnt_start) PegInHole::getCompensationWrench(accumulated_wrench_, arm.f_ext_, cnt_start, count_, cnt_max);    
    count_++;
    if(count_ >= cnt_max)
    {
      count_ = cnt_max;
      accumulated_wrench_a_.head<3>() = T_WA_.linear().inverse() * accumulated_wrench_.head<3>();
      accumulated_wrench_a_.tail<3>() = T_WA_.linear().inverse() * accumulated_wrench_.tail<3>();
      std::cout << "output for compensation: " << accumulated_wrench_.transpose() << std::endl;
      // heavy_mass_ = Criteria::holdHeavyMass(accumulated_wrench_.head<3>(), 8.0);
      is_first_ = true;
    } 
  }
  else
  {
    f_ext = arm.f_ext_ - accumulated_wrench_;
    f_ext_a.head<3>() = T_WA_.linear().inverse()*f_ext.head<3>() - accumulated_wrench_a_.head<3>();
    f_ext_a.tail<3>() = T_WA_.linear().inverse() * f_ext.tail<3>() - accumulated_wrench_a_.tail<3>();

    m_sum = sqrt(pow(f_ext_a(3), 2) + pow(f_ext_a(4), 2) + pow(f_ext_a(5), 2));
    friction = sqrt(pow(f_ext_a(0), 2) + pow(f_ext_a(1), 2));

    if(run_time > 0.05 && friction >= 5.0 && m_sum > f_threshold_)
    //if(run_time > 0.05 && m_sum > f_threshold_)
    {
      std::cout<<"HOLE IS DETECTED"<<std::endl;
      std::cout<<"m_sum(2): "<<m_sum<<std::endl;
      std::cout<<"friction(2): "<<friction<<std::endl;
      setSucceeded();
    }
    if (timeOut(time.toSec(), arm.task_start_time_.toSec(), duration_)) //duration wrong??
    {
      std::cout << "Time out" << std::endl;
      setAborted();
    }
    Eigen::Vector3d rot_axis, f_press, m_press;
    rot_axis << 0, 0, 1.0;

    f_press << 0, 0, pressing_force_;
    f_star = PegInHole::tiltMotion(origin_, current_, xd, T_7P_, rot_axis, range_, time.toSec(), arm.task_start_time_.toSec(), duration_).head<3>();
    f_star = f_star + T_WA_.linear() * f_press;

    m_press = T_7A_.translation().cross(f_press);
    m_star = PegInHole::tiltMotion(origin_, current_, xd, T_7P_, rot_axis, range_, time.toSec(), arm.task_start_time_.toSec(), duration_).tail<3>();
    m_star = 0.5*m_star + current_.linear() * m_press;
  }

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_*f_star_zero;
  // f_star_zero.setZero();

  arm.setTorque(desired_torque);  

  fm_rotation_search << f_ext_a.transpose() << std::endl;
  pr_rotation_search << current_.translation().transpose() << std::endl;  

  return true;
}


void AssembleRotationActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleRotationActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}
