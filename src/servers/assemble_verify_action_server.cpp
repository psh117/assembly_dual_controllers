#include <assembly_dual_controllers/servers/assemble_verify_action_server.h>

AssembleVerifyActionServer::AssembleVerifyActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleVerifyActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleVerifyActionServer::preemptCallback, this));
  as_.start();
}

void AssembleVerifyActionServer::goalCallback()
{ 
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleVerify goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleVerifyActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  
  origin_ = mu_[goal_->arm_name]->transform_;  

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
  
  if(verify_pr_data.is_open())  verify_pr_data.close();
  if(verify_ft_data.is_open()) verify_ft_data.close();    
  verify_pr_data.open("verify_pr_data.txt");
  verify_ft_data.open("verify_ft_data.txt");

  control_running_ = true;

  spiral_origin_(0) = goal_->spiral_origin.position.x;
  spiral_origin_(1) = goal_->spiral_origin.position.y;
  spiral_origin_(2) = goal_->spiral_origin.position.z;

  
  verify_origin_ = T_WA_.translation();
  
  search_dir_ = setSearchDirection(spiral_origin_, verify_origin_, T_WA_);
  
  is_mode_changed_ = true;
  search_index_ = 0;
  detect_object_.setZero();
  cnt_ = 0;
  state_ = STATE::READY;
  //--------------------------------
  threshold_ = goal_ ->threshold;
  search_range_ = goal_->search_range;
  search_duration_ = goal_->search_duration;
  
  // std::cout<<"T_EA_: \n"<<T_EA_.matrix()<<std::endl;
  std::cout << "spiral_origin: " << spiral_origin_.transpose() << std::endl;
  std::cout << "verify_origin: " << T_WA_.translation().transpose() << std::endl;
  std::cout << "origin_: " << origin_.translation().transpose() << std::endl;
  std::cout << "search dir: " << search_dir_.transpose() << std::endl;
  std::cout<<"detection threshold: "<<threshold_<<std::endl;
}

void AssembleVerifyActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleVerifyActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleVerifyActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}

bool AssembleVerifyActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
      return false; 

  auto & mass = arm.mass_matrix_;
  auto & rotation = arm.rotation_;
  auto & position = arm.position_;
  auto & jacobian = arm.jacobian_;
  auto & tau_measured = arm.tau_measured_;
  auto & gravity = arm.gravity_;
  auto & xd = arm.xd_; //velocity
  
  Eigen::Vector3d f_star, m_star, f_detection;
  Eigen::Vector6d f_star_zero, f_ext;
  double run_time;
  int cnt_start, cnt_max;
  cnt_start = 100;
  cnt_max = 150;

  f_ext = arm.f_ext_;
  current_ = arm.transform_;
  run_time = time.toSec() - arm.task_start_time_.toSec();

  if(is_mode_changed_)
  {
    arm.task_start_time_ = time;
    origin_ = arm.transform_;
    is_mode_changed_ = false;
        
    if(state_ == FORWARD)       target_(search_index_) = search_dir_(search_index_) * search_range_; //EX [0.1, 0], update the search direction
    // else if(state_ == BACKWARD) target_(search_index_) = search_dir_(search_index_) * (-search_range_);
    
  }

  switch (state_)
  {
  case READY:
    f_star = PegInHole::keepCurrentPosition(origin_, current_, xd, 0.0, 0.0);
    
    if(cnt_ > cnt_start) PegInHole::getCompensationWrench(accumulated_wrench_, f_ext, cnt_start, cnt_, cnt_max);
    
    cnt_++;
    
    if(cnt_ >= cnt_max)
    {
      is_mode_changed_ = true;
      state_ = FORWARD;
      cnt_ = cnt_max;
      std::cout<<"initially measured force: "<< accumulated_wrench_.head<3>().transpose()<<std::endl;
    }
    
    break;

  case FORWARD:
    
    f_detection = f_ext.head<3>() - accumulated_wrench_.head<3>();
    
    if(run_time > 0.001 && detectObject(origin_, current_, f_detection, T_WA_, threshold_))
    {
      is_mode_changed_ = true;
      detect_object_(search_index_) = 1; //there exist object
      state_ = BACKWARD;

      target_.setZero();
      target_(search_index_) = (origin_.translation() - current_.translation())(search_index_);
      target_ = origin_.translation() - current_.translation(); // {E} w.r.t {W}     
      
      std::cout<<"target for BACKWORD: "<<target_.transpose()<<std::endl;
    }
    
    if (timeOut(time.toSec(), arm.task_start_time_.toSec(), search_duration_))
    {
      is_mode_changed_ = true;
      detect_object_(search_index_) = 0; //no object
      state_ = BACKWARD;
      
      target_.setZero();
      target_(search_index_) = (origin_.translation() - current_.translation())(search_index_);
      target_ = origin_.translation() - current_.translation(); // {E} w.r.t {W}     
      
      std::cout<<"target for BACKWORD: "<<target_.transpose()<<std::endl;
    }
    
    f_star = PegInHole::threeDofMoveEE(origin_, current_, target_, xd, T_EA_, time.toSec(), arm.task_start_time_.toSec(), search_duration_);
    break;

  case BACKWARD:
    if (timeOut(time.toSec(), arm.task_start_time_.toSec(), search_duration_ + 0.01))
    {
      is_mode_changed_ = true;
      search_index_++;
      state_ = FORWARD;
    }
    
    f_star = PegInHole::threeDofMoveEE(origin_, current_, target_, xd, T_EA_, time.toSec(), arm.task_start_time_.toSec(), search_duration_);
    
    break;
  }

  if (search_index_ == 2) // finish searches along x and y direction
  {
    int sum = 0;
    sum = detect_object_(0) + detect_object_(1); //detect_objet_ = [0, 1], [1, 0], [1, 1]
    result_.object_location.data.push_back(detect_object_(0) * search_dir_(0));
    result_.object_location.data.push_back(detect_object_(1) * search_dir_(1));
    std::cout << "detect_result: " << detect_object_.transpose() << std::endl;
    if (sum == 2)
    {
      setSucceeded();
      std::cout << "DETECT HOLE!!" << std::endl;
    }
    else
    {
      setAborted();
      std::cout << "FAIL" << std::endl;
    }
    f_star = PegInHole::keepCurrentPosition(origin_, current_, xd,  3000, 100);
    std::cout<<"---------------------------"<<std::endl;
    std::cout<<"origin: "<< origin_.translation().transpose()<<std::endl;
    std::cout<<"current_: "<< current_.translation().transpose()<<std::endl;
    std::cout<<"f_star: "<< f_star.transpose()<<std::endl;
  }

  f_star = T_WA_.linear() * f_star;
  m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 200, 5);

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  // f_star_zero.setZero();
  
  // if(state_ == BACKWARD) f_star_zero.setZero();

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  verify_pr_data << current_.translation().transpose() << std::endl;
  if(cnt_ < cnt_max)  verify_ft_data << (f_ext).transpose() << std::endl;
  else verify_ft_data << (f_ext - accumulated_wrench_).transpose() << std::endl;



  return true;
}

void AssembleVerifyActionServer::setSucceeded()
{
  result_.is_completed = true;
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleVerifyActionServer::setAborted()
{
  result_.is_completed = false;
  as_.setAborted(result_);
  control_running_ = false;
}

Eigen::Vector2d AssembleVerifyActionServer::setSearchDirection(const Eigen::Vector3d &spiral_origin, //w.r.t robot
                                                               const Eigen::Vector3d &start_point,     //w.r.t robot
                                                               const Eigen::Isometry3d &T_wa)
{
  Eigen::Vector2d search_dir; //w.r.t {A} frame  
  Eigen::Vector3d p_w, p_a;
  
  // p_w = spiral_origin - start_point;
  p_w = start_point - spiral_origin;
  p_w(2) = 0.0;
  
  p_a = T_wa.linear().inverse()*p_w;

  for(int i = 0; i < 2; i ++)
  {
    if(p_a(i) > 0) search_dir(i) = -1.0;
    else search_dir(i) = 1.0;
  }
  
  std::cout<<"relative position w.r.t {W}: "<<p_w.transpose()<<std::endl;
  std::cout<<"relative position w.r.t {A}: "<<p_a.transpose()<<std::endl;
  return search_dir;
}