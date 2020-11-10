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
  
  control_running_ = true;

  verify_origin_ = T_WA_.translation();

  search_dir_<< 1, 1; // search along +x/+y direction first

  is_mode_changed_ = false;
  search_index_ = 0;
  detect_object_.setZero();
  cnt_ = 0;
  state_ = STATE::READY;
  //--------------------------------
  
  
  
  threshold_ = goal_ ->threshold;
  search_range_ = goal_->search_range;
  search_duration_ = goal_->search_duration;
   

  std::cout<<"T_7A_: \n"<<T_7A_.matrix()<<std::endl;
  std::cout<<"T_WA_: \n"<<T_WA_.matrix()<<std::endl;
  std::cout<<"T_WE_: \n"<<origin_.linear()<<std::endl;
  // std::cout << "verify_origin: " << T_WA_.translation().transpose() << std::endl;
  // std::cout << "origin_: " << origin_.translation().transpose() << std::endl;
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

bool AssembleVerifyActionServer::printDebugState(ros::Time time)
{
  std::cout<<"is_control_changed:"<<is_mode_changed_<<std::endl;
  std::cout<<"current state:\t"<<state_<<std::endl;
  std::cout<<"previous state:\t"<<prev_state_<<std::endl;
  std::cout<<"current time: \t"<<time.toSec()<<std::endl;
  
  return true;
}

bool AssembleVerifyActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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
  
  Eigen::Vector3d f_star, m_star, f_detection;
  Eigen::Vector3d force_compensation;
  Eigen::Vector6d f_star_zero, f_ext;
  double run_time, backward_dist;
  int cnt_start, cnt_max;
  cnt_start = 100;
  cnt_max = 150;

  f_ext = arm.f_ext_;
  current_ = arm.transform_;

  if(is_mode_changed_)
  {
    arm.task_start_time_ = time;
    origin_ = arm.transform_;
    is_mode_changed_ = false;
    cnt_ = 0;
    if(state_ == FORWARD)
    {
      target_.setZero();
      target_(search_index_) = search_dir_(search_index_) * search_range_; //EX [0.1, 0], update the search direction
      std::cout<<"forward target: "<< target_.transpose()<<std::endl;
    } 
    else if(state_ == BACKWARD)  std::cout<<"BACKWARD"<<std::endl;    
  }
  
  run_time = time.toSec() - arm.task_start_time_.toSec();

  switch (state_)
  {
    case READY:
      
      if (cnt_ >= cnt_max)
      {
        std::cout << "cnt is max" << std::endl;
        is_mode_changed_ = true;
        state_ = FORWARD;
        std::cout<<"force compensation: "<< accumulated_wrench_.head<3>().transpose()<<std::endl;
        force_compensation = accumulated_wrench_.head<3>();
        break;
      }

      if (cnt_ > cnt_start)   PegInHole::getCompensationWrench(accumulated_wrench_, f_ext, cnt_start, cnt_, cnt_max);
      cnt_++;
      f_star = PegInHole::keepCurrentPosition(origin_, current_, xd, 3000, 100);      
      break;

    case FORWARD:

      f_detection = f_ext.head<3>() - accumulated_wrench_.head<3>();

      if(run_time > search_duration_)
      //if (timeOut(time.toSec(), arm.task_start_time_.toSec(), search_duration_))
      {
        is_mode_changed_ = true;
        detect_object_(search_index_) = 0; //no object
        state_ = BACKWARD;

        target_.setZero();
        target_(search_index_) = search_dir_(search_index_) * (-search_range_);
        std::cout << "no object in this direction" <<std::endl;
        break;
      }
      else if (run_time > 0.1 && detectObject(f_detection, T_WA_, threshold_))
      {
        is_mode_changed_ = true;
        detect_object_(search_index_) = 1; //there exist object
        state_ = BACKWARD;

        target_.setZero();
        target_(search_index_) = search_dir_(search_index_) * (-search_range_ / 1.5);
        std::cout << "detect an object in this direction"<<std::endl;
        break;
      }

      f_star = PegInHole::threeDofMoveEE(origin_, current_, target_, xd, T_7A_, time.toSec(), arm.task_start_time_.toSec(), search_duration_, 3000, 50);
      f_star = T_WA_.linear() * (f_star) + force_compensation;
      break;

    case BACKWARD:
      f_detection = f_ext.head<3>() - accumulated_wrench_.head<3>();

      if (timeOut(time.toSec(), arm.task_start_time_.toSec(), search_duration_ + 0.005))
      {
        is_mode_changed_ = true;
        search_index_++;
        state_ = READY;
        std::cout << "back to the target position" << std::endl;
      }
      else if (run_time > 0.1 && detectObject(f_detection, T_WA_, threshold_))
      {
        is_mode_changed_ = true;
        search_index_++;
        state_ = READY;
        std::cout << "detect object during backward" << std::endl;
      }

      detectObjectLocation();
      
      f_star = PegInHole::threeDofMoveEE(origin_, current_, target_, xd, T_7A_, time.toSec(), arm.task_start_time_.toSec(), search_duration_, 3000, 50);
      f_star = T_WA_.linear() * (f_star) + force_compensation;
      break;
  }

  m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd,  200, 5);

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  // f_star_zero.setZero();
  
  // if(state_ == BACKWARD) f_star_zero.setZero();

  // Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_*f_star_zero;
  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() *f_star_zero;

  arm.setTorque(desired_torque);

  verify_pr_data << current_.translation().transpose() << std::endl;
  // if(cnt_ < cnt_max)  verify_ft_data << (f_ext.head<3>()).transpose() << std::endl;
  // else verify_ft_data << f_detection.transpose() << std::endl;

  if(state_ == FORWARD) verify_ft_data<<(T_WA_.linear().inverse()*f_detection).transpose()<<std::endl;
  
  prev_state_ = state_;
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

void AssembleVerifyActionServer::detectObjectLocation()
{

  if (search_dir_(0) == 1 && search_index_ == 2)
  {
    int sum = 0;
    sum = detect_object_(0) + detect_object_(1); //detect_objet_ = [0, 1], [1, 0], [1, 1]

    if (sum == 2)
    {
      setSucceeded();
      std::cout << "DETECT HOLE!!" << std::endl;
    }
    else if (sum == 1)
    {
      result_.object_location.data.push_back(detect_object_(0) * search_dir_(0));
      result_.object_location.data.push_back(detect_object_(1) * search_dir_(1));
      std::cout << "search result: " << detect_object_.transpose() << std::endl;
      std::cout << "FAIL" << std::endl;
      setAborted();
    }
    else
    {
      std::cout << "Change search direction and search again" << std::endl;
      search_index_ = 0;
      is_mode_changed_ = true;
      state_ = FORWARD;
      search_dir_ << -1.0, -1.0;
    }
  }
  else if (search_dir_(0) == -1 && search_index_ == 1)
  {
    if (detect_object_(0) == 1)
    {
      result_.object_location.data.push_back(detect_object_(0) * search_dir_(0));
      result_.object_location.data.push_back(detect_object_(1) * search_dir_(1));
      std::cout << "search result: " << detect_object_.transpose() << std::endl;
      std::cout << "FAIL" << std::endl;
      setAborted();
    }
  }
  else if (search_dir_(0) == -1 && search_index_ == 2)
  {
    if (detect_object_(1) == 1)
    {
      result_.object_location.data.push_back(detect_object_(0) * search_dir_(0));
      result_.object_location.data.push_back(detect_object_(1) * search_dir_(1));
      std::cout << "search result: " << detect_object_.transpose() << std::endl;
      std::cout << "FAIL" << std::endl;
      setAborted();
    }
    else
    {
      result_.object_location.data.push_back(detect_object_(0) * search_dir_(0));
      result_.object_location.data.push_back(detect_object_(1) * search_dir_(1));
      std::cout << "No objects nearby" << std::endl;
      setAborted();
    }
  }
}
