#include <assembly_dual_controllers/servers/assemble_kitting_action_server.h>

AssembleKittingActionServer::AssembleKittingActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleKittingActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleKittingActionServer::preemptCallback, this));
  as_.start();
}

void AssembleKittingActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleApproach goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleKittingActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  
  origin_ = mu_[goal_->arm_name]->initial_transform_;
  
  contact_force_ = goal_->contact_force;  
  speed_ = goal_ ->speed;  
  dir_ = goal_->dir;
  kitting_dir_.setZero();
  kitting_dir_(dir_) = 1;

  
  control_running_ = true;

  std::cout<<"KITTING START"<<std::endl;
  
}

void AssembleKittingActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleKittingActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleKittingActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleKittingActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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
  Eigen::Vector6d f_star_zero, f_ext;
  double run_time;
  Eigen::Vector3d force;

  f_ext = arm.f_ext_;
  current_ = arm.transform_;
  
  int cnt_max = 200;
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
      arm.task_start_time_ = time;
      std::cout << "output for compensation: " << accumulated_wrench_.transpose() << std::endl;
    } 
  }
  else
  {
    run_time = time.toSec() - arm.task_start_time_.toSec();
    force = f_ext.head<3>() - accumulated_wrench_.head<3>();

    if (run_time > 0.5 && Criteria::checkForceLimit(force, contact_force_))
    {
      std::cout << "CHECK CONTATCT!!!!!" << std::endl;
      setSucceeded();
    }

    f_star = PegInHole::straightMotion(origin_, current_, xd, kitting_dir_, speed_, time.toSec(), arm.task_start_time_.toSec(), 800, 20);
    m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 2000, 15);

    kitting_fm_data << force.transpose() << std::endl;
  }


  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  
  // f_star_zero.setZero();  
  // std::cout<<"f_measured: "<<f_measured_.transpose()<<std::endl;
  // std::cout<<"f_filtered: "<<f_ext.transpose()<<std::endl;

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() *arm.modified_lambda_matrix_* f_star_zero;
  // std::cout<<"tau_null_cmd : "<< tau_null_cmd_.transpose()<<std::endl;
  arm.setTorque(desired_torque);

  return true;
}

void AssembleKittingActionServer::setSucceeded()
{
  result_.is_completed = true;
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleKittingActionServer::setAborted()
{
  result_.is_completed = false;
  as_.setAborted(result_);
  control_running_ = false;
}