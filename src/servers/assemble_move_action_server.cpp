#include <assembly_dual_controllers/servers/assemble_move_action_server.h>

AssembleMoveActionServer::AssembleMoveActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleMoveActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleMoveActionServer::preemptCallback, this));
  as_.start();
}

void AssembleMoveActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleMove goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleMoveActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  

  f_measured_.setZero();

  origin_ = mu_[goal_->arm_name]->position_;  
  init_rot_ = mu_[goal_->arm_name]->rotation_;
  
  target_(0) = goal_ ->p.x;
  target_(1) = goal_ ->p.y;
  target_(2) = goal_ ->p.z;  
  
  type_ = goal_ ->mode;
  dir_ = goal_ ->dir;
  option_ = goal_ ->option;

  is_first_ = true;

}

void AssembleMoveActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool AssembleMoveActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleMoveActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleMoveActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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
  
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Matrix<double, 6, 1> f_star_zero;
  double speed;
  double dis;

  if(is_first_ == true)
  {
    origin_ = position;
    init_rot_ = rotation;
    arm.task_start_time_ = time;
    dis = getDistance(target_, origin_, type_, dir_);
    speed = 0.01;
    duration_ = dis/speed;

    is_first_ = false;
    std::cout<<"origin_: "<<origin_.transpose()<<std::endl;
    std::cout<<"distance: "<<dis<<std::endl;
    std::cout<<"duration: "<<duration_<<std::endl;
    std::cout<<"Go to the target position!"<<std::endl;
  }

  if(time.toSec() - arm.task_start_time_.toSec() > duration_)
  {
    std::cout<<"Arrived!!"<<std::endl;
    as_.setSucceeded();
  }

  if(type_ == 1)
  {
    f_star = oneDofMove(origin_, position, target_(dir_), xd, time.toSec(), arm.task_start_time_.toSec(), duration_, 0.0, dir_);
    if(option_ == 0)  m_star = keepOrientationPerpenticular(init_rot_, rotation, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());  
    if(option_ == 1)  m_star = keepOrientationPerpenticularOnlyXY(init_rot_, rotation, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());
    if(option_ == 2)  m_star = keepCurrentState(origin_, init_rot_, position, rotation, xd, 5000, 100).tail<3>();
    
  }

  if(type_ == 2)
  {
    f_star = twoDofMove(origin_, position, target_, xd, time.toSec(), arm.task_start_time_.toSec(), duration_, 0.0, dir_);
    if(option_ == 0)  m_star = keepOrientationPerpenticular(init_rot_, rotation, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());  
    if(option_ == 1)  m_star = keepOrientationPerpenticularOnlyXY(init_rot_, rotation, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());
    if(option_ == 2)  m_star = keepCurrentState(origin_, init_rot_, position, rotation, xd, 5000, 100).tail<3>();
  }


  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  return true;
}

double AssembleMoveActionServer::getDistance(const Eigen::Vector3d target, const Eigen::Vector3d start, 
  const int type, const int dir)
{
  double dis;
  Eigen::Vector3d dx;

  dx << target(0) - start(0), target(1) - start(1), target(2) - start(2); 
  
  if(type == 1)
  {
    dis = sqrt(pow(dx(dir),2));
  }

  if(type == 2)
  {
    dx(dir) = 0;
    dis = sqrt(pow(dx(0),2) + pow(dx(1),2) + pow(dx(2),2));
  }

  return dis;  
}