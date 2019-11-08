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
  task_end_time_2_ = ros::Time(mu_[goal_->arm_name]->task_start_time_.toSec() + 6.0);
  task_end_time_3_ = ros::Time(mu_[goal_->arm_name]->task_start_time_.toSec() + 7.0);

  f_measured_.setZero();

  origin_ = mu_[goal_->arm_name]->position_;  
  init_rot_ = mu_[goal_->arm_name]->rotation_;

  target_force_(0) = goal_ ->target_force.force.x;
  target_force_(1) = goal_ ->target_force.force.y;
  target_force_(2) = goal_ ->target_force.force.z;

  assemble_dir_ = goal_ ->assemble_dir;
  
  target_(0) = goal_ ->p.x;
  target_(1) = goal_ ->p.y;
  target_(2) = goal_ ->p.z;  
  
  mode_ = goal_ ->mode;
  dir_ = goal_ ->dir;
  option_ = goal_ ->option;
  //f_threshold_ = goal_ ->f_threshold;

  range_ = goal_ ->range*M_PI/180;
  swing_dir_ = goal_ ->swing_dir;

  is_first_ = true;
  state_ = 0;

  range_1_ = -10*M_PI/180;
  range_2_ = range_;
  rot_1_ << cos(range_1_), 0, sin(range_1_), 0, 1, 0, -sin(range_1_), 0, cos(range_1_);
  rot_2_ << cos(range_2_), -sin(range_2_), 0, sin(range_2_), cos(range_2_), 0, 0, 0, 1;

}

void AssembleTripleRecoveryActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool AssembleTripleRecoveryActionServer::compute(ros::Time time)
{
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
  double speed;
  double dis;

  Eigen::Matrix<double, 6, 1> f_star_zero;
  Eigen::Matrix<double, 6, 6> lambda;
  Eigen::Matrix<double, 7, 6> J_bar;
  
  lambda = (jacobian*mass.inverse()*jacobian.transpose()).inverse();
  J_bar = mass.inverse()*jacobian.transpose()*lambda;
  f_measured_ = J_bar.transpose()*(tau_measured - gravity);


  if(is_first_ == true)
  {
    origin_ = position;
    init_rot_ = rotation;
    arm.task_start_time_ = time;
    dis = getDistance(target_, origin_, mode_, dir_);
    speed = 0.01;
    duration_ = dis/speed;

    is_first_ = false;
    std::cout<<"origin_: "<<origin_.transpose()<<std::endl;
    std::cout<<"distance: "<<dis<<std::endl;
    std::cout<<"duration: "<<duration_<<std::endl;
    std::cout<<"Go to the target position!"<<std::endl;
 
    state_init_rot_1_ = init_rot_ * rot_1_;
    state_init_rot_2_ = init_rot_ * rot_1_ * rot_2_;
  }

  // if(timeOut(time.toSec(), arm.task_start_time_.toSec(), 10.0)) //duration wrong??
  // { 
  //   std::cout<<"Time out"<<std::endl;
  //   as_.setAborted();
  // } 

  if((time.toSec() >= arm.task_end_time_.toSec()) && (time.toSec() < task_end_time_2_.toSec()))
  { 
    state_ = 1;
    //std::cout<<"RECOVERY state1"<<std::endl;
  }
  else if(time.toSec() < task_end_time_3_.toSec())
  { 
    state_ = 2;
    //std::cout<<"RECOVERY state2"<<std::endl;
  }
  else if(time.toSec() >= task_end_time_3_.toSec())
  {
    //std::cout<<"RECOVERY END"<<std::endl;
    as_.setSucceeded();
  }



  if(state_ == 0)
  {
    f_star = keepCurrentState(origin_, init_rot_, position, rotation, xd, 400, 40).head<3>();
    f_star(assemble_dir_) = target_force_(assemble_dir_); 
    m_star = rotateWithEeAxis(init_rot_, rotation, xd, range_1_, arm.task_start_time_.toSec(), time.toSec(), arm.task_end_time_.toSec(), 1);
    //m_star(1) = 0.0;
  }
  else if(state_ == 1)
  {
    f_star = keepCurrentState(origin_, state_init_rot_1_, position, rotation, xd, 400, 40).head<3>();
    f_star(assemble_dir_) = target_force_(assemble_dir_); 
    m_star = rotateWithEeAxis(state_init_rot_1_, rotation, xd, range_2_, arm.task_end_time_.toSec(), time.toSec(), task_end_time_2_.toSec(), swing_dir_);
    //m_star(1) = 0.0;
  }
  else if(state_ == 2)
  {
    f_star.setZero();
    f_star(assemble_dir_) = target_force_(assemble_dir_); 
    m_star = keepCurrentState(origin_, state_init_rot_2_, position, rotation, xd, 400, 40).tail<3>();
    m_star(1) = 0.0;
  }

  // if(mode_ == 1)
  // {
  //   f_star = oneDofMove(origin_, position, target_(dir_), xd, time.toSec(), arm.task_start_time_.toSec(), duration_, 0.0, dir_);
  //   if(option_ == 0)  m_star = keepOrientationPerpenticular(init_rot_, rotation, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());  
  //   if(option_ == 1)  m_star = keepOrientationPerpenticularOnlyXY(init_rot_, rotation, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());
  //   if(option_ == 2)  m_star = keepCurrentState(origin_, init_rot_, position, rotation, xd, 5000, 100).tail<3>();
    
  // }

  // if(mode_ == 2)
  // {
  //   f_star = twoDofMove(origin_, position, target_, xd, time.toSec(), arm.task_start_time_.toSec(), duration_, 0.0, dir_);
  //   if(option_ == 0)  m_star = keepOrientationPerpenticular(init_rot_, rotation, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());  
  //   if(option_ == 1)  m_star = keepOrientationPerpenticularOnlyXY(init_rot_, rotation, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());
  //   if(option_ == 2)  m_star = keepCurrentState(origin_, init_rot_, position, rotation, xd, 5000, 100).tail<3>();
  // }


  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  return true;
}

double AssembleTripleRecoveryActionServer::getDistance(const Eigen::Vector3d target, const Eigen::Vector3d start, 
  const int mode, const int dir)
{
  double dis;
  Eigen::Vector3d dx;

  dx << target(0) - start(0), target(1) - start(1), target(2) - start(2); 
  
  if(mode == 1)
  {
    dis = sqrt(pow(dx(dir),2));
  }

  if(mode == 2)
  {
    dx(dir) = 0;
    dis = sqrt(pow(dx(0),2) + pow(dx(1),2) + pow(dx(2),2));
  }

  return dis;  
}