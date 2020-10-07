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
  mu_[goal_->arm_name]->task_end_time_ = ros::Time(mu_[goal_->arm_name]->task_start_time_.toSec() + 2.0);

  f_measured_.setZero();
  desired_xd_.setZero();

  origin_ = mu_[goal_->arm_name]->position_;  
  init_rot_ = mu_[goal_->arm_name]->rotation_;
  
  threshold_ = goal_ ->threshold;
  range_ = goal_ ->range*M_PI/180;
  mode_ = goal_ ->mode;
  swing_dir_ = goal_ ->swing_dir;

  std::cout<<"detection threshold: "<<threshold_<<std::endl;
}

void AssembleVerifyActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool AssembleVerifyActionServer::compute(ros::Time time)
{
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
  
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Matrix<double, 6, 1> f_star_zero;
  Eigen::Matrix<double, 6, 6> lambda;
  Eigen::Matrix<double, 7, 6> J_bar;
  Eigen::Vector3d f_ee;
  Eigen::Vector3d m_ee;
  bool is_done = false;

  lambda = (jacobian*mass.inverse()*jacobian.transpose()).inverse();
  J_bar = mass.inverse()*jacobian.transpose()*lambda;
  f_measured_ = J_bar.transpose()*(tau_measured - gravity);
  
  //base on end-effector's frame
  f_ee = rotation.transpose()*f_measured_.head<3>();
  m_ee = rotation.transpose()*f_measured_.tail<3>();

  if(time.toSec() < arm.task_end_time_.toSec())
  { 
    f_star.setZero();
    f_star(2) = -0.0;
    m_star = rotateWithGlobalAxis(init_rot_, rotation, xd, range_, arm.task_start_time_.toSec(), time.toSec() ,arm.task_end_time_.toSec(), swing_dir_);
    saveMoment(m_ee);
   
  }
  else
  {
    if(mode_ == 1)
    {
      is_done = checkMomentLimit(mx_, my_, mz_, swing_dir_, threshold_);
      if(is_done == true)
      {
        std::cout<<"Single peg in hole is complete"<<std::endl;
        as_.setSucceeded();
      } 
      else
      {
        std::cout<<"Single peg in hole is incomplete"<<std::endl;
        as_.setAborted();
      } 
    }

    if(mode_ == 2)
    {
      is_done = checkMomentLimit(mx_, my_, mz_, swing_dir_, threshold_);
      if(is_done == true)
      {
        std::cout<<"Dual peg in hole is complete"<<std::endl;
        as_.setSucceeded();
      } 
      else
      {
        std::cout<<"Dual peg in hole is incomplete"<<std::endl;
        std::cout<<"threshold: "<<threshold_<<std::endl;
        as_.setAborted();
      } 
    }
    
  }

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  return true;
}

void AssembleVerifyActionServer::saveMoment(const Eigen::Vector3d m)
{
    mx_.push_back(m(0));
    my_.push_back(m(1));
    mz_.push_back(m(2));

    // fprintf(save_data_fm, "%lf\t %lf\t %lf\t\n", mx_.back(), my_.back(), mz_.back() );
}
