#include <assembly_dual_controllers/servers/assemble_spiral_action_server.h>

AssembleSpiralActionServer::AssembleSpiralActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleSpiralActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleSpiralActionServer::preemptCallback, this));
  as_.start();
}

void AssembleSpiralActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleSpiral goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleSpiralActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  mu_[goal_->arm_name]->task_end_time_ = ros::Time(mu_[goal_->arm_name]->task_start_time_.toSec() + 100.0);

  f_measured_.setZero();
  desired_xd_.setZero();

  origin_ = mu_[goal_->arm_name]->position_;  
  init_rot_ = mu_[goal_->arm_name]->rotation_;

  lin_vel_ = goal_->linear_vel;   
  pitch_ = goal_ ->pitch;
  mode_ = goal_ ->mode;
  assemble_dir_ = goal_ ->assemble_dir;

  ori_change_dir_ = 0;
  is_first_ = true;
  ori_duration_ = 1.0;

  std::cout<<"sprial origin: "<<origin_.transpose()<<std::endl;
  if(mode_ == 1)std::cout<<"Single peg in hole"<<std::endl;
  if(mode_ == 2)std::cout<<"dual peg in hole"<<std::endl;
}

void AssembleSpiralActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool AssembleSpiralActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleSpiralActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleSpiralActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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
  double f = 0.0;

  lambda = (jacobian*mass.inverse()*jacobian.transpose()).inverse();
  J_bar = mass.inverse()*jacobian.transpose()*lambda;
  f_measured_ = J_bar.transpose()*(tau_measured - gravity);

  if(timeOut(time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec()))
  {
    std::cout<<"Time out"<<std::endl;
    as_.setAborted();
  } 

  f_star = generateSpiral(origin_, position, xd, pitch_, lin_vel_, assemble_dir_, time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec());
  f_star(assemble_dir_) = -6.0; // put some value!!! //-6 //-12

  // command for m_star
  if(mode_ == 1) //signle peg in hole
  {
    m_star = keepOrientationPerpenticular(init_rot_, rotation, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());
    // motionForSingle(time, arm, m_star, rotation, xd);
  }
  if(mode_ == 2) //dual peg in hole
  {
    motionForDual(time, m_star, rotation, xd.tail<3>());
  }


  f = calculateFriction(assemble_dir_, f_measured_.head<3>(), f);
  
  if(detectHole(origin_(assemble_dir_),position(assemble_dir_),f,0.003,15.0))
  {
    std::cout<<"HOLE IS DETECTED"<<std::endl;
    std::cout<<"dz: "<<origin_(assemble_dir_)-position(assemble_dir_)<<std::endl;
    std::cout<<"f: "<<f<<std::endl;
    as_.setSucceeded();
  }
  
  
  // f_star.setZero();
  // m_star.setZero();
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  
  return true;
}

void AssembleSpiralActionServer::motionForSingle(ros::Time time, FrankaModelUpdater &arm,
  Eigen::Vector3d m_star, Eigen::Matrix3d rot, Eigen::Matrix<double, 6, 1> xd)
{
  m_star = keepOrientationPerpenticular(init_rot_, rot, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());
  // std::cout<<"running time : "<<time.toSec() - arm.task_start_time_.toSec()<<std::endl;
  // std::cout<<"start time : "<<time.toSec()<<std::endl;
  // std::cout<<"current time : "<<arm.task_end_time_.toSec()<<std::endl;
}

void AssembleSpiralActionServer::motionForDual(ros::Time time, Eigen::Vector3d m_star, Eigen::Matrix3d rot, Eigen::Vector3d angular)
{
  if (ori_change_dir_ == 0)
  {
    if (is_first_ == true)
    {
      ori_start_time_ = time.toSec();
      is_first_ = false;
      // ori_init_ = desired_rotation_M;
    }  

    m_star = generateSpiralWithRotation(init_rot_, rot, angular, time.toSec(), ori_start_time_, ori_duration_, ori_change_dir_, 2);

    if (time.toSec() > ori_start_time_ + ori_duration_ / 2)
    {
      ori_change_dir_ = 1;
      is_first_ = true;
    }
  }

  if (ori_change_dir_ == 1)
  {
    if (is_first_ == true)
    {
      ori_start_time_ = time.toSec();
      is_first_ = false;
    }
    
    m_star = generateSpiralWithRotation(init_rot_, rot, angular, time.toSec(), ori_start_time_, ori_duration_, ori_change_dir_, 2);

    if (time.toSec() > ori_start_time_ + ori_duration_)
    {
      ori_change_dir_ = 2;
      is_first_ = true;
    }
  }

  if (ori_change_dir_ == 2)
  {
    if (is_first_ == true)
    {
      ori_start_time_ = time.toSec();
      is_first_ = false;
    }

    m_star = generateSpiralWithRotation(init_rot_, rot, angular, time.toSec(), ori_start_time_, ori_duration_, ori_change_dir_, 2);

    if (time.toSec() > ori_start_time_ + ori_duration_)
    {
      ori_change_dir_ = 1;
      is_first_ = true;
    }
  }
}
