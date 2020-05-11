#include <assembly_dual_controllers/servers/assemble_side_chair_action_server.h>

AssembleSideChairActionServer::AssembleSideChairActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleSideChairActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleSideChairActionServer::preemptCallback, this));
  as_.start();
}

void AssembleSideChairActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->mover) != mu_.end() && mu_.find(goal_->pusher) != mu_.end() ) 
  {
    ROS_INFO("AssembleSideChair goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleSideChairActionServer::goalCallback] the name %s and %s are not in the arm list.", goal_->pusher.c_str(), goal_ ->mover.c_str());
    return ;
  }

  mu_[goal_->mover]->task_start_time_ = ros::Time::now();
  mu_[goal_->pusher]->task_start_time_ = ros::Time::now();
  
  f_measured_.setZero();
  
  origin_pusher_ = mu_[goal_->pusher]->position_;  
  init_rot_pusher_ = mu_[goal_->pusher]->rotation_;

  origin_mover_ = mu_[goal_->mover]->position_;  
  init_rot_mover_ = mu_[goal_->mover]->rotation_;

  linear_vel_ = goal_ ->linear_vel;
  pitch_ = goal_ ->pitch;
  assemble_dir_ = goal_ ->assemble_dir;
  duration_ = goal_ ->duration;

  target_force_(0) = goal_ ->target_force.force.x;
  target_force_(1) = goal_ ->target_force.force.y;
  target_force_(2) = goal_ ->target_force.force.z;

  mu_[goal_->mover]->task_end_time_ = ros::Time(mu_[goal_->mover]->task_start_time_.toSec() + duration_);
  
  
  // save_data_fm_mover_ = fopen("/home/dyros/catkin_ws/src/snu_assembly/save_data/save_data_fm_mover.txt","w");   
  // save_data_pv_mover_ = fopen("/home/dyros/catkin_ws/src/snu_assembly/save_data/save_data_pv_mover.txt","w");
  // save_data_fm_pusher_ = fopen("/home/dyros/catkin_ws/src/snu_assembly/save_data/save_data_fm_pusher.txt","w");   
  // save_data_pv_pusher_ = fopen("/home/dyros/catkin_ws/src/snu_assembly/save_data/save_data_pv_pusher.txt","w");
  
}

void AssembleSideChairActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool AssembleSideChairActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->mover) != mu_.end() && mu_.find(goal_->pusher) != mu_.end() ) {
    computeMover(time, *mu_[goal_->mover]);
    computePusher(time, *mu_[goal_->pusher]);
    return true;
  } else {
    ROS_ERROR("[AssembleSideChairActionServer::compute] the name %s and %s are not in the arm list.", goal_->pusher.c_str(), goal_ ->mover.c_str());
  }
  
  return false;
}


bool AssembleSideChairActionServer::computePusher(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
      return false; 
  
  auto & mass = arm.mass_matrix_;
  auto & rotation = arm.rotation_;
  auto & position = arm.position_;
  auto & jacobian = arm.jacobian_;
  auto & tau_measured = arm.tau_measured_;
  auto & gravity = arm.gravity_;
  auto & xd = arm.xd_;
  
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Matrix<double, 6, 1> f_star_zero;
  Eigen::Matrix<double, 6, 6> lambda;
  Eigen::Matrix<double, 7, 6> J_bar;
  Eigen::Vector3d f_ee;
  Eigen::Vector3d m_ee;

  double reaction_force = 0.0;

  lambda = (jacobian*mass.inverse()*jacobian.transpose()).inverse();
  J_bar = mass.inverse()*jacobian.transpose()*lambda;
  f_measured_ = J_bar.transpose()*(tau_measured - gravity);

  f_ee = rotation.transpose()*f_measured_.head<3>();
  m_ee = rotation.transpose()*f_measured_.tail<3>();
  
  f_star = keepCurrentState(origin_pusher_, init_rot_pusher_, position, rotation, xd, 5000, 100).head<3>();
  f_star(assemble_dir_) = target_force_(assemble_dir_);
  m_star = keepCurrentState(origin_pusher_, init_rot_pusher_, position, rotation, xd, 5000, 100).tail<3>();  
       
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  for(int i = 0; i < 3; i++)
  {
    if( i == assemble_dir_) reaction_force += 0.0;
    else reaction_force += pow(f_ee(i),2);
  }
  reaction_force = sqrt(reaction_force);

  
  if(checkForceLimit(reaction_force, 12.0))
  {
    as_.setSucceeded();
    std::cout<<"reaction_force: "<<reaction_force<<std::endl;
    std::cout<<"f_sensing: "<<f_ee.transpose()<<std::endl;
  
  }
  
  // fprintf(save_data_fm_pusher_, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_ee(0), f_ee(1), f_ee(2), m_ee(0), m_ee(1), m_ee(2));
  // fprintf(save_data_pv_pusher_, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", position(0), position(1), position(2), xd(0), xd(1), xd(2));

  
  return true;
}


bool AssembleSideChairActionServer::computeMover(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
      return false; 
  
  auto & mass = arm.mass_matrix_;
  auto & rotation = arm.rotation_;
  auto & position = arm.position_;
  auto & jacobian = arm.jacobian_;
  auto & tau_measured = arm.tau_measured_;
  auto & gravity = arm.gravity_;
  auto & xd = arm.xd_;
    
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Matrix<double, 6, 1> f_star_zero;
  Eigen::Matrix<double, 6, 6> lambda;
  Eigen::Matrix<double, 7, 6> J_bar;
  Eigen::Vector3d f_ee;
  Eigen::Vector3d m_ee;

  lambda = (jacobian*mass.inverse()*jacobian.transpose()).inverse();
  J_bar = mass.inverse()*jacobian.transpose()*lambda;
  f_measured_ = J_bar.transpose()*(tau_measured - gravity);

  f_ee = rotation.transpose()*f_measured_.head<3>();
  m_ee = rotation.transpose()*f_measured_.tail<3>();

  if(timeOut(time.toSec(), arm.task_start_time_.toSec(), duration_))
  { 
    std::cout<<"Time out"<<std::endl;
    as_.setAborted();
  } 
    
  f_star = generateSpiral(origin_mover_, position, xd, pitch_, linear_vel_, assemble_dir_, time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec());
  f_star.segment<1>(assemble_dir_) = keepCurrentState(origin_mover_, init_rot_mover_, position, rotation, xd, 5000, 100).block<1,1>(assemble_dir_,0);
  
  
  m_star = keepOrientationPerpenticular(init_rot_mover_, rotation, xd, 2.0, time.toSec(), arm.task_start_time_.toSec());
       
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  // fprintf(save_data_fm_mover_, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_ee(0), f_ee(1), f_ee(2), m_ee(0), m_ee(1), m_ee(2));
  // fprintf(save_data_pv_mover_, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", position(0), position(1), position(2), xd(0), xd(1), xd(2));
  
  return true;
}

