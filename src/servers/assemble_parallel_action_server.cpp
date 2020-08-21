#include <assembly_dual_controllers/servers/assemble_parallel_action_server.h>

AssembleParallelActionServer::AssembleParallelActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleParallelActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleParallelActionServer::preemptCallback, this));
  as_.start();
}

void AssembleParallelActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleParallel goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleParallelActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  

  f_measured_.setZero();

  origin_ = mu_[goal_->arm_name]->position_;  
  init_rot_ = mu_[goal_->arm_name]->rotation_;
  
  duration_ = goal_ ->duration;
  
  p1_(0) = goal_ ->p1.x;
  p1_(1) = goal_ ->p1.y;
  p1_(2) = goal_ ->p1.z;
  p2_(0) = goal_ ->p2.x;
  p2_(1) = goal_ ->p2.y;
  p2_(2) = goal_ ->p2.z;
  p3_(0) = goal_ ->p3.x;
  p3_(1) = goal_ ->p3.y;
  p3_(2) = goal_ ->p3.z;
    
  
  is_first_ = true;
  
  mu_[goal_->arm_name]->task_end_time_ = ros::Time(mu_[goal_->arm_name]->task_start_time_.toSec() + duration_);

}

void AssembleParallelActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}

bool AssembleParallelActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleParallelActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleParallelActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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
  
  Eigen::Vector3d ee_z_axis;
  Eigen::Vector3d n;
  double d;
  Eigen::Vector3d ex, ey, ez;
  Eigen::Vector3d ex_proj, ey_proj;
  Eigen::Matrix3d o_rot_t, e_rot_t, target_rot;
  Eigen::Vector3d euler;
  double alpha, beta, gamma;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Matrix<double, 6, 1> f_star_zero;

  // if(is_first_ == true)
  // {
  //   origin_ = position;
  //   init_rot_ = rotation;
  //   arm.task_start_time_ = time;

  //   ee_z_axis << init_rot_(0,2), init_rot_(1,2), init_rot_(2,2);    
  //   n = computeNomalVector(p1_, p2_, p3_);

  //   if(ee_z_axis(2)*n(2) < 0) n = -n;

  //   d = -n(0)*origin_(0) - n(1)*origin_(1) - n(2)*origin_(2);
  //   ex = init_rot_.block<3,1>(0,0);
  //   ey = init_rot_.block<3,1>(0,1);
  //   ez = init_rot_.block<3,1>(0,2);
  //   ex_proj << ex(0), ex(1), (-n(0)*ex(0)-n(1)*ex(1)-d)/n(2) - origin_(2);
  //   ey_proj << ey(0), ey(1), (-n(0)*ey(0)-n(1)*ey(1)-d)/n(2) - origin_(2);
  //   ex_proj = ex_proj.normalized();
  //   ey_proj = ey_proj.normalized();
    
  //   o_rot_t.block<3,1>(0,0) = ex_proj;
  //   o_rot_t.block<3,1>(0,1) = ey_proj;
  //   o_rot_t.block<3,1>(0,2) = n;

  //   // o_rot_t << 0.0090053, 0.9996858, 0.0233918, 0.9867422, 0.0050930, 0.1622157, 0.1620456, 0.0245425, -0.9864780; 
  //   e_rot_t = init_rot_.transpose()*o_rot_t;
  
  //   //The desired orientation(quarternion) : 0.702, 0.707, 0.066, -0.049

        
  //   euler = dyros_math::rot2Euler(e_rot_t); //w.r.p end-effector frame
  //   roll_ = euler(0);
  //   pitch_ = euler(1);
  //   yaw_ = euler(2);

  //   is_first_ = false;    
  // }

  // if(time.toSec() - arm.task_start_time_.toSec() > duration_)
  // {
  //   std::cout<<"revolving is done"<<std::endl;
  //   as_.setSucceeded();
  // } 

  // alpha = dyros_math::cubic(time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec(), 0.0, roll_, 0.0, 0.0);
  // beta = dyros_math::cubic(time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec(), 0.0, pitch_, 0.0, 0.0);
  // gamma = dyros_math::cubic(time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec(), 0.0, yaw_, 0.0, 0.0);

  // target_rot = init_rot_*dyros_math::rotateWithZ(gamma) * dyros_math::rotateWithY(beta) * dyros_math::rotateWithX(alpha);

  // delphi_delta = -0.5 * dyros_math::getPhi(rotation, target_rot);

  // f_star = keepCurrentState(origin_,init_rot_, position, rotation, xd, 5000, 100).head<3>();        
  // m_star = (1.0) * 200.0 * delphi_delta + 5.0 * (-xd.tail<3>());  

  // f_star_zero.head<3>() = f_star;
  // f_star_zero.tail<3>() = m_star;
  
  // Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  // arm.setTorque(desired_torque);

  return true;
}
