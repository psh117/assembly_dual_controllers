#include <assembly_dual_controllers/servers/joint_limit_avoidance_action_server.h>

JointLimitAvoidanceActionServer::JointLimitAvoidanceActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&JointLimitAvoidanceActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&JointLimitAvoidanceActionServer::preemptCallback, this));
  as_.start();
}

void JointLimitAvoidanceActionServer::goalCallback()
{ 
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("JointLimitAvoidance goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[JointLimitAvoidanceActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  
  q_init_ = mu_[goal_->arm_name]->q_;  
  delta_q_ = goal_->joint_limit_angle_margin;
  delta_q_ = delta_q_/180 * M_PI;
  duration_ = goal_->duration;
  
  q_target_ = JointLimitAvoidanceActionServer::setJointTarget(rbdl_panda_.getJointLimit(), q_init_, delta_q_);

  std::cout<<"======================"<<std::endl;
  std::cout<<"START TO AVOID JOINT LIMITATION!!"<<std::endl;  
  std::cout<<"delta_q_ : "<<delta_q_<<std::endl;
  std::cout<<"duration_ : "<<duration_<<std::endl;
  std::cout<<"q_init_   : "<<q_init_.transpose()<<std::endl;
  std::cout<<"q_target_ : "<<q_target_.transpose()<<std::endl;
  control_running_ = true;
}

void JointLimitAvoidanceActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool JointLimitAvoidanceActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[JointLimitAvoidanceActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}

bool JointLimitAvoidanceActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
      return false; 
  
  Eigen::Matrix<double, 7, 1> q_desired, qd_desired, qdd_desired, tau_cmd;
  Eigen::Vector3d q_desired_set;
  double run_time;
  run_time = time.toSec() - arm.task_start_time_.toSec();

  if(run_time >= duration_ + 0.5)  setSucceeded();
  
  for (int i = 0; i < 7; i++)
  {
    q_desired_set = dyros_math::quinticSpline(run_time, 0.0, duration_, q_init_(i), 0.0, 0.0, q_target_(i), 0.0, 0.0);    
    q_desired(i) = q_desired_set(0);
    qd_desired(i) = q_desired_set(1);
    qdd_desired(i) = q_desired_set(2);
  }

  Eigen::Matrix<double, 7,7> kp, kv;
  Eigen::Matrix<double, 7, 1> kp_d, kd_d;
  kp_d << 800, 800, 800, 800, 500, 400, 300;
  kd_d << 10, 10, 10, 10, 5, 5, 3;
  
  kp = kp_d.asDiagonal();
  kv = kd_d.asDiagonal();
  
  Eigen::Matrix<double,7,1> desired_torque;  
  Eigen::Matrix<double, 7,7> kpp = Eigen::Matrix<double, 7,7>::Identity() * 0.2;
  kpp(6,6) = 0.05;

  desired_torque = kpp * qdd_desired + (kp*(q_desired - arm.q_) + kv*(qd_desired - arm.qd_)) + arm.coriolis_;
  arm.setTorque(desired_torque);

  // std::cout<<"=================================================="<<std::endl;
  // std::cout<<"run time : "<< run_time<<std::endl;
  // std::cout<<"q_init   : "<<(q_init_*180/M_PI).transpose()<<std::endl;
  // std::cout<<"q_desired: "<<(q_desired*180/M_PI).transpose()<<std::endl;
  // std::cout<<"q_target : "<<(q_target_*180/M_PI).transpose()<<std::endl;

  return true;
}

void JointLimitAvoidanceActionServer::setSucceeded()
{
  result_.is_completed = true;
  as_.setSucceeded(result_);
  control_running_ = false;
}
void JointLimitAvoidanceActionServer::setAborted()
{
  result_.is_completed = false;
  as_.setAborted(result_);
  control_running_ = false;
}

Eigen::Vector7d JointLimitAvoidanceActionServer::setJointTarget(const Eigen::Ref<const Eigen::Matrix<double, 7, 2>> &joint_limit_info,
                                                                const Eigen::Ref<const Eigen::Vector7d> &q_init,
                                                                const double joint_limit_margin)
{
    Eigen::Vector7d joint_lb, joint_ub, joint_margin;
    Eigen::Vector7d q_target;

    joint_margin = joint_margin.setOnes() * joint_limit_margin;

    joint_lb = joint_limit_info.col(0) + joint_margin;
    joint_ub = joint_limit_info.col(1) - joint_margin;

    for (int i = 0; i < 7; i++)
    {
        if (q_init(i) >= joint_ub(i))
        {
          q_target(i) = q_init(i) - joint_limit_margin;        
          std::cout<<i<<"th joint is near joint limitation"<<std::endl;
        } 
        else if (q_init(i) <= joint_lb(i))
        {
          q_target(i) = q_init(i) + joint_limit_margin;   
          std::cout<<i<<"th joint is near joint limitation"<<std::endl;
        }   
        else
        {
          q_target(i) = q_init(i);   
        } 
    }

    return q_target;
}
