#include <assembly_dual_controllers/servers/assemble_approach_bolt_action_server.h>

AssembleApproachBoltActionServer::AssembleApproachBoltActionServer(std::string name, ros::NodeHandle &nh,
                                                                   std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleApproachBoltActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleApproachBoltActionServer::preemptCallback, this));
  as_.start();
}

void AssembleApproachBoltActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if (mu_.find(goal_->arm_name) != mu_.end())
  {
    ROS_INFO("AssembleApproachBolt goal has been received.");
  }
  else
  {
    ROS_ERROR("[AssembleApproachBoltActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();

  // origin_ = mu_[goal_->arm_name]->transform_;
  origin_ = mu_[goal_->arm_name]->initial_transform_;
  contact_force_ = goal_->contact_force;

  descent_speed_ = goal_->descent_speed;
  time_limit_ = goal_->time_limit;
  
  init_yaw_angle_ = mu_[goal_->arm_name]->q_(6,0);  
  revolve_duration_ = abs(init_yaw_angle_)/0.5; //desired_omega = 0.5 rad/s

  state_ = READY;
  
  ee_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  ee_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  ee_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  ee_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  ee_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  ee_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  ee_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;

  //TODO : delete T_WD_, T_AD_
  T_EA_.linear() = ee_to_assembly_quat_.toRotationMatrix();
  T_EA_.translation() = ee_to_assembly_point_;
  T_WA_ = origin_ * T_EA_;

  is_mode_changed_ = false;

  control_running_ = true;

  std::cout << "BOLT APPROACH START" << std::endl;
  // std::cout<<"state_: "<<state_<<std::endl;
  std::cout<<"T_EA_: \n"<< T_EA_.matrix()<<std::endl;
  std::cout << "contact_threshold: " << contact_force_ << std::endl;
  std::cout<<"init yaw angle: "<<init_yaw_angle_*RAD2DEG<<std::endl;
  std::cout<<"revolving duration: "<<revolve_duration_<<std::endl;
  // std::cout<<"descent speed: "<< descent_speed_<<std::endl;
  // std::cout<<goal_->arm_name<<std::endl;
}

void AssembleApproachBoltActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleApproachBoltActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
    return false;

  if (mu_.find(goal_->arm_name) != mu_.end())
  {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  }
  else
  {
    ROS_ERROR("[AssembleApproachBoltActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}

bool AssembleApproachBoltActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
    return false;

  auto &mass = arm.mass_matrix_;
  auto &rotation = arm.rotation_;
  auto &position = arm.position_;
  auto &jacobian = arm.jacobian_;
  auto &tau_measured = arm.tau_measured_;
  auto &gravity = arm.gravity_;
  auto &xd = arm.xd_;

  Eigen::Vector3d f_star, m_star;
  Eigen::Vector6d f_star_zero, f_ext;
  double run_time, revolve_duration;
  Eigen::Vector3d force;

  f_ext = arm.f_ext_;
  current_ = arm.transform_;
  run_time = time.toSec() - arm.task_start_time_.toSec();
  
  if(is_mode_changed_)
  {
    arm.task_start_time_ = time;
    origin_ = arm.transform_;
    is_mode_changed_ = false;
    if(state_ == EXEC) std::cout<<"START APPROACH"<<std::endl;
  }

  switch (state_)
  {
    case READY:
      if (timeOut(time.toSec(), arm.task_start_time_.toSec(), revolve_duration_+0.01))
      {
        state_ = EXEC;
        is_mode_changed_ = true;
        std::cout << "RORATE DONE" << std::endl;
      }
      f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 5000, 200, 200, 5).head<3>(); //w.r.t {W}
      m_star = PegInHole::generateYawingMotionEE(origin_, current_, T_EA_, xd, -init_yaw_angle_, revolve_duration_, time.toSec(), arm.task_start_time_.toSec());
      m_star = T_WA_.linear()*m_star;
      break;

    case EXEC:
      force = f_ext.head<3>();

      if (run_time > 0.05 && Criteria::checkContact(force, T_WA_, contact_force_))
      {
        std::cout << "CHECK CONTATCT!!!!!" << std::endl;
        setSucceeded();
      }

      if (timeOut(time.toSec(), arm.task_start_time_.toSec(), time_limit_)) //duration wrong??
      {
        std::cout << "Time out" << std::endl;
        setAborted();
      }

      f_star = PegInHole::approachComponentEE(origin_, current_, xd, T_EA_, descent_speed_, time.toSec(), arm.task_start_time_.toSec());
      f_star = T_WA_.linear() * f_star;

      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 400, 5);

      break;
  }

  if (state_ == (ASSEMBLY_STATE)EXEC)
  {
    Eigen::Vector6d f_contact;
    f_contact.head<3>() = T_WA_.linear().inverse() * f_ext.head<3>();
    f_contact.tail<3>() = T_WA_.linear().inverse() * f_ext.tail<3>();
    force_moment <<f_contact.transpose() << std::endl;
}

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;

  // f_star_zero.setZero();
  // std::cout<<"f_measured: "<<f_measured_.transpose()<<std::endl;
  // std::cout<<"f_filtered: "<<f_ext.transpose()<<std::endl;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

  return true;  
}

void AssembleApproachBoltActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleApproachBoltActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}