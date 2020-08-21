#include <assembly_dual_controllers/servers/assemble_spiral_action_server.h>

AssembleSpiralActionServer::AssembleSpiralActionServer(std::string name, ros::NodeHandle &nh,
                                                       std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleSpiralActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleSpiralActionServer::preemptCallback, this));
  as_.start();

  // spiral_data = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/spiral_data.txt","w");
}

void AssembleSpiralActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if (mu_.find(goal_->arm_name) != mu_.end())
  {
    ROS_INFO("AssembleSpiral goal has been received.");
  }
  else
  {
    ROS_ERROR("[AssembleSpiralActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return;
  }

  lin_vel_ = goal_->linear_vel;
  pitch_ = goal_->pitch;
  mode_ = goal_->mode;
  // assemble_dir_ = goal_->assemble_dir;
  depth_ = goal_->depth;
  friction_ = goal_->friction;
  spiral_duration_ = goal_->spiral_duration;
  pressing_force_ = goal_->pressing_force;
  range_ = goal_->range;
  twist_duration_ = goal_->twist_duration;

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  mu_[goal_->arm_name]->task_end_time_ = ros::Time(mu_[goal_->arm_name]->task_start_time_.toSec() + spiral_duration_);

  f_measured_.setZero();
  
  origin_ = mu_[goal_->arm_name]->transform_;

  ee_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  ee_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  ee_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  ee_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  ee_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  ee_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  ee_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;
  
  force_compensation_(0) = goal_->compensation.force.x; //w.r.t {A}
  force_compensation_(1) = goal_->compensation.force.y;
  force_compensation_(2) = goal_->compensation.force.z;
  moment_compensation_(0) = goal_->compensation.torque.x;
  moment_compensation_(1) = goal_->compensation.torque.y;
  moment_compensation_(2) = goal_->compensation.torque.z;

  std::cout<<"force_compensation_:\n"<<force_compensation_.transpose()<<std::endl;
  std::cout<<"moment_compensation_:\n"<<moment_compensation_.transpose()<<std::endl;

  T_EA_.linear() = ee_to_assembly_quat_.toRotationMatrix(); // it means T_7A_
  T_EA_.translation() = ee_to_assembly_point_;

  T_WA_ = origin_ * T_EA_;
  
  result_.spiral_origin.position.x = T_WA_.translation()(0);
  result_.spiral_origin.position.y = T_WA_.translation()(1);
  result_.spiral_origin.position.z = T_WA_.translation()(2);
  Eigen::Quaterniond temp(T_WA_.linear());
  result_.spiral_origin.orientation.x = temp.x();
  result_.spiral_origin.orientation.y = temp.y();
  result_.spiral_origin.orientation.z = temp.z();
  result_.spiral_origin.orientation.w = temp.w();

  is_first_ = true;

  std::cout << "mode_ : " << mode_ << std::endl;
  
  if (mode_ == 1)
    std::cout << "Single peg in hole" << std::endl;
  if (mode_ == 2)
    std::cout << "dual peg in hole" << std::endl;

  if (spiral_arm_position.is_open())    spiral_arm_position.close();
  if (twist_search.is_open())    twist_search.close();
  if (spiral_search.is_open())    spiral_search.close();
  if (force_spiral_cmd.is_open())  force_spiral_cmd.close();

  spiral_arm_position.open("spiral_arm_position.txt");
  twist_search.open("twist_search.txt");
  spiral_search.open("spiral_search.txt");
  force_spiral_cmd.open("force_spiral_cmd.txt");

  control_running_ = true;
  count_ = 0;

  state_ = READY;
  is_mode_changed_ = true;
  
  std::cout<<"pressing_force: "<<pressing_force_<<std::endl;
  std::cout<<"spiral pitch : "<<pitch_<<std::endl;
  std::cout<<"T_7A: \n"<<T_EA_.matrix()<<std::endl;

  twist_pos_save_.setZero();
}

void AssembleSpiralActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleSpiralActionServer::compute(ros::Time time)
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
    ROS_ERROR("[AssembleSpiralActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}

bool AssembleSpiralActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
    return false;

  auto &mass = arm.mass_matrix_;
  auto &rotation = arm.rotation_;
  auto &position = arm.position_;
  auto &jacobian = arm.jacobian_;
  auto &tau_measured = arm.tau_measured_;
  auto &gravity = arm.gravity_;
  auto &xd = arm.xd_; //velocity

  Eigen::Vector3d f_star, m_star;
  Eigen::Vector6d f_star_zero, f_ext;
  Eigen::Vector3d position_change;

  f_ext = arm.f_ext_;
  current_ = arm.transform_;
  position_change = (T_WA_.inverse()*current_*T_EA_).translation();

  int cnt_max = 150;
  int cnt_start = 50;
  
  if(is_mode_changed_)
  {
    arm.task_start_time_ = time;
    origin_ = arm.transform_;
    is_mode_changed_ = false;
  }

  switch (state_)
  {
  case READY:
    f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 5000, 200, 200, 5).head<3>(); //w.r.t {W}
    m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 5000, 200, 200, 5).tail<3>();
    
    if(count_ > cnt_start) PegInHole::getCompensationWrench(accumulated_wrench_, f_ext, cnt_start, count_, cnt_max);    
    count_++;
    if(count_ >= cnt_max)
    {
      state_ = EXEC;
      is_mode_changed_ = true;
      count_ = cnt_max;      
      accumulated_wrench_a_.head<3>() = T_WA_.linear().inverse() * accumulated_wrench_.head<3>();
      accumulated_wrench_a_.tail<3>() = T_WA_.linear().inverse() * accumulated_wrench_.tail<3>();
      std::cout << "output for compensation: " << accumulated_wrench_.transpose() << std::endl;
      std::cout<<"start spiral search"<<std::endl;
    } 
    break;
  
  case EXEC:
      if (timeOut(time.toSec(), arm.task_start_time_.toSec(), spiral_duration_)) //duration wrong??
      {
        std::cout << "Time out" << std::endl;
        setAborted();
      }

      f_star = PegInHole::generateSpiralEE(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_EA_, time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec());
      // spiral_pos_save_ = PegInHole::generateSpiralEE_datasave(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_EA_, time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec());
      f_star = T_WA_.linear() * (f_star + force_compensation_);

      //signle peg in hole
      if (mode_ == 1)
      {
        m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 350., 5.0);
        m_star = m_star + T_WA_.linear()*moment_compensation_;
      }
      
      //dual peg in hole
      if (mode_ == 2)
      {
        Eigen::Vector6d f_spiral;              
        f_spiral = PegInHole::generateTwistSpiralEE(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_EA_, range_, time.toSec(), arm.task_start_time_.toSec(), spiral_duration_, twist_duration_);    
        
        twist_pos_save_ = PegInHole::generateTwistSpiralEE_savedata(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_EA_, range_, time.toSec(), arm.task_start_time_.toSec(), spiral_duration_, twist_duration_);
               
        f_star = T_WA_.linear()*(f_spiral.head<3>()+force_compensation_);
        m_star = T_WA_.linear() * f_spiral.tail<3>();
      }

      if (detectHole(origin_, current_, f_ext.head<3>() - accumulated_wrench_.head<3>(), T_WA_, friction_))
      // if(detectHole(init_pos_(assemble_dir_),position(assemble_dir_),f,depth_,friction_))
      {
        std::cout << "HOLE IS DETECTED" << std::endl;
        setSucceeded();
      }
      else if(position_change(2) >= depth_)
      {
        std::cout << "HOLE IS DETECTED" << std::endl;
        setSucceeded();
      }
    break;
  }
  
  
  // f_star.setZero();
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  // std::cout<<"pressing_force: "<<pressing_force_<<std::endl;
  // f_star_zero.setZero();

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);
  
  if(state_ == (ASSEMBLY_STATE) EXEC)
  {
    Eigen::Vector6d f_contact;
    f_contact.head<3>() = T_WA_.linear().inverse()*f_ext.head<3>() - accumulated_wrench_a_.head<3>();
    f_contact.tail<3>() = T_WA_.linear().inverse()*f_ext.tail<3>() - accumulated_wrench_a_.tail<3>();
    force_spiral_cmd<< f_contact.transpose()<<std::endl;

    spiral_search << position_change.transpose()<<std::endl;
    if(mode_ == 2) twist_search << twist_pos_save_.transpose() <<std::endl;
    
  }

  Eigen::Vector6d traj;
  traj << arm.position_, (current_*T_EA_).translation();
  spiral_arm_position << traj.transpose()<<std::endl;
  
  return true;
}

// Eigen::Vector3d 
void AssembleSpiralActionServer::setSucceeded()
{
  //result_.is_completed = true;
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleSpiralActionServer::setAborted()
{
  // result_.is_completed = true;
  as_.setAborted(result_);
  control_running_ = false;
}

