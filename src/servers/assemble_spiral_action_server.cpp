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
  minimum_depth_ = goal_->depth;
  friction_ = goal_->friction;
  spiral_duration_ = goal_->spiral_duration;
  pressing_force_ = goal_->pressing_force;
  range_ = goal_->range;
  twist_duration_ = goal_->twist_duration;
  set_tilt_ = goal_->set_tilt;
  partial_search_dir_ = goal_->partial_search_dir;
  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  mu_[goal_->arm_name]->task_end_time_ = ros::Time(mu_[goal_->arm_name]->task_start_time_.toSec() + spiral_duration_);

  f_measured_.setZero();
  
  origin_ = mu_[goal_->arm_name]->transform_;
  return_to_origin_ = origin_.translation();

  flange_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  flange_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  flange_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  flange_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  flange_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  flange_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  flange_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;
  
  // force_compensation_(0) = goal_->compensation.force.x; //w.r.t {A}
  // force_compensation_(1) = goal_->compensation.force.y;
  // force_compensation_(2) = goal_->compensation.force.z;
  // moment_compensation_(0) = goal_->compensation.torque.x;
  // moment_compensation_(1) = goal_->compensation.torque.y;
  // moment_compensation_(2) = goal_->compensation.torque.z;

  T_7A_.linear() = flange_to_assembly_quat_.toRotationMatrix(); // it means T_7A_
  T_7A_.translation() = flange_to_assembly_point_;
  T_WA_ = origin_ * T_7A_;

  flange_to_assembly_point_distance_ = T_7A_.translation().norm();
  flange_to_drill_distance_ = 0.169654;
  std::cout<<"distance : "<< flange_to_assembly_point_distance_<<std::endl;
  is_first_ = true;

  std::cout << "mode_ : " << mode_ << std::endl;  
  if (mode_ == 1)
    std::cout << "Single peg in hole" << std::endl;
  if (mode_ == 2)
    std::cout << "dual peg in hole" << std::endl;

  count_ = 0;

  state_ = READY;
  is_mode_changed_ = true;
  
  std::cout<<"pressing_force: "<<pressing_force_<<std::endl;
  std::cout<<"spiral pitch : "<<pitch_<<std::endl;
  std::cout<<"T_7A: \n"<<T_7A_.matrix()<<std::endl;
  std::cout<<"friction_: "<<friction_<<std::endl;
  std::cout<<"lin_vel :"<<lin_vel_<<std::endl;
  std::cout<<"origin : \n"<<origin_.matrix()<<std::endl;
  if(set_tilt_) std::cout<<" TILTED "<< std::endl;

  twist_pos_save_.setZero();
  control_running_ = true;
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

  // auto &mass = arm.mass_matrix_;
  // auto &rotation = arm.rotation_;
  auto &position = arm.position_;
  auto &jacobian = arm.jacobian_;
  // auto &tau_measured = arm.tau_measured_;
  // auto &gravity = arm.gravity_;
  auto &xd = arm.xd_; //velocity

  Eigen::Vector3d f_star, m_star, m_tilt;
  Eigen::Vector6d f_star_zero, f_ext;
  double run_time;
  int cnt_max = 150;
  int cnt_start = 50;

  f_ext = arm.f_ext_;
  current_ = arm.transform_;
  // position_change = (T_WA_.inverse()*current_*T_7A_).translation();

  Eigen::Vector3d p_init_a, p_cur_a;
  p_init_a = (T_WA_.inverse() * origin_).translation();
  p_cur_a = (T_WA_.inverse() * current_).translation();
  position_change_ = abs(p_cur_a(2) - p_init_a(2));

  if(is_mode_changed_)
  {
    arm.task_start_time_ = time;
    origin_ = arm.transform_;
    is_mode_changed_ = false;
    if(mode_ == 3)  return_to_origin_(2) = origin_.translation()(2);
  }

  switch (state_)
  {
    case READY:
      
      if(count_ > cnt_start) PegInHole::getCompensationWrench(accumulated_wrench_, f_ext, cnt_start, count_, cnt_max);    
      count_++;
      // std::cout<<count_<<std::endl;
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
      f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 900, 40, 2000, 15).head<3>(); //w.r.t {W}
      m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 900, 40, 2000, 15).tail<3>();
      
      break;
  
   case EXEC:
      run_time = time.toSec() - arm.task_start_time_.toSec();
      
      if (timeOut(time.toSec(), arm.task_start_time_.toSec(), spiral_duration_)) //duration wrong??
      {
        std::cout << "Time out" << std::endl;
        setAborted();
        break;
      }
      
      if (run_time > 0.5 && detectHole(origin_, current_, f_ext.head<3>() - accumulated_wrench_.head<3>(), T_WA_, friction_) && position_change_ > minimum_depth_)
      // for partial spiral search, it is required to back to the origin
      {
        if(mode_ == 3 )
        {
          state_ = RETURN;
          is_mode_changed_ = true;
          break;
        }
        else
        {
          std::cout << "HOLE IS DETECTED" << std::endl;
          setSucceeded();
          break; 
        }
      }

      //single peg in hole
      if (mode_ == 1)
      {
        if (set_tilt_)  m_star = PegInHole::generateMoment(origin_, current_, T_7A_, xd, 1.50*M_PI/180, time.toSec(), arm.task_start_time_.toSec(), 1.0, 2000, 30);       
        else      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 2000, 15);     
        
        f_star = PegInHole::generateSpiralEE(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_, time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec(), 5.0, set_tilt_);        
        
        if(flange_to_assembly_point_distance_ == flange_to_drill_distance_)
        { 
          Eigen::Vector3d f_asm, m_a, m_compensation, p_ae;
          f_asm << 0.0, 0.0, f_star(2); // f_star is still represented w.r.t {A}
          p_ae = T_7A_.inverse().translation();
          m_a = -3.0*p_ae.cross(f_asm);
          m_compensation = T_WA_.linear().inverse()*m_a;
          m_star = m_star + m_compensation;
        }

        f_star = T_WA_.linear()*f_star;
        
        spiral_pos_save_ = PegInHole::generateSpiralEE_datasave(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_, time.toSec(), arm.task_start_time_.toSec(), arm.task_end_time_.toSec());
      }      
      //dual peg in hole
      else if (mode_ == 2)
      {
        Eigen::Vector6d f_spiral;              
        f_spiral = PegInHole::generateTwistSpiralEE(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_, range_, time.toSec(), arm.task_start_time_.toSec(), spiral_duration_, twist_duration_);    
        twist_pos_save_ = PegInHole::generateTwistSpiralEE_savedata(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_, range_, time.toSec(), arm.task_start_time_.toSec(), spiral_duration_, twist_duration_);       
        f_star = T_WA_.linear()*(f_spiral.head<3>());
        m_star = T_WA_.linear() * f_spiral.tail<3>();
      }
      // last step for turning a chair
      else if(mode_ == 3)
      {
        f_star = PegInHole::generatePartialSpiralEE(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_, partial_search_dir_, time.toSec(), arm.task_start_time_.toSec(), spiral_duration_, 300, 1.0);
        f_star = T_WA_.linear()*f_star;
        m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 2000, 15);
      }      
      break;

    case RETURN:
      run_time = time.toSec() - arm.task_start_time_.toSec();
      
      if (timeOut(time.toSec(), arm.task_start_time_.toSec(), 2.05)) //duration wrong??
      {
        std::cout << "HOLE IS DETECTED" << std::endl;
        setSucceeded();
        break;
      }
      
      f_star = PegInHole::threeDofMove(origin_, current_, return_to_origin_, xd, time.toSec(), arm.task_start_time_.toSec(), 2.0);
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 2000, 15);
      break;
  }

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  // std::cout<<"force compensation : "<<force_compensation_.transpose()<<std::endl;
  // std::cout<<"accumulated_wrench_ : "<< accumulated_wrench_.head<3>().transpose()<<std::endl;
  // std::cout<<"f_add : "<<f_add.transpose()<<std::endl;
  // std::cout << "det(j)=" << arm.jacobian_.determinant() << std::endl;
  // std::cout<<"================="<<std::endl;
  

  
  // f_star_zero.setZero();

  
  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_ *f_star_zero;
  arm.setTorque(desired_torque);
  
  if(state_ == (ASSEMBLY_STATE) EXEC)
  {
    Eigen::Vector6d f_contact;
    f_contact.head<3>() = T_WA_.linear().inverse()*f_ext.head<3>() - accumulated_wrench_a_.head<3>();
    f_contact.tail<3>() = T_WA_.linear().inverse()*f_ext.tail<3>() - accumulated_wrench_a_.tail<3>();
    force_spiral_cmd<< f_contact.transpose()<<std::endl;

    spiral_search << spiral_pos_save_.transpose()<<std::endl;
    if(mode_ == 2) twist_search << twist_pos_save_.transpose() <<std::endl;
    
  }

  Eigen::Vector6d traj;
  traj << position, (current_*T_7A_).translation();
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

