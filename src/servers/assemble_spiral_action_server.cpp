#include <assembly_dual_controllers/servers/assemble_spiral_action_server.h>

AssembleSpiralActionServer::AssembleSpiralActionServer(std::string name, ros::NodeHandle &nh,
                                                       std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleSpiralActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleSpiralActionServer::preemptCallback, this));
  as_.start();

  for (auto iter = mu_.begin(); iter != mu_.end(); iter++)
  {
    Eigen::VectorXd spiral_gain(4);
    spiral_gain << 1000, 10, 3000, 20;      
    // arm_gain_map_[iter] = std::make_pair(spiral_gain);
    arm_gain_map_[iter->first] = spiral_gain;
  }
  server_  = nh_.advertiseService(name + "_gain_set", &AssembleSpiralActionServer::setTarget, this);  
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
  global_target_ = goal_->global_target;
  use_global_depth_ = goal_->use_global_depth;

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
  force_error_sum_ = 0.0;

  state_ = READY;
  is_mode_changed_ = true;
  
  std::cout<<"pressing_force: "<<pressing_force_<<std::endl;
  std::cout<<"spiral pitch : "<<pitch_<<std::endl;
  std::cout<<"T_7A: \n"<<T_7A_.matrix()<<std::endl;
  std::cout<<"friction_: "<<friction_<<std::endl;
  std::cout<<"lin_vel :"<<lin_vel_<<std::endl;
  std::cout<<"origin : \n"<<origin_.matrix()<<std::endl;
  std::cout<<"global target: "<<global_target_<<std::endl;
  if(set_tilt_) std::cout<<" TILTED "<< std::endl;

  twist_pos_save_.setZero();

  // for GMM estimators
  torque_captured_.resize(torque_data_dimension_);
  pos_vel_captured_.resize(pos_vel_data_dimension_);
  torque_captured_.setZero();
  pos_vel_captured_.setZero();
  
  torque_init_.resize(torque_data_dimension_);
  pos_vel_init_.resize(pos_vel_data_dimension_);
  torque_init_.setZero();
  pos_vel_init_.setZero();

  // initializeGMMModels();
  initializeGMMModelsTriple();
  init_dist_ = computeRelativeLocation();

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
    // saveRobotState();
    // estimateContactState();
    // estimateContactStateTriple();
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
  Eigen::Vector6d f_star_zero, f_d, f_ext;
  Eigen::Vector3d f_star_motion, f_star_active_force;

  double run_time;
  int cnt_max = 150;
  int cnt_start = 50;
  double global_depth_change;

  f_ext = arm.f_ext_;
  current_ = arm.transform_;
  global_depth_change = global_target_ - origin_.translation()(2);

  f_star_motion.setZero();
  f_star_active_force.setZero();
  f_d.setZero();
  // position_change = (T_WA_.inverse()*current_*T_7A_).translation();

  auto & spiral_gain_set = arm_gain_map_[arm.arm_name_];
  double kp = spiral_gain_set(0);
  double kv = spiral_gain_set(1);
  double kp_o = spiral_gain_set(2);
  double kv_o = spiral_gain_set(3);
  // std::cout<<"spiral gain set received : "<< spiral_gain_set.transpose()<<std::endl;

  Eigen::Vector3d p_init_a, p_cur_a;
  p_init_a = (T_WA_.inverse() * origin_).translation();
  p_cur_a = (T_WA_.inverse() * current_).translation();
  position_change_ = abs(p_cur_a(2) - p_init_a(2));

  if(is_mode_changed_)
  {
    if(is_mode_changed_ == true)
      std::cout<<"The end state is "<< state_ << std::endl;
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
      } 
      f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 1000, 10, 2000, 20).head<3>(); //w.r.t {W}
      // m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 900, 40, 2000, 15).tail<3>();
      m_star = PegInHole::rotateWithMat(origin_, current_, xd, origin_.linear(), time.toSec(), arm.task_start_time_.toSec(), cnt_max/1000, 3500, 20);

      spiral_elapsed_time_ = 0.0;
      ready_elapsed_time_ = time.toSec() - arm.task_start_time_.toSec();
      break;
  
   case EXEC:
      run_time = time.toSec() - arm.task_start_time_.toSec();
      spiral_elapsed_time_ = run_time;

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
      if (run_time > 0.5 && detectHole(origin_, current_, f_ext.head<3>() - accumulated_wrench_.head<3>(), T_WA_, friction_*1.85))      
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

      else if(run_time > 0.5 && use_global_depth_ == true && global_depth_change >= 0.030)
      // else if(run_time > 0.5 && detectHole(origin_, current_, f_ext.head<3>() - accumulated_wrench_.head<3>(), T_WA_, friction_*1.5))
      {
        if(mode_ == 3 )
        {
          state_ = RETURN;
          is_mode_changed_ = true;
          break;
        }
        else
        {
          std::cout << "HOLE IS DETECTED WITHOUT SPIRAL MOTION" << std::endl;
          setSucceeded();
          break; 
        }
      }

      //single peg in hole
      if (mode_ == 1)
      {
        if (set_tilt_)  m_star = PegInHole::generateMoment(origin_, current_, T_7A_, xd, 1.50*M_PI/180, time.toSec(), arm.task_start_time_.toSec(), 1.0, 2000, 30);       
        else            m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, kp_o, kv_o);     
        
        // f_star = PegInHole::generateSpiralEE(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_, 
        //                                      time.toSec(), arm.task_start_time_.toSec(), arm.task_start_time_.toSec() + spiral_duration_, 5.0, 
        //                                      set_tilt_, kp, kv);
        // f_star = T_WA_.linear()*f_star;

        Eigen::Vector6d f_spiral_a;
        f_spiral_a = PegInHole::generateSpiralEE2(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_,
                                              time.toSec(), arm.task_start_time_.toSec(), arm.task_start_time_.toSec() + spiral_duration_, 5.0, 
                                              set_tilt_, kp, kv);
        f_star_motion = f_spiral_a.head<3>();
        f_star_motion(2) = 0.0;
        f_star_active_force = f_spiral_a.tail<3>();

        // ============================ P controller w.r.t {A} ============================
        Eigen::Vector3d p_init_a, p_cur_a, v_cur_a, f_feedback;
        Eigen::Vector3d z_axis, z_axis_w;
        double z_target, z_local;
        
        z_target = 1.0;
        
        p_init_a = T_WA_.linear().inverse()*origin_.translation();
        p_cur_a = T_WA_.linear().inverse()*current_.translation();
        v_cur_a = T_WA_.linear().inverse()*xd.head<3>();

        z_axis = (origin_.linear()).block<3,1>(0,2);
        z_axis_w = Eigen::Vector3d::UnitZ();

        
        z_local = p_cur_a(2) - p_init_a(2);
        f_feedback.setZero();
        if(z_axis.transpose()*z_axis_w > 0)
        {
          if(z_local < 0)  f_feedback(2) = 3.0*(z_target-z_local) -1200.0*z_local;
          else             f_feedback(2) = 3.0*(z_target-z_local) - 100*v_cur_a(2);
        } 
        else
        {
          if(z_local < 0)  f_feedback(2) = 2.0*(z_target-z_local) - 500.0*z_local;//-2000.0*z_local;
          else             f_feedback(2) = 3.0*(z_target-z_local) - 25*v_cur_a(2);
        }
        
                
        controller_debug << f_star_motion.transpose() <<" "<< (f_star_active_force + f_feedback).transpose()<<std::endl;
        // spiral_arm_position << z_local<<std::endl;
        // =================================================================================

        f_star_motion = T_WA_.linear() * f_star_motion;
        f_star_active_force = T_WA_.linear() * (f_star_active_force + f_feedback);

        f_star = f_star_motion;
        f_d.head<3>() = f_star_active_force;
        
        // f_d.setZero();
        
        spiral_pos_save_ = PegInHole::generateSpiralEE_datasave(origin_, current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_, time.toSec(), arm.task_start_time_.toSec(), arm.task_start_time_.toSec() + spiral_duration_);
        spiral_arm_position << spiral_pos_save_.transpose()<<std::endl;

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
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, kp_o, kv_o);
      break;
  }

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  // std::cout<<"force compensation : "<<force_compensation_.transpose()<<std::endl;
  // std::cout<<"accumulated_wrench_ : "<< accumulated_wrench_.head<3>().transpose()<<std::endl;
  // std::cout<<"f_add : "<<f_add.transpose()<<std::endl;
  // std::cout << "det(j)=" << arm.jacobian_.determinant() << std::endl;
  // std::cout<<"================="<<std::endl;
  
  // std::cout<<arm.f_ext_env_(2)<<std::endl;
  
  // f_star_zero.setZero();

  save_torque << arm.tau_ext_filtered_.transpose()<<" "<<arm.tau_desired_read_.transpose()<<std::endl;
  force_spiral_cmd << f_star.transpose()<<" "<<f_d.head<3>().transpose()<<" "<<(arm.modified_lambda_matrix_ *f_star_zero + f_d).transpose()<<std::endl;
  contact_force<<(T_WA_.linear().inverse()*arm.f_ext_.head<3>()).transpose()<<" "<<arm.f_ext_.head<3>().transpose()<<std::endl;


  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * (arm.modified_lambda_matrix_ *f_star_zero + f_d);  
  // desired_torque.setZero();
  arm.setTorque(desired_torque);
  
  

  // if(state_ == (ASSEMBLY_STATE) EXEC)
  // {
  //   Eigen::Vector6d f_contact;
  //   f_contact.head<3>() = T_WA_.linear().inverse()*f_ext.head<3>() - accumulated_wrench_a_.head<3>();
  //   f_contact.tail<3>() = T_WA_.linear().inverse()*f_ext.tail<3>() - accumulated_wrench_a_.tail<3>();
  //   // force_spiral_cmd<< f_contact.transpose()<<std::endl;

  //   // spiral_search << spiral_pos_save_.transpose()<<std::endl;
  //   // if(mode_ == 2) twist_search << twist_pos_save_.transpose() <<std::endl;
    
  // }

  // Eigen::Vector6d traj;
  // traj << position, (current_*T_7A_).translation();
  // spiral_arm_position << traj.transpose()<<std::endl;

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

bool AssembleSpiralActionServer::setTarget(assembly_msgs::SetSpiralGain::Request &req,
                                           assembly_msgs::SetSpiralGain::Response &res)
{
  auto it = arm_gain_map_.find(req.arm_name);
  if (it == arm_gain_map_.end())
  {
    ROS_WARN("arm name %s is not in the arm_gain_map_. ignore", req.arm_name.c_str());
    res.is_succeed = false;
    return true;
  }

  if (req.spiral_gain.size() != 4)
  {
    ROS_INFO("req.spiral_gain != 4, resetting the gains");

    Eigen::VectorXd spiral_gain(4);
    spiral_gain << 1000, 10, 3000, 20;      
    arm_gain_map_[req.arm_name] = spiral_gain;
  }
  else
  {
    arm_gain_map_[req.arm_name] = Eigen::VectorXd::Map(req.spiral_gain.data(),4);
  }
  res.is_succeed = true;

  return true;
}

void AssembleSpiralActionServer::saveRobotState()
{
  std::string panda_right, panda_left, panda_top;
  Eigen::Vector7d right_torque, left_torque, top_torque;
  Eigen::Vector3d right_position, left_position, top_position;
  Eigen::Vector6d right_velocity, left_velocity, top_velocity;
  Eigen::Vector6d right_force, left_force, top_force;
  Eigen::Isometry3d T_B_L, T_B_R, T_eer_O, T_eel_P, T_O_P, T_O_H, T_H_P, T_B_H, T_B_P;
  Eigen::Vector3d task_arm_position, task_arm_velocity, task_arm_force;

  panda_right = "panda_right";
  panda_left = "panda_left";
  panda_top = "panda_top";

  T_B_L.setIdentity();
  T_B_R.setIdentity();
  T_B_L(1,3) = 0.3;
  T_B_R(1,3) = -0.3;
  T_eer_O .linear() << -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0;
  T_eer_O.translation() << 0.0, 0.0, 0.0825;
  T_eel_P = T_7A_;
  T_eel_P(2,3) = 0.103 + 0.027;

  //T_O_H.linear() << 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0; // for 0/1 assembly index  
  T_O_H.linear() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0; // for 2/3 assembly index

  // T_O_H.translation() << 0.140, 0.016, 0.0; // for 0 assembly index
  // T_O_H.translation() << 0.140, -0.016, 0.0; // for 1 assembly index
  T_O_H.translation() << -0.130, 0.016, 0.0; // for 2 assembly index
  //T_O_H.translation() << -0.130, -0.016, 0.0; // for 3 assembly index
  
  T_B_H = T_B_R*(mu_[panda_right]->transform_)*T_eer_O*T_O_H;
  T_B_P = T_B_L*(mu_[panda_left]->transform_)*T_eel_P;

  T_H_P = (T_B_H.inverse())*T_B_P;

  // std::cout<<"======="<<std::endl;
  // std::cout<<"H      : "<<T_B_H.translation().transpose()<<std::endl;
  // std::cout<<"P      : "<<T_B_P.translation().transpose()<<std::endl;
  // std::cout<<"H_to_P : "<<T_H_P.translation().transpose()<<std::endl;
  // std::cout<<"eer : "<<(T_B_R*(mu_[panda_right]->transform_)).translation().transpose()<<std::endl;
  // std::cout<<"eel : "<<(T_B_L*(mu_[panda_left]->transform_)).translation().transpose()<<std::endl;
  // std::cout<<"T_7A: \n"<<T_7A_.matrix()<<std::endl;

  
  
  right_torque = mu_[panda_right]->tau_ext_filtered_;
  left_torque = mu_[panda_left]->tau_ext_filtered_;
  top_torque = mu_[panda_top]->tau_ext_filtered_;

  right_position = mu_[panda_right]->position_;
  left_position = mu_[panda_left]->position_;
  top_position = mu_[panda_top]->position_;
  
  right_velocity = mu_[panda_right]->xd_;
  left_velocity = mu_[panda_left]->xd_;
  top_velocity = mu_[panda_top]->xd_;

  right_force = mu_[panda_right]->f_ext_;
  left_force = mu_[panda_left]->f_ext_;
  top_force = mu_[panda_top]->f_ext_;


  task_arm_position = T_H_P.translation();
  task_arm_velocity = T_WA_.linear().inverse()*left_velocity.head<3>();
  task_arm_force = T_WA_.linear().inverse()*((mu_[goal_->arm_name]->f_ext_).head<3>());

  // Parameters for single peg-in-hole
  // double init_pos_offset, hole_to_peg;
  // init_pos_offset = -0.2675;//-0.2675;                    
  // if(abs(init_dist_ - init_pos_offset) < 0.01)  hole_to_peg = computeRelativeLocation() - init_dist_;
  // else                                          hole_to_peg = computeRelativeLocation() - init_pos_offset;

  Eigen::Vector3d P_WA, P_WA_cur, P_WA_init, P_AA;
  Eigen::Matrix3d R_AW;

  P_WA_init = T_WA_.translation(); //  T_WA_ = origin_ * T_7A_;
  P_WA_cur = (current_*T_7A_).translation();


  P_WA = P_WA_cur - P_WA_init;
  R_AW = T_WA_.rotation().inverse();
  P_AA = R_AW*P_WA;

  // torque_captured_.head<7>() = left_torque.transpose();
  // torque_captured_.tail<7>() = right_torque.transpose();

  torque_captured_.head<7>() = left_torque;
  torque_captured_.block<7,1>(7,0) = right_torque;
  torque_captured_.tail<7>() = top_torque;

  // pos_vel_captured_(0) = hole_to_peg*1000;
  pos_vel_captured_(0) = P_AA(2)*1000;

  // pos_vel_captured_(1) = task_arm_velocity(2)*100;
  pos_vel_captured_(1) = (R_AW*right_velocity.head<3>())(2)*100;

  // std::cout<<"init_dist_ : "<< init_dist_<<std::endl;
  // save_captured_torque<< left_torque.transpose()<<" "<<right_torque.transpose()<<" "<<top_torque.transpose()<<std::endl;
  // save_captured_position << pos_vel_captured_.transpose() << std::endl;
  // simple_test << P_AA.transpose()<<std::endl;
  // save_captured_position<< left_position.transpose()<<" "<<right_position.transpose()<<" "<<top_position.transpose()<<std::endl;
  // save_captured_velocity<< left_velocity.transpose()<<" "<<right_velocity.transpose()<<" "<<top_velocity.transpose()<<std::endl;
  // save_captured_force<<left_force.transpose()<<" "<<right_force.transpose()<<" "<<top_force.transpose()<<std::endl;
  // save_task_pfv<<task_arm_position.transpose()<<" "<<task_arm_velocity.transpose()<<" "<<task_arm_force.transpose()<<std::endl;

}

double AssembleSpiralActionServer::computeRelativeLocation()
{
  std::string panda_right, panda_left, panda_top;
  Eigen::Isometry3d T_B_L, T_B_R, T_B_eel, T_B_eer, T_B_rel;
  Eigen::Isometry3d T_eer_eel, T_eel_eer;

  double dist;

  panda_right = "panda_right";
  panda_left = "panda_left";
  panda_top = "panda_top";

  T_B_L.setIdentity();
  T_B_R.setIdentity();
  T_B_L(1,3) = 0.3;
  T_B_R(1,3) = -0.3;

  T_B_eel = T_B_L*(mu_[panda_left]->transform_); // for the task arm
  T_B_eer = T_B_R*(mu_[panda_right]->transform_); // for the assist arm

  T_eer_eel.linear() = (T_B_eer.linear()).inverse()*(T_B_eel.linear());
  T_eel_eer.linear() = (T_B_eel.linear()).inverse()*(T_B_eer.linear());

  T_B_rel.setIdentity();
  T_B_rel.translation()= T_B_eel.translation() - T_B_eer.translation(); // w.r.t. {B}
  
  T_eer_eel.translation() = (T_B_eer.linear()).inverse()*T_B_rel.translation();
  dist = T_eer_eel.translation()(0);  // x component == assembly direction

  // P_B_eel = T_B_eel.translation();
  // P_B_eer = T_B_eer.translation();

  // P_eer_eel = P_B_eel - P_B_eer;
  

  // std::cout<<"MEAUSER TRUE DISTANCE FOR PEG IN HOLE: "<< dist<<std::endl;
  // std::cout<<"MEASUER THE RELATIVE POSITION FROM PANDA_RIGHT TO PANDA_LEFT : "<<P_eer_eel.transpose()<<std::endl;
  // std::cout<<"panda_right_ee : "<<T_B_eer.translation().transpose()<<std::endl;
  // std::cout<<"panda_left_ee : "<<T_B_eel.translation().transpose()<<std::endl;
  // std::cout<<"R_r_l : \n "<<T_eer_eel.linear()<<std::endl;
  // std::cout<<"R_l_r : \n "<<T_eel_eer.linear()<<std::endl;
  // std::cout<<"P_B_rel : \t "<<T_B_rel.translation().transpose()<<std::endl;
  // std::cout<<"p       : \t "<<T_eer_eel.translation().transpose()<<std::endl;
  // std::cout<<"dist    : \t "<<dist<<std::endl;

  return dist;
}

void AssembleSpiralActionServer::initializeGMMModels()
{
  torque_model_small_.mu.resize(torque_data_dimension_);
  torque_model_small_.Sigma.resize(torque_data_dimension_,torque_data_dimension_);   

  torque_model_large_.mu.resize(torque_data_dimension_);
  torque_model_large_.Sigma.resize(torque_data_dimension_,torque_data_dimension_);
  
  pos_vel_model_surface_.mu.resize(pos_vel_data_dimension_);
  pos_vel_model_surface_.Sigma.resize(pos_vel_data_dimension_,pos_vel_data_dimension_);
  
  pos_vel_model_shallow_.mu.resize(pos_vel_data_dimension_);
  pos_vel_model_shallow_.Sigma.resize(pos_vel_data_dimension_,pos_vel_data_dimension_);
  
  pos_vel_model_floating_.mu.resize(pos_vel_data_dimension_);
  pos_vel_model_floating_.Sigma.resize(pos_vel_data_dimension_,pos_vel_data_dimension_);
  
  pos_vel_model_deep_.mu.resize(pos_vel_data_dimension_);
  pos_vel_model_deep_.Sigma.resize(pos_vel_data_dimension_,pos_vel_data_dimension_);
  
  torque_model_small_.mu << 0.826178031301924, 	0.951652316388172, 	0.722971818877865, 	0.681282521725961, 	0.174115016721744, 	0.286046290422887, 	0.233550986924840, 
                          	0.654484795509362, 	0.632709485008311, 	0.712985824663597, 	0.691143408055256, 	0.233099580477550, 	0.283191962387374, 	0.0826557769239171;
  
  torque_model_large_.mu << 3.11944614816480,	2.49981054441851,	2.53424587545022,	1.74253236018272,	1.03652916273793,	1.04901389649385,	0.301127349974888,
                          	2.54658903663213,	2.57942176442797,	2.00078305757714,	2.20769596560338,	0.745200200317623,	1.07304517073657,	0.594198801430098;

  torque_model_small_.Sigma << 0.5911, -0.0534,  0.3601, 0.1839,  0.0013,  0.0430,  0.0090, 0.1415, 0.0958, 0.1069, 0.2339,  0.0108,  0.1332, 0.0137,
                              -0.0534,  0.5171, -0.0895, 0.1210,  0.0159,  0.0352,  0.0071, 0.1573, 0.2325, 0.2503, 0.0622,  0.0855,  0.0033, 0.0102,
                               0.3601, -0.0895,  0.3262, 0.0891,  0.0019,  0.0192,  0.0092, 0.0611, 0.0242, 0.0621, 0.1158,  0.0025,  0.0783, 0.0120,
                               0.1839,  0.1210,  0.0891, 0.2294,  0.0091,  0.0159,  0.0082, 0.0694, 0.1076, 0.0898, 0.1213,  0.0135,  0.0667, 0.0058,
                               0.0013,  0.0159,  0.0019, 0.0091,  0.0162, -0.0017,  0.0043, 0.0073, 0.0061, 0.0089, 0.0093,  0.0051,  0.0019, 0.0014,
                               0.0430,  0.0352,  0.0192, 0.0159, -0.0017,  0.0574, -0.0005, 0.0283, 0.0510, 0.0310, 0.0361,  0.0047,  0.0031, 0.0032,
                               0.0090,  0.0071,  0.0092, 0.0082,  0.0043, -0.0005,  0.0203, 0.0009, 0.0053, 0.0055, 0.0071,  0.0020,  0.0061, 0.0024,
                               0.1415,  0.1573,  0.0611, 0.0694,  0.0073,  0.0283,  0.0009, 0.1898, 0.0910, 0.1842, 0.0630,  0.0520,  0.0226, 0.0069,
                               0.0958,  0.2325,  0.0242, 0.1076,  0.0061,  0.0510,  0.0053, 0.0910, 0.2173, 0.1042, 0.1186,  0.0342,  0.0366, 0.0090,
                               0.1069,  0.2503,  0.0621, 0.0898,  0.0089,  0.0310,  0.0055, 0.1842, 0.1042, 0.2847, 0.0150,  0.0737,  0.0161, 0.0121,
                               0.2339,  0.0622,  0.1158, 0.1213,  0.0093,  0.0361,  0.0071, 0.0630, 0.1186, 0.0150, 0.2016,  0.0072,  0.0691, 0.0056,
                               0.0108,  0.0855,  0.0025, 0.0135,  0.0051,  0.0047,  0.0020, 0.0520, 0.0342, 0.0737, 0.0072,  0.0308, -0.0003, 0.0031,
                               0.1332,  0.0033,  0.0783, 0.0667,  0.0019,  0.0031,  0.0061, 0.0226, 0.0366, 0.0161, 0.0691, -0.0003,  0.0455, 0.0049,
                               0.0137,  0.0102,  0.0120, 0.0058,  0.0014,  0.0032,  0.0024, 0.0069, 0.0090, 0.0121, 0.0056,  0.0031,  0.0049, 000057;


  torque_model_large_.Sigma << 4.8315,  0.9895,  1.8497,  0.8007,  0.5710,  0.0930,  0.0201, -0.0753,  0.7851, -0.1006,  0.4144,  0.0048,  0.8433,  0.2315,
                               0.9895,  3.4208,  0.4128,  0.5743,  0.3551,  0.5267,  0.0250,  0.7185,  2.5637, -0.0024,  1.3623,  0.3427,  0.0238,  0.0260,
                               1.8497,  0.4128,  3.2340, -0.4797,  0.4614,  0.0127,  0.0306,  0.6038,  0.7521, -0.0108,  0.9956,  0.3459,  0.4933,  0.2174,
                               0.8007,  0.5743, -0.4797,  1.7428,  0.4632, -0.0458,  0.0464, -0.1354,  0.7859,  0.2341,  0.3398, -0.0936, -0.0110, -0.0350,
                               0.5710,  0.3551,  0.4614,  0.4632,  0.4752, -0.0685,  0.0366,  0.5287,  0.4852,  0.3412,  0.2990,  0.1382, -0.0594, -0.0086,
                               0.0930,  0.5267,  0.0127, -0.0458, -0.0685,  0.4361, -0.0237,  0.2958,  0.2669,  0.2729,  0.1532,  0.1171,  0.1234,  0.1523,
                               0.0201,  0.0250,  0.0306,  0.0464,  0.0366, -0.0237,  0.0228,  0.0017,  0.0438,  0.0022,  0.0189, -0.0075, -0.0217, -0.0096,
                              -0.0753,  0.7185,  0.6038, -0.1354,  0.5287,  0.2958,  0.0017,  4.1765,  0.4076,  1.8659,  0.7978,  0.9618, -0.2283, -0.0256,
                               0.7851,  2.5637,  0.7521,  0.7859,  0.4852,  0.2669,  0.0438,  0.4076,  4.2290, -0.3468,  1.6651,  0.3460, -0.0700, -0.1343,
                              -0.1006, -0.0024, -0.0108,  0.2341,  0.3412,  0.2729,  0.0022,  1.8659, -0.3468,  1.9287, -0.1409,  0.4633, -0.0645,  0.1189,
                               0.4144,  1.3623,  0.9956,  0.3398,  0.2990,  0.1532,  0.0189,  0.7978,  1.6651, -0.1409,  2.6796,  0.4533,  0.1469,  0.0095,
                               0.0048,  0.3427,  0.3459, -0.0936,  0.1382,  0.1171, -0.0075,  0.9618,  0.3460,  0.4633,  0.4533,  0.3737, -0.0297,  0.0192,
                               0.8433,  0.0238,  0.4933, -0.0110, -0.0594,  0.1234, -0.0217, -0.2283, -0.0700, -0.0645,  0.1469, -0.0297,  0.5502,  0.1984,
                               0.2315,  0.0260,  0.2174, -0.0350, -0.0086,  0.1523, -0.0096, -0.0256, -0.1343,  0.1189,  0.0095,  0.0192,  0.1984,  0.1689;

  torque_model_small_.pi = 0.360584996905935;
  torque_model_large_.pi = 0.639415003094065;

  pos_vel_model_surface_.mu<< 0.150107144763883, 0.0284150123509363;  
  pos_vel_model_shallow_.mu <<  1.37412630832404,  0.382384260758829;
  pos_vel_model_floating_.mu << 12.1519441849704,  1.60060295196681;
  pos_vel_model_deep_.mu << 18.1460249097833,  0.00666067964037633;

  pos_vel_model_surface_.Sigma << 0.0396, 0.0168,
                                   0.0168, 0.0897;
    
  pos_vel_model_shallow_.Sigma << 0.7835, 0.2292,
                                  0.2292, 1.0959;

  pos_vel_model_floating_.Sigma << 16.7156, -0.6760,
                                   -0.6760, 4.9490;

  pos_vel_model_deep_.Sigma << 0.1490, 0.0161,
                                0.0161, 0.1626;

  pos_vel_model_surface_.pi = 0.287448353525457;
  pos_vel_model_shallow_.pi = 0.0458649414142552;
  pos_vel_model_floating_.pi = 0.0704021907466943;
  pos_vel_model_deep_.pi = 0.596284514313593;


  ms_.multi_sampling_start_theta_ = 0.0;
}

void AssembleSpiralActionServer::estimateContactState()
{
  int n_laps;
  Estimator::TORQUE_LABEL torque_label;
  Estimator::POS_VEL_LABEL pos_vel_label;
  Estimator::CONTACT_STATE CS;
  double t_2pi, t_4pi;
  Eigen::VectorXd torque_input, pos_vel_input;


  t_2pi = pow((2 * M_PI), 2) * pitch_ / (4 * lin_vel_ * M_PI);
  t_4pi = pow((4 * M_PI), 2) * pitch_ / (4 * lin_vel_ * M_PI);

  torque_input.resize(torque_data_dimension_);
  pos_vel_input.resize(pos_vel_data_dimension_);  
  torque_input.setZero();
  pos_vel_input.setZero();

  if (spiral_elapsed_time_ > t_2pi && spiral_elapsed_time_ <= t_4pi)
  {
    torque_init_ += torque_captured_;
    pos_vel_init_ += pos_vel_captured_;

    if ( t_4pi - spiral_elapsed_time_ < 0.001)
    {
      torque_init_ = torque_init_ / ((t_4pi - t_2pi) * 1000);
      pos_vel_init_ = pos_vel_init_ / ((t_4pi - t_2pi) * 1000);
      std::cout << "=====================" << std::endl;
      std::cout << "torque init for the task arm   : " << torque_init_.head<7>().transpose() << std::endl;
      std::cout << "torque init for the assist arm : " << torque_init_.tail<7>().transpose() << std::endl;
      std::cout << "position init for the both arm : " << pos_vel_init_.transpose() << std::endl;
    }
  }

  n_laps = Estimator::countCurrentSpiralLaps(pitch_, lin_vel_, spiral_elapsed_time_);

  if (n_laps <= 1)
  {
    CS = Estimator::CONTACT_STATE::SEARCH;    
    torque_label = Estimator::TORQUE_LABEL::READY_TORQUE;    
    pos_vel_label = Estimator::POS_VEL_LABEL::READY_POS_VEL;
  } 
  else
  {
    for (int i = 0; i < torque_data_dimension_; i++)  torque_input(i) = abs(torque_captured_(i) - torque_init_(i));    
    pos_vel_input = (pos_vel_captured_);// - pos_vel_init_);

    torque_label = Estimator::torqueEstimator(torque_input, torque_model_small_, torque_model_large_);
    pos_vel_label = Estimator::positionEstimator(pos_vel_input, pos_vel_model_surface_, pos_vel_model_shallow_, pos_vel_model_floating_, pos_vel_model_deep_);
    CS = Estimator::contactStateTable(torque_label, pos_vel_label);

    
    if(ms_.multi_sampling_flag_ == false && (CS == Estimator::CONTACT_STATE::DEVIATION || CS == Estimator::CONTACT_STATE::INSERTION))
    {
      ms_.multi_sampling_flag_ = true;            
      ms_.multi_sampling_start_theta_ = PegInHole::getSpiralTheta(pitch_, lin_vel_, spiral_elapsed_time_);  
      ms_.multi_sampling_start_time_ = spiral_elapsed_time_;
      std::cout<<"SET MULTI-SAMPLING ESTIMATOR"<<std::endl;
    }
  }    
  
  if(ms_.multi_sampling_flag_ == true)
  {
    double current_spiral_theta, del_theta;
    current_spiral_theta = PegInHole::getSpiralTheta(pitch_, lin_vel_, spiral_elapsed_time_);

    del_theta = current_spiral_theta - ms_.multi_sampling_start_theta_;

    if(CS == Estimator::CONTACT_STATE::DEVIATION) ms_.count_deviation_ ++;
    else if(CS == Estimator::CONTACT_STATE::INSERTION) ms_.count_insertion_ ++;

    if(del_theta >= 2*M_PI)
    {
      ms_.deviation_ratio_ = ms_.count_deviation_ /((spiral_elapsed_time_ - ms_.multi_sampling_start_time_)*1000);
      std::cout << ms_.deviation_ratio_<<std::endl;
      if(ms_.deviation_ratio_ > 0.135)
      {        
        std::cout<<"THE TERMINAL CONTACT STATE IS DEVIATION"<<std::endl;
        std::cout<<ms_.deviation_ratio_<<std::endl;
        setAborted();
      } 
      else
      {
        std::cout<<"THE TERMINAL CONTACT STATE IS INSERTION"<<std::endl;
        std::cout<<ms_.deviation_ratio_<<std::endl;
        setSucceeded();
      } 
    }
    std::cout<<"=============="<<std::endl;
    std::cout<<"start theta   : "<< ms_.multi_sampling_start_theta_*180/M_PI<<std::endl;
    std::cout<<"current theta : "<< current_spiral_theta*180/M_PI<<std::endl;
    std::cout<<"delta theta   : "<< del_theta*180/M_PI<<std::endl;
    std::cout<<"count_insertion : "<< ms_.count_insertion_<<std::endl;
    std::cout<<"count_deviation : "<< ms_.count_deviation_<<std::endl;
    std::cout<<"start_time      : "<< ms_.multi_sampling_start_time_<<std::endl;;
  }


  save_contact_estimation << (int) CS << std::endl;
  // save_captured_torque << torque_input.transpose()<<std::endl;
  // save_captured_position << pos_vel_input.transpose()<<std::endl;
  save_captured_torque << torque_captured_.transpose()<<std::endl;
  save_captured_position << pos_vel_captured_.transpose()<<std::endl;
  save_torque_label << torque_label<<std::endl;
  save_pos_vel_label << pos_vel_label<<std::endl;

  // cs_print_count_ ++;

  // if(cs_print_count_ > 150 && cs_print_count_ < 1000 / 0.02)
  // {
  //   std::cout<<"-------------------"<<std::endl;
  //   std::cout<<"torque_label  : "<<torque_label<<std::endl;
  //   std::cout<<"pos_vel_label : "<<pos_vel_label<<std::endl;
  //   std::cout<<"CS            : "<< CS<<std::endl;

  //   cs_print_count_ = 150;
  // }
  // std::cout<<" ================================ "<<std::endl;
  // std::cout<<" torque   : "<< torque_label<<std::endl;
  // std::cout<<" position : " << position_label<<std::endl;
  // if (CS == SEARCH)           std::cout << "Current Contact State : "<< CS << std::endl;
  // else if (CS == CROSSING)      std::cout << "Current Contact State : "<< CS << std::endl;
  // else if (CS == INSERTION)   std::cout << "Current Contact State : "<< CS << std::endl;
  // else if (CS == DEVIATION)   std::cout << "Current Contact State : "<< CS << std::endl;
}

void AssembleSpiralActionServer::initializeGMMModelsTriple()
{
  t_small_.mu.resize(torque_data_dimension_);
  t_small_.Sigma.resize(torque_data_dimension_,torque_data_dimension_);   

  t_medium_.mu.resize(torque_data_dimension_);
  t_medium_.Sigma.resize(torque_data_dimension_,torque_data_dimension_);

  t_large_.mu.resize(torque_data_dimension_);
  t_large_.Sigma.resize(torque_data_dimension_,torque_data_dimension_);
  
  pv_surface_.mu.resize(pos_vel_data_dimension_);
  pv_surface_.Sigma.resize(pos_vel_data_dimension_,pos_vel_data_dimension_);
  
  pv_shallow_.mu.resize(pos_vel_data_dimension_);
  pv_shallow_.Sigma.resize(pos_vel_data_dimension_,pos_vel_data_dimension_);
  
  pv_floating_.mu.resize(pos_vel_data_dimension_);
  pv_floating_.Sigma.resize(pos_vel_data_dimension_,pos_vel_data_dimension_);
  
  pv_moderate_.mu.resize(pos_vel_data_dimension_);
  pv_moderate_.Sigma.resize(pos_vel_data_dimension_,pos_vel_data_dimension_);

  pv_deep_.mu.resize(pos_vel_data_dimension_);
  pv_deep_.Sigma.resize(pos_vel_data_dimension_,pos_vel_data_dimension_);

  t_small_.mu <<  0.6119, 3.7217, 0.6763, 2.4554, 0.7056, 0.7160, 0.1266, // panda right(task arm)
                  0.0283, 0.0349, 0.0305, 0.0257, 0.0015, 0.0086, 0.0013, // panda left(assist arm)
                  0.0489, 0.0191, 0.0472, 0.0671, 0.0030, 0.0186, 0.0058; // panda top(assist arm)
 
  t_medium_.mu <<  3.6249, 4.3171, 3.3946, 2.8497, 0.7457, 1.1872, 0.8509, // panda right(task arm)
                  0.8100, 0.6504, 0.7690, 0.4182, 0.1936, 0.1993, 0.1203, // panda left(assist arm)
                  0.8845, 0.5067, 0.8338, 1.1905, 0.1895, 0.5728, 0.1727; // panda top(assist arm)

  t_large_.mu <<  4.3940, 3.8799, 4.2279, 2.5539, 0.6990, 1.1923, 0.1583, // panda right(task arm)
                  1.0427, 0.2137, 1.0143, 0.2349, 0.1073, 0.0779, 0.0733, // panda left(assist arm)
                  1.0744, 0.3127, 0.9897, 0.5455, 0.0876, 0.1130, 0.0558; // panda top(assist arm)


  t_small_.Sigma << 0.1483,  0.1855,  0.1444, 0.0749, -0.0007, 0.0081,  0.0176, 0.0009, 0.0005, 0.0011, 0.0009, 0.0000, 0.0002, 0.0000, 0.0020,  0.0007, -0.0002, -0.0036, 0.0001,  0.0000, 0.0002,
                    0.1855,  1.4935,  0.2055, 0.4125,  0.1122, 0.2032,  0.0306, 0.0070, 0.0044, 0.0063, 0.0047, 0.0004, 0.0030, 0.0001, 0.0098,  0.0025,  0.0136, -0.0045, 0.0006,  0.0047, 0.0016,
                    0.1444,  0.2055,  0.1941, 0.0700,  0.0005, 0.0050,  0.0196, 0.0007, 0.0001, 0.0010, 0.0010, 0.0000, 0.0001, 0.0000, 0.0022,  0.0005, -0.0003, -0.0033, 0.0001, -0.0001, 0.0001,
                    0.0749,  0.4125,  0.0700, 0.3646,  0.0656, 0.0845,  0.0144, 0.0040, 0.0053, 0.0047, 0.0037, 0.0001, 0.0016, 0.0001, 0.0070,  0.0014,  0.0054,  0.0106, 0.0004,  0.0023, 0.0005,
                   -0.0007,  0.1122,  0.0005, 0.0656,  0.0419, 0.0322,  0.0040, 0.0005, 0.0008, 0.0003, 0.0006, 0.0000, 0.0003, 0.0000, 0.0007,  0.0000,  0.0013,  0.0021, 0.0000,  0.0006, 0.0001,
                    0.0081,  0.2032,  0.0050, 0.0845,  0.0322, 0.0456,  0.0051, 0.0013, 0.0010, 0.0011, 0.0010, 0.0001, 0.0008, 0.0000, 0.0016,  0.0003,  0.0021,  0.0015, 0.0001,  0.0009, 0.0002,
                    0.0176,  0.0306,  0.0196, 0.0144,  0.0040, 0.0051,  0.0059, 0.0002, 0.0000, 0.0002, 0.0001, 0.0000, 0.0001, 0.0000, 0.0004,  0.0001,  0.0000, -0.0004, 0.0000,  0.0000, 0.0000,
                    0.0009,  0.0070,  0.0007, 0.0040,  0.0005, 0.0013,  0.0002, 0.0008, 0.0001, 0.0008, 0.0003, 0.0000, 0.0002, 0.0000, 0.0007,  0.0000,  0.0004,  0.0001, 0.0000,  0.0001, 0.0000,
                    0.0005,  0.0044,  0.0001, 0.0053,  0.0008, 0.0010,  0.0000, 0.0001, 0.0014, 0.0001, 0.0006, 0.0000, 0.0000, 0.0000, 0.0000,  0.0002,  0.0001,  0.0002, 0.0000,  0.0002, 0.0000,
                    0.0011,  0.0063,  0.0010, 0.0047,  0.0003, 0.0011,  0.0002, 0.0008, 0.0001, 0.0008, 0.0003, 0.0000, 0.0002, 0.0000, 0.0008,  0.0000,  0.0004,  0.0001, 0.0000,  0.0001, 0.0000,
                    0.0009,  0.0047,  0.0010, 0.0037,  0.0006, 0.0010,  0.0001, 0.0003, 0.0006, 0.0003, 0.0007, 0.0000, 0.0002, 0.0000, 0.0002,  0.0001,  0.0003,  0.0002, 0.0000,  0.0001, 0.0000,
                    0.0000,  0.0004,  0.0000, 0.0001,  0.0000, 0.0001,  0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  0.0000,  0.0000,  0.0000, 0.0000,  0.0000, 0.0000,
                    0.0002,  0.0030,  0.0001, 0.0016,  0.0003, 0.0008,  0.0001, 0.0002, 0.0000, 0.0002, 0.0002, 0.0000, 0.0002, 0.0000, 0.0002,  0.0000,  0.0002,  0.0001, 0.0000,  0.0000, 0.0000,
                    0.0000,  0.0001,  0.0000, 0.0001,  0.0000, 0.0000,  0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  0.0000,  0.0000,  0.0000, 0.0000,  0.0000, 0.0000,
                    0.0020,  0.0098,  0.0022, 0.0070,  0.0007, 0.0016,  0.0004, 0.0007, 0.0000, 0.0008, 0.0002, 0.0000, 0.0002, 0.0000, 0.0017,  0.0000,  0.0007,  0.0003, 0.0000,  0.0002, 0.0000,
                    0.0007,  0.0025,  0.0005, 0.0014,  0.0000, 0.0003,  0.0001, 0.0000, 0.0002, 0.0000, 0.0001, 0.0000, 0.0000, 0.0000, 0.0000,  0.0003,  0.0001, -0.0001, 0.0000,  0.0000, 0.0000,
                   -0.0002,  0.0136, -0.0003, 0.0054,  0.0013, 0.0021,  0.0000, 0.0004, 0.0001, 0.0004, 0.0003, 0.0000, 0.0002, 0.0000, 0.0007,  0.0001,  0.0013,  0.0002, 0.0000,  0.0001, 0.0000,
                   -0.0036, -0.0045, -0.0033, 0.0106,  0.0021, 0.0015, -0.0004, 0.0001, 0.0002, 0.0001, 0.0002, 0.0000, 0.0001, 0.0000, 0.0003, -0.0001,  0.0002,  0.0025, 0.0000,  0.0004, 0.0000,
                    0.0001,  0.0006,  0.0001, 0.0004,  0.0000, 0.0001,  0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  0.0000,  0.0000,  0.0000, 0.0000,  0.0000, 0.0000,
                    0.0000,  0.0047, -0.0001, 0.0023,  0.0006, 0.0009,  0.0000, 0.0001, 0.0002, 0.0001, 0.0001, 0.0000, 0.0000, 0.0000, 0.0002,  0.0000,  0.0001,  0.0004, 0.0000,  0.0002, 0.0000,
                    0.0002,  0.0016,  0.0001, 0.0005,  0.0001, 0.0002,  0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,  0.0000,  0.0000,  0.0000, 0.0000,  0.0000, 0.0000;

  t_medium_.Sigma <<   8.7331,  1.6959,  8.1685,  0.2253, -0.4757,  0.4517,  0.2372,  1.1924,  0.0062,  1.2055,  0.7432,  0.1764,  0.2092,  0.0372,  0.9706, -0.4258,  0.4001,  0.8768,  0.1354, 0.2706, 0.0920,
                       1.6959,  6.2508,  1.7606, -3.1788, -0.8130,  0.6010, -0.4976, -0.3290,  0.0626, -0.2688,  0.0564,  0.2051,  0.0067,  0.0582, -0.3568, -0.2645, -0.3246,  0.9703,  0.1043, 0.0347, 0.0271,
                       8.1685,  1.7606,  7.6823,  0.1464, -0.4750,  0.3648,  0.1889,  1.1006, -0.0005,  1.1118,  0.7034,  0.1709,  0.1991,  0.0356,  0.8684, -0.3971,  0.3536,  0.8402,  0.1279, 0.2363, 0.0819,
                       0.2253, -3.1788,  0.1464,  2.6571,  0.4840, -0.3382,  0.5827,  0.5248,  0.0474,  0.4675,  0.1104, -0.0766,  0.0382, -0.0127,  0.4332,  0.1460,  0.1964, -0.2639, -0.0240, 0.1391, 0.0365,
                      -0.4757, -0.8130, -0.4750,  0.4840,  0.2044, -0.0590,  0.1479,  0.0002,  0.0579, -0.0044, -0.0354, -0.0190,  0.0009,  0.0033,  0.0229,  0.0837,  0.0731, -0.0193, -0.0045, 0.0682, 0.0185,
                       0.4517,  0.6010,  0.3648, -0.3382, -0.0590,  0.7389,  0.2339,  0.1921,  0.1059,  0.1829,  0.0214,  0.0071, -0.0123, -0.0016,  0.1558, -0.0347, -0.0610,  0.0971, -0.0098, 0.1673, 0.0508,
                       0.2372, -0.4976,  0.1889,  0.5827,  0.1479,  0.2339,  0.5515,  0.1940,  0.2397,  0.1757,  0.0313,  0.0294,  0.0242,  0.0256,  0.1458,  0.0877,  0.0165,  0.4088,  0.0209, 0.3293, 0.1006,
                       1.1924, -0.3290,  1.1006,  0.5248,  0.0002,  0.1921,  0.1940,  0.4931, -0.0072,  0.4604,  0.1523, -0.0064,  0.0244, -0.0096,  0.3663, -0.0114,  0.0329, -0.0562, -0.0077, 0.0595, 0.0168,
                       0.0062,  0.0626, -0.0005,  0.0474,  0.0579,  0.1059,  0.2397, -0.0072,  0.2288, -0.0023, -0.0128,  0.0462,  0.0218,  0.0335,  0.0168,  0.0685,  0.0596,  0.4012,  0.0377, 0.2255, 0.0665,
                       1.2055, -0.2688,  1.1118,  0.4675, -0.0044,  0.1829,  0.1757,  0.4604, -0.0023,  0.4352,  0.1483, -0.0007,  0.0255, -0.0063,  0.3524, -0.0132,  0.0449, -0.0246, -0.0020, 0.0628, 0.0177,
                       0.7432,  0.0564,  0.7034,  0.1104, -0.0354,  0.0214,  0.0313,  0.1523, -0.0128,  0.1483,  0.1020,  0.0148,  0.0241,  0.0018,  0.1191, -0.0189,  0.0407,  0.0390,  0.0118, 0.0129, 0.0065,
                       0.1764,  0.2051,  0.1709, -0.0766, -0.0190,  0.0071,  0.0294, -0.0064,  0.0462, -0.0007,  0.0148,  0.0295,  0.0123,  0.0151,  0.0131,  0.0079,  0.0294,  0.1613,  0.0224, 0.0555, 0.0179,
                       0.2092,  0.0067,  0.1991,  0.0382,  0.0009, -0.0123,  0.0242,  0.0244,  0.0218,  0.0255,  0.0241,  0.0123,  0.0142,  0.0063,  0.0278,  0.0029,  0.0384,  0.0731,  0.0113, 0.0263, 0.0081,
                       0.0372,  0.0582,  0.0356, -0.0127,  0.0033, -0.0016,  0.0256, -0.0096,  0.0335, -0.0063,  0.0018,  0.0151,  0.0063,  0.0101,  0.0062,  0.0123,  0.0242,  0.0942,  0.0131, 0.0392, 0.0119,
                       0.9706, -0.3568,  0.8684,  0.4332,  0.0229,  0.1558,  0.1458,  0.3663,  0.0168,  0.3524,  0.1191,  0.0131,  0.0278,  0.0062,  0.4050,  0.0078,  0.1380,  0.0014,  0.0205, 0.0713, 0.0227,
                      -0.4258, -0.2645, -0.3971,  0.1460,  0.0837, -0.0347,  0.0877, -0.0114,  0.0685, -0.0132, -0.0189,  0.0079,  0.0029,  0.0123,  0.0078,  0.1645,  0.0615,  0.0471, -0.0034, 0.0744, 0.0242,
                       0.4001, -0.3246,  0.3536,  0.1964,  0.0731, -0.0610,  0.0165,  0.0329,  0.0596,  0.0449,  0.0407,  0.0294,  0.0384,  0.0242,  0.1380,  0.0615,  0.2709,  0.1175,  0.0377, 0.0671, 0.0171,
                       0.8768,  0.9703,  0.8402, -0.2639, -0.0193,  0.0971,  0.4088, -0.0562,  0.4012, -0.0246,  0.0390,  0.1613,  0.0731,  0.0942,  0.0014,  0.0471,  0.1175,  1.2510,  0.1300, 0.4816, 0.1446,
                       0.1354,  0.1043,  0.1279, -0.0240, -0.0045, -0.0098,  0.0209, -0.0077,  0.0377, -0.0020,  0.0118,  0.0224,  0.0113,  0.0131,  0.0205, -0.0034,  0.0377,  0.1300,  0.0245, 0.0430, 0.0136,
                       0.2706,  0.0347,  0.2363,  0.1391,  0.0682,  0.1673,  0.3293,  0.0595,  0.2255,  0.0628,  0.0129,  0.0555,  0.0263,  0.0392,  0.0713,  0.0744,  0.0671,  0.4816,  0.0430, 0.2943, 0.0870,
                       0.0920,  0.0271,  0.0819,  0.0365,  0.0185,  0.0508,  0.1006,  0.0168,  0.0665,  0.0177,  0.0065,  0.0179,  0.0081,  0.0119,  0.0227,  0.0242,  0.0171,  0.1446,  0.0136, 0.0870, 0.0279;


  t_large_.Sigma << 16.9885, -0.1295,  16.1472, -0.3013, -0.1860,  2.4490, -0.0669,  2.8263,  0.5221,  2.7556,  0.6966,  0.3002,  0.1543,  0.1468,  2.7890,  0.3019,  2.4575,  1.3858,  0.1435,  0.2472,  0.1288,
                    -0.1295,  3.1731,  -0.0729,  0.1698, -0.4793, -0.8254,  0.0209, -1.0365, -0.2468, -1.0280,  0.0048, -0.1065,  0.0291, -0.0994, -1.0645, -0.2791, -0.9354, -0.4346, -0.0306, -0.0448, -0.0647,
                    16.1472, -0.0729,  15.3868, -0.2853, -0.1809,  2.3047, -0.0639,  2.6670,  0.4916,  2.6007,  0.6601,  0.2844,  0.1468,  0.1385,  2.6324,  0.2858,  2.3211,  1.3241,  0.1350,  0.2384,  0.1236,
                    -0.3013,  0.1698,  -0.2853,  0.2635, -0.0528, -0.1652,  0.0043, -0.2474, -0.0565, -0.2384, -0.0177, -0.0293,  0.0006, -0.0189, -0.2669, -0.0515, -0.2347, -0.0791, -0.0078, -0.0183, -0.0106,
                    -0.1860, -0.4793,  -0.1809, -0.0528,  0.1220,  0.1567, -0.0071,  0.1891,  0.0514,  0.1866, -0.0071,  0.0211, -0.0080,  0.0193,  0.1927,  0.0461,  0.1667,  0.0779,  0.0043,  0.0079,  0.0129,
                     2.4490, -0.8254,   2.3047, -0.1652,  0.1567,  0.7991, -0.0181,  0.8471,  0.1846,  0.8287,  0.1052,  0.0893,  0.0106,  0.0612,  0.8574,  0.1325,  0.7357,  0.3663,  0.0388,  0.0441,  0.0407,
                    -0.0669,  0.0209,  -0.0639,  0.0043, -0.0071, -0.0181,  0.0107, -0.0166, -0.0031, -0.0157, -0.0012, -0.0021, -0.0004, -0.0001, -0.0149, -0.0001, -0.0146, -0.0022,  0.0001, -0.0002, -0.0008,
                     2.8263, -1.0365,   2.6670, -0.2474,  0.1891,  0.8471, -0.0166,  1.2494,  0.2492,  1.2157,  0.1589,  0.1354,  0.0271,  0.0930,  1.2694,  0.2438,  1.1236,  0.5023,  0.0598,  0.0743,  0.0575,
                     0.5221, -0.2468,   0.4916, -0.0565,  0.0514,  0.1846, -0.0031,  0.2492,  0.0809,  0.2422,  0.0390,  0.0275,  0.0046,  0.0217,  0.2565,  0.0339,  0.2030,  0.1493,  0.0170,  0.0105,  0.0143,
                     2.7556, -1.0280,   2.6007, -0.2384,  0.1866,  0.8287, -0.0157,  1.2157,  0.2422,  1.1843,  0.1532,  0.1317,  0.0253,  0.0907,  1.2342,  0.2375,  1.0928,  0.4889,  0.0575,  0.0728,  0.0563,
                     0.6966,  0.0048,   0.6601, -0.0177, -0.0071,  0.1052, -0.0012,  0.1589,  0.0390,  0.1532,  0.0548,  0.0192,  0.0134,  0.0097,  0.1615,  0.0178,  0.1355,  0.1034,  0.0141,  0.0118,  0.0086,
                     0.3002, -0.1065,   0.2844, -0.0293,  0.0211,  0.0893, -0.0021,  0.1354,  0.0275,  0.1317,  0.0192,  0.0167,  0.0038,  0.0098,  0.1365,  0.0233,  0.1207,  0.0607,  0.0073,  0.0082,  0.0068,
                     0.1543,  0.0291,   0.1468,  0.0006, -0.0080,  0.0106, -0.0004,  0.0271,  0.0046,  0.0253,  0.0134,  0.0038,  0.0054,  0.0010,  0.0277,  0.0019,  0.0234,  0.0232,  0.0039,  0.0024,  0.0012,
                     0.1468, -0.0994,   0.1385, -0.0189,  0.0193,  0.0612, -0.0001,  0.0930,  0.0217,  0.0907,  0.0097,  0.0098,  0.0010,  0.0084,  0.0961,  0.0203,  0.0821,  0.0404,  0.0046,  0.0051,  0.0047,
                     2.7890, -1.0645,   2.6324, -0.2669,  0.1927,  0.8574, -0.0149,  1.2694,  0.2565,  1.2342,  0.1615,  0.1365,  0.0277,  0.0961,  1.3098,  0.2564,  1.1513,  0.5228,  0.0632,  0.0750,  0.0590,
                     0.3019, -0.2791,   0.2858, -0.0515,  0.0461,  0.1325, -0.0001,  0.2438,  0.0339,  0.2375,  0.0178,  0.0233,  0.0019,  0.0203,  0.2564,  0.1205,  0.2622,  0.0150,  0.0008,  0.0228,  0.0113,
                     2.4575, -0.9354,   2.3211, -0.2347,  0.1667,  0.7357, -0.0146,  1.1236,  0.2030,  1.0928,  0.1355,  0.1207,  0.0234,  0.0821,  1.1513,  0.2622,  1.0686,  0.3645,  0.0433,  0.0749,  0.0495,
                     1.3858, -0.4346,   1.3241, -0.0791,  0.0779,  0.3663, -0.0022,  0.5023,  0.1493,  0.4889,  0.1034,  0.0607,  0.0232,  0.0404,  0.5228,  0.0150,  0.3645,  0.4883,  0.0559,  0.0234,  0.0327,
                     0.1435, -0.0306,   0.1350, -0.0078,  0.0043,  0.0388,  0.0001,  0.0598,  0.0170,  0.0575,  0.0141,  0.0073,  0.0039,  0.0046,  0.0632,  0.0008,  0.0433,  0.0559,  0.0087,  0.0014,  0.0035,
                     0.2472, -0.0448,   0.2384, -0.0183,  0.0079,  0.0441, -0.0002,  0.0743,  0.0105,  0.0728,  0.0118,  0.0082,  0.0024,  0.0051,  0.0750,  0.0228,  0.0749,  0.0234,  0.0014,  0.0114,  0.0039,
                     0.1288, -0.0647,   0.1236, -0.0106,  0.0129,  0.0407, -0.0008,  0.0575,  0.0143,  0.0563,  0.0086,  0.0068,  0.0012,  0.0047,  0.0590,  0.0113,  0.0495,  0.0327,  0.0035,  0.0039,  0.0043;

  Eigen::MatrixXd I; // prevent too low determinant
  I.resize(torque_data_dimension_,torque_data_dimension_);
  I.setZero();
  I = I.setIdentity()*0.0001;

  t_small_.Sigma += I;
  // t_medium_.Sigma += I;
  // t_large_.Sigma += I;
  
  t_small_.pi = 0.3964;
  t_medium_.pi = 0.1195;
  t_large_.pi = 0.4841;

  pv_surface_.mu << 0.4153, 0.0021;
  pv_shallow_.mu << 1.1474, 0.0072;
  pv_floating_.mu << 3.3792, 1.3518;
  pv_moderate_.mu << 5.3657, -0.0215;
  pv_deep_.mu << 7.1509, -0.0284;

  pv_surface_.Sigma << 0.0219, -0.0021,
                      -0.0021,  0.0366;
  pv_shallow_.Sigma << 0.0227, 0.0007,
                       0.0007, 0.0465;
  pv_floating_.Sigma << 5.7030, 2.1447,
                        2.1447, 11.2875;
  pv_moderate_.Sigma << 0.0798, 0.0131,
                        0.0131, 0.0586;
  pv_deep_.Sigma << 0.1039, 0.0061,
                    0.0061, 0.0418;

  pv_surface_.pi = 0.451369;
  pv_shallow_.pi = 0.042328;
  pv_floating_.pi = 0.019504;
  pv_moderate_.pi = 0.236826;
  pv_deep_.pi = 0.249973;

  ms_.multi_sampling_start_theta_ = 0.0;
}

void AssembleSpiralActionServer::estimateContactStateTriple()
{
  int n_laps;
  Estimator::TORQUE_LABEL_TRIPLE torque_label;
  Estimator::POS_VEL_LABEL_TRIPLE pos_vel_label;
  Estimator::CONTACT_STATE CS;
  double t_hold;
  Eigen::VectorXd torque_input, pos_vel_input;

  t_hold = 0.15;

  torque_input.resize(torque_data_dimension_); // [panda_right, panda_left, panda_top]
  pos_vel_input.resize(pos_vel_data_dimension_);  
  torque_input.setZero();
  pos_vel_input.setZero();

  if (state_ == ASSEMBLY_STATE::READY)
  {
    torque_init_ += torque_captured_;
    pos_vel_init_ += pos_vel_captured_; 
  }

  if(is_mode_changed_ == true && state_ == ASSEMBLY_STATE::EXEC)  
  {
    torque_init_ = torque_init_ / (ready_elapsed_time_ * 1000);
    pos_vel_init_ = pos_vel_init_ / (ready_elapsed_time_ * 1000);
    std::cout << "===========" << ready_elapsed_time_ << "===========" << std::endl;
    std::cout << "torque init for the left arm  : " << torque_init_.head<7>().transpose() << std::endl;
    std::cout << "torque init for the right arm : " << torque_init_.block<7,1>(7,0).transpose() << std::endl;
    std::cout << "torque init for the top arm   : " << torque_init_.tail<7>().transpose() << std::endl;
    std::cout << "position init for the both arm: " << pos_vel_init_.transpose() << std::endl;
  }
  

  // n_laps = Estimator::countCurrentSpiralLaps(pitch_, lin_vel_, spiral_elapsed_time_);

  if (state_ == ASSEMBLY_STATE::READY)
  {
    CS = Estimator::CONTACT_STATE::READY_CS;    
    torque_label = Estimator::TORQUE_LABEL_TRIPLE::READY_TORQUE_T;    
    pos_vel_label = Estimator::POS_VEL_LABEL_TRIPLE::READY_PV_T;
  } 
  else // real execution!!
  {
    Eigen::VectorXd temp_torque;
    temp_torque.resize(torque_data_dimension_);
    temp_torque.setZero();
    for (int i = 0; i < torque_data_dimension_; i++)  temp_torque(i) = abs(torque_captured_(i) - torque_init_(i));    

    torque_input.head<7>() = temp_torque.block<7,1>(7,0); // torques from panda right
    torque_input.block<7,1>(7,0) = temp_torque.head<7>(); // torques from panda left
    torque_input.tail<7>() = temp_torque.tail<7>(); // torques from panda top
    pos_vel_input = (pos_vel_captured_);// - pos_vel_init_);

    // std::cout<<"-------------------------"<<std::endl;
    // std::cout<<"right torque intput : "<< torque_input.head<7>().transpose()<<std::endl;
    // std::cout<<"left toruqe input : " << torque_input.block<7,1>(7,0).transpose()<<std::endl;
    // std::cout<<"top torque input : "<< torque_input.tail<7>().transpose()<<std::endl;

    torque_label = Estimator::torqueEstimatorTriple(torque_input, t_small_, t_medium_, t_large_);
    pos_vel_label = Estimator::positionEstimatorTriple(pos_vel_input, pv_surface_, pv_shallow_, pv_floating_, pv_moderate_, pv_deep_);
    CS = Estimator::contactStateTableTriple(torque_label, pos_vel_label);

    
    if(ms_.multi_sampling_flag_ == false && (CS == Estimator::CONTACT_STATE::DEVIATION || CS == Estimator::CONTACT_STATE::INSERTION))
    {
      ms_.multi_sampling_flag_ = true;            
      ms_.multi_sampling_start_theta_ = PegInHole::getSpiralTheta(pitch_, lin_vel_, spiral_elapsed_time_);  
      ms_.multi_sampling_start_time_ = spiral_elapsed_time_;
      std::cout<<"SET MULTI-SAMPLING ESTIMATOR"<<std::endl;
    }
  }    
  
  if(ms_.multi_sampling_flag_ == true)
  {
    double current_spiral_theta, del_theta;
    current_spiral_theta = PegInHole::getSpiralTheta(pitch_, lin_vel_, spiral_elapsed_time_);

    del_theta = current_spiral_theta - ms_.multi_sampling_start_theta_;

    if(CS == Estimator::CONTACT_STATE::DEVIATION) ms_.count_deviation_ ++;
    else if(CS == Estimator::CONTACT_STATE::INSERTION) ms_.count_insertion_ ++;

    if(del_theta >= 2*M_PI)//2*M_PI)
    {
      ms_.deviation_ratio_ = ms_.count_deviation_ /((spiral_elapsed_time_ - ms_.multi_sampling_start_time_)*1000);
      std::cout << ms_.deviation_ratio_<<std::endl;
      if(ms_.deviation_ratio_ > 0.45)
      {        
        std::cout<<"THE TERMINAL CONTACT STATE IS DEVIATION"<<std::endl;
        std::cout<<ms_.deviation_ratio_<<std::endl;
        setAborted();
      } 
      else
      {
        std::cout<<"THE TERMINAL CONTACT STATE IS INSERTION"<<std::endl;
        std::cout<<ms_.deviation_ratio_<<std::endl;
        setSucceeded();
      } 
    }
  }


  save_contact_estimation << (int) CS << std::endl;
  // save_captured_torque << torque_input.transpose()<<std::endl;
  // save_captured_position << pos_vel_input.transpose()<<std::endl;
  simple_test << torque_input.transpose()<<std::endl;
  save_captured_torque << torque_captured_.transpose()<<std::endl;
  save_captured_position << pos_vel_captured_.transpose()<<std::endl;
  save_torque_label << torque_label<<std::endl;
  save_pos_vel_label << pos_vel_label<<std::endl;
}
