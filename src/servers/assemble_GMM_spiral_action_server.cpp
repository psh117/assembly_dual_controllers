#include <assembly_dual_controllers/servers/assemble_GMM_spiral_action_server.h>

AssembleGMMSpiralActionServer::AssembleGMMSpiralActionServer(std::string name, ros::NodeHandle &nh,
                                                             std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleGMMSpiralActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleGMMSpiralActionServer::preemptCallback, this));
  as_.start();
}

void AssembleGMMSpiralActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if (mu_.find(goal_->task_arm) != mu_.end() && mu_.find(goal_->assist_arm) != mu_.end())
  {
    ROS_INFO("AssembleGMMSpiralAction goal has been received.");
  }
  else
  {
    ROS_ERROR("[AssembleGMMSpiralActionServer::goalCallback] the name %s and %s are not in the arm list.", goal_->assist_arm.c_str(), goal_->task_arm.c_str());
    return;
  }

  // Data from the task manager
  task_arm_origin_ = mu_[goal_->task_arm]->transform_;
  assist_arm_origin_ = mu_[goal_->assist_arm]->transform_;
  lin_vel_ = goal_->linear_vel;
  pitch_ = goal_->pitch;    
  friction_ = goal_->friction;
  spiral_duration_ = goal_->spiral_duration;
  pressing_force_ = goal_->pressing_force;

  flange_to_assembly_point_task_(0) = goal_->ee_to_assemble_task.position.x;
  flange_to_assembly_point_task_(1) = goal_->ee_to_assemble_task.position.y;
  flange_to_assembly_point_task_(2) = goal_->ee_to_assemble_task.position.z;
  flange_to_assembly_quat_task_.x() = goal_->ee_to_assemble_task.orientation.x;
  flange_to_assembly_quat_task_.y() = goal_->ee_to_assemble_task.orientation.y;
  flange_to_assembly_quat_task_.z() = goal_->ee_to_assemble_task.orientation.z;
  flange_to_assembly_quat_task_.w() = goal_->ee_to_assemble_task.orientation.w;

  flange_to_assembly_point_assist_(0) = goal_->ee_to_assemble_assist.position.x;
  flange_to_assembly_point_assist_(1) = goal_->ee_to_assemble_assist.position.y;
  flange_to_assembly_point_assist_(2) = goal_->ee_to_assemble_assist.position.z;
  flange_to_assembly_quat_assist_.x() = goal_->ee_to_assemble_assist.orientation.x;
  flange_to_assembly_quat_assist_.y() = goal_->ee_to_assemble_assist.orientation.y;
  flange_to_assembly_quat_assist_.z() = goal_->ee_to_assemble_assist.orientation.z;
  flange_to_assembly_quat_assist_.w() = goal_->ee_to_assemble_assist.orientation.w;

  //-----------------

  count_  = 0;
  is_mode_changed_ = true;
  state_ = READY;
  set_tilt_ = false;

  mu_[goal_->task_arm]->task_start_time_ = ros::Time::now();
  mu_[goal_->assist_arm]->task_start_time_ = ros::Time::now();  

  // Define transformations of EE w.r.t {A}
  T_7A_task_.linear() = flange_to_assembly_quat_task_.toRotationMatrix();
  T_7A_task_.translation() = flange_to_assembly_point_task_;
  T_WA_task_ = task_arm_origin_ * T_7A_task_;

  T_7A_assist_.linear() = flange_to_assembly_quat_assist_.toRotationMatrix();
  T_7A_assist_.translation() = flange_to_assembly_point_assist_;
  T_WA_assist_ = assist_arm_origin_ * T_7A_assist_;

  // Define the dimension of captured data
  torque_captured_.resize(14);
  position_captured_.resize(2);
  torque_captured_.setZero();
  position_captured_.setZero();
  
  torque_init_.resize(14);
  position_init_.resize(2);
  torque_init_.setZero();
  position_init_.setZero();

  joint_list_.resize(torque_data_dimension_);
  joint_list_ << 0, 1, 2, 4, 5, 6, 11, 12, 13; //pre-defined joint number
     

  // INITIALIZE THE GMM MODELS
  // AssembleGMMSpiralActionServer::initializeGMMModels();


  std::cout << "sprial origin: " << task_arm_origin_.translation().transpose() << std::endl;
  std::cout << "pressing_force: " << pressing_force_ << std::endl;
  std::cout << "spiral pitch : " << pitch_ << std::endl;
  std::cout << "linear vel   : " << lin_vel_ << std::endl;  
  std::cout << "T_7A_task : \n"  << T_7A_task_.matrix() << std::endl;

  control_running_ = true;
}

void AssembleGMMSpiralActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleGMMSpiralActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
    return false;

  if (mu_.find(goal_->task_arm) != mu_.end() && mu_.find(goal_->assist_arm) != mu_.end())
  {
    computeTaskArm(time, *mu_[goal_->task_arm]);    
    computeAssistArm(time, *mu_[goal_->assist_arm]);    
    estimateContactState();
    return true;
  }
  else
  {
    ROS_ERROR("[AssembleGMMSpiralActionServer::compute] the name %s and %s are not in the arm list.", goal_->assist_arm.c_str(), goal_->task_arm.c_str());
  }

  return false;
}

bool AssembleGMMSpiralActionServer::computeTaskArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
    return false;

  // auto &mass = arm.mass_matrix_;
  // auto &rotation = arm.rotation_;
  // auto &position = arm.position_;
  auto &jacobian = arm.jacobian_;
  // auto &tau_measured = arm.tau_measured_;
  // auto &gravity = arm.gravity_;
  auto &xd = arm.xd_; //velocity

  Eigen::Vector3d f_star_motion, f_star_active_force;
  Eigen::Vector3d m_star, f_star;
  Eigen::Vector6d f_star_zero, f_d;
  Eigen::Vector6d f_ext;
  double run_time;
  int cnt_max = 150;
  int cnt_start = 50;
  Eigen::Vector6d reaction_force;
  Eigen::Vector3d pos, rot;

  f_ext = arm.f_ext_;
  task_arm_current_ = arm.transform_;

  rot = dyros_math::getPhi(task_arm_current_.linear(), task_arm_origin_.linear());
  rot = task_arm_origin_.linear().inverse()*rot;
  pos = task_arm_current_.translation() - task_arm_origin_.translation();  
  pos = T_WA_task_.linear().inverse()*pos; // w.r.t {A}

  reaction_force.head<3>() = task_arm_current_.linear().inverse() * f_ext.head<3>();
  reaction_force.tail<3>() = task_arm_current_.linear().inverse() * f_ext.tail<3>();

  f_d.setZero();

  if(is_mode_changed_)
  {
    arm.task_start_time_ = time;
    task_arm_origin_ = arm.transform_;
    is_mode_changed_ = false;    
    std::cout<<"GMM SPIRAL STATE IS CHANGED!!"<<std::endl;
  }

  switch (state_)
  {
  case READY:
    if (count_ > cnt_start)
      PegInHole::getCompensationWrench(accumulated_wrench_, f_ext, cnt_start, count_, cnt_max);
    count_++;
    if (count_ >= cnt_max)
    {
      state_ = EXEC;
      is_mode_changed_ = true;
      count_ = cnt_max;
      accumulated_wrench_a_.head<3>() = T_WA_task_.linear().inverse() * accumulated_wrench_.head<3>();
      accumulated_wrench_a_.tail<3>() = T_WA_task_.linear().inverse() * accumulated_wrench_.tail<3>();
      std::cout << "output for compensation: " << accumulated_wrench_.transpose() << std::endl;
      std::cout << "start spiral search" << std::endl;
      time_delay_ = time.toSec() - arm.task_start_time_.toSec();
      std::cout<<"time delay : "<< time_delay_<<std::endl;
    }

    f_star = PegInHole::keepCurrentPose(task_arm_origin_, task_arm_current_, xd, 1000, 10, 3000, 20).head<3>(); //w.r.t {W}
    m_star = PegInHole::rotateWithMat(task_arm_origin_, task_arm_current_, xd, task_arm_origin_.linear(), time.toSec(), arm.task_start_time_.toSec(), cnt_max / 1000, 3500, 20);
    
    spiral_elapsed_time_ = 0.0;
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
      
      if (run_time > 0.5 && detectHole(task_arm_origin_, task_arm_current_, f_ext.head<3>() - accumulated_wrench_.head<3>(), T_WA_task_, friction_*1.85))      
      {
        std::cout << "HOLE IS DETECTED" << std::endl;
        setSucceeded();
        break; 

      }

      Eigen::Vector6d f_spiral_a;
      f_spiral_a = PegInHole::generateSpiralEE2(task_arm_origin_, task_arm_current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_task_,
                                            time.toSec(), arm.task_start_time_.toSec(), arm.task_start_time_.toSec() + spiral_duration_, 5.0, 
                                            set_tilt_, 1000, 10);
      f_star_motion = f_spiral_a.head<3>();
      f_star_motion(2) = 0.0;
      f_star_active_force = f_spiral_a.tail<3>();

      // ============================ P controller w.r.t {A} ============================
      Eigen::Vector3d p_init_a, p_cur_a, v_cur_a, f_feedback;
      Eigen::Vector3d z_axis, z_axis_w;
      double z_target, z_local;
      
      z_target = 1.0;
      
      p_init_a = T_WA_task_.linear().inverse()*task_arm_origin_.translation();
      p_cur_a = T_WA_task_.linear().inverse()*task_arm_current_.translation();
      v_cur_a = T_WA_task_.linear().inverse()*xd.head<3>();

      z_axis = (task_arm_origin_.linear()).block<3,1>(0,2);
      z_axis_w = Eigen::Vector3d::UnitZ();

      
      z_local = p_cur_a(2) - p_init_a(2);
      f_feedback.setZero();
      if(z_axis.transpose()*z_axis_w > 0)
      {
        if(z_local < 0)  f_feedback(2) = 3.0*(z_target-z_local) -2000.0*z_local;
        else             f_feedback(2) = 3.0*(z_target-z_local);
      } 
      else
      {
        if(z_local < 0)  f_feedback(2) = -2000.0*z_local;
      }
      
      // =================================================================================
      f_star_motion = T_WA_task_.linear() * f_star_motion;
      f_star_active_force = T_WA_task_.linear() * (f_star_active_force + f_feedback);

      f_star = f_star_motion;
      f_d.head<3>() = f_star_active_force;
      
      m_star = PegInHole::keepCurrentOrientation(task_arm_origin_, task_arm_current_, xd, 3000, 15);     
      // f_d.setZero();        
      break;
  }

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * (arm.modified_lambda_matrix_ *f_star_zero + f_d);  
  arm.setTorque(desired_torque);

  save_task_arm_force << reaction_force.transpose() << std::endl; // w.r.t {7}
  save_task_arm_pose << pos.transpose()<<" "<< rot.transpose()<<std::endl; // w.r.t {7}
  save_task_arm_torque << arm.tau_ext_filtered_.transpose() << std::endl; 
  save_task_arm_joint << arm.q_.transpose()<<std::endl;  
  save_task_arm_command_global << f_star_motion.transpose()<<" "<<f_star_active_force.transpose()<<std::endl;
  torque_captured_.head<7>() = arm.tau_ext_filtered_;
  position_captured_.head<1>() = pos.tail<1>();


  return true;
}

bool AssembleGMMSpiralActionServer::computeAssistArm(ros::Time time, FrankaModelUpdater &arm)
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
  Eigen::Isometry3d T_7A; // transform of assembly point wrt assist arm
  Eigen::Vector6d reaction_force;
  Eigen::Vector3d pos, rot;
  double reaction_force_sum;

  f_ext = arm.f_ext_;
  assist_arm_current_ = arm.transform_;
  // T_7A = assist_arm_current_.inverse() * T_WA_task_;

  rot = dyros_math::getPhi(assist_arm_current_.linear(), assist_arm_origin_.linear());
  rot = assist_arm_origin_.linear().inverse()*rot;
  pos = assist_arm_current_.translation() - assist_arm_origin_.translation(); // w.r.t {B}
  // pos = assist_arm_origin_.linear().inverse()*pos; // w.r.t {E}
  pos = T_WA_assist_.linear().inverse()*pos;

  f_star = PegInHole::keepCurrentPosition(assist_arm_origin_, assist_arm_current_, xd, 1000, 20);
  m_star = PegInHole::keepCurrentOrientation(assist_arm_origin_, assist_arm_current_, xd, 3500, 15);

  reaction_force.head<3>() = assist_arm_current_.linear().inverse() * f_ext.head<3>();
  reaction_force.tail<3>() = assist_arm_current_.linear().inverse() * f_ext.tail<3>();

  reaction_force_sum = sqrt(pow(reaction_force(0), 2) + pow(reaction_force(1), 2));

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_ * f_star_zero;
  arm.setTorque(desired_torque);

  save_assist_arm_force << reaction_force.transpose() << std::endl; // w.r.t {7}
  save_assist_arm_pose << pos.transpose()<<" "<< rot.transpose()<<std::endl; // w.r.t {7}
  save_assist_arm_torque << arm.tau_ext_filtered_.transpose() << std::endl; 
  save_assist_arm_joint << arm.q_.transpose()<<std::endl;

  torque_captured_.tail<7>() = arm.tau_ext_filtered_;
  position_captured_.tail<1>() = pos.tail<1>();

  return true;
}

// bool AssembleGMMSpiralActionServer::estimateContactState()
// {
//   int n_laps;
//   Estimator::LABEL torque_label, position_label;
//   Estimator::CONTACT_STATE CS;
//   double t_2pi, t_4pi;
//   Eigen::VectorXd torque_input_all, torque_input_selected, position_input;


//   t_2pi = pow((2 * M_PI), 2) * pitch_ / (4 * lin_vel_ * M_PI);
//   t_4pi = pow((4 * M_PI), 2) * pitch_ / (4 * lin_vel_ * M_PI);

//   torque_input_all.resize(14);
//   position_input.resize(position_data_dimension_);
//   torque_input_selected.resize(torque_data_dimension_);
//   torque_input_all.setZero();
//   position_init_.setZero();
//   torque_input_selected.setZero();

//   if (spiral_elapsed_time_ > t_2pi && spiral_elapsed_time_ <= t_4pi)
//   {
//     torque_init_ += torque_captured_;
//     position_init_ += position_captured_;

//     if ( t_4pi - spiral_elapsed_time_ < 0.001)
//     {
//       torque_init_ = torque_init_ / ((t_4pi - t_2pi) * 1000);
//       position_init_ = position_init_ / ((t_4pi - t_2pi) * 1000);
//       std::cout << "=====================" << std::endl;
//       std::cout << "torque init for the task arm   : " << torque_init_.head<7>().transpose() << std::endl;
//       std::cout << "torque init for the assist arm : " << torque_init_.tail<7>().transpose() << std::endl;
//       std::cout << "position init for the both arm : " << position_init_.transpose() << std::endl;
//     }
//   }

//   n_laps = Estimator::countCurrentSpiralLaps(pitch_, lin_vel_, spiral_elapsed_time_);

//   if (n_laps <= 2)
//   {
//     CS = SEARCH;     
//     torque_label = READY_FOR_ESTIMATION;
//     position_label = READY_FOR_ESTIMATION;
//   } 
//   else
//   {
//     for (int i = 0; i < 14; i++)    torque_input_all(i) = abs(torque_captured_(i) - torque_init_(i));    
//     for(int i = 0; i < torque_data_dimension_; i++) torque_input_selected(i) = torque_input_all(joint_list_(i));    
//     position_input = (position_captured_ - position_init_);

//     torque_label = Estimator::torqueEstimator(torque_input_selected, torque_model_small_, torque_model_large_);
//     position_label = Estimator::positionEstimator(position_input, position_model_zero_, position_model_small_, position_model_medium_, position_model_large_);
//     CS = Estimator::contactStateTable(torque_label, position_label);
//   }    

//   save_contact_estimation << (int) CS << std::endl;
//   save_captured_torque << torque_captured_.transpose()<<std::endl;
//   save_captured_position << position_captured_.transpose()<<std::endl;
//   save_torque_label << torque_label<<std::endl;
//   save_position_label << position_label<<std::endl;
//   // std::cout<<" ================================ "<<std::endl;
//   // std::cout<<" torque   : "<< torque_label<<std::endl;
//   // std::cout<<" position : " << position_label<<std::endl;
//   // if (CS == SEARCH)           std::cout << "Current Contact State : "<< CS << std::endl;
//   // else if (CS == CROSSING)      std::cout << "Current Contact State : "<< CS << std::endl;
//   // else if (CS == INSERTION)   std::cout << "Current Contact State : "<< CS << std::endl;
//   // else if (CS == DEVIATION)   std::cout << "Current Contact State : "<< CS << std::endl;
    

//   return true;
// }

// void AssembleGMMSpiralActionServer::initializeGMMModels()
// {
//   torque_model_small_.mu.resize(torque_data_dimension_);
//   torque_model_small_.Sigma.resize(torque_data_dimension_,torque_data_dimension_);   

//   torque_model_large_.mu.resize(torque_data_dimension_);
//   torque_model_large_.Sigma.resize(torque_data_dimension_,torque_data_dimension_);
  
//   position_model_zero_.mu.resize(position_data_dimension_);
//   position_model_zero_.Sigma.resize(position_data_dimension_,position_data_dimension_);
  
//   position_model_small_.mu.resize(position_data_dimension_);
//   position_model_small_.Sigma.resize(position_data_dimension_,position_data_dimension_);
  
//   position_model_medium_.mu.resize(position_data_dimension_);
//   position_model_medium_.Sigma.resize(position_data_dimension_,position_data_dimension_);
  
//   position_model_large_.mu.resize(position_data_dimension_);
//   position_model_large_.Sigma.resize(position_data_dimension_,position_data_dimension_);
  

//   torque_model_small_.mu << 0.7469, 0.6586, 0.7049, 0.2329, 0.2959, 0.3148, 0.2074, 0.2401, 0.1279;
//   torque_model_large_.mu << 3.7412, 3.5535, 2.5854, 1.1819, 1.3607, 0.3651, 0.9599, 1.3707, 0.8865;


//   torque_model_small_.Sigma <<0.4838,    0.0757,    0.3237,    0.0458,    0.0249,    0.0017,    0.0458,    0.0946,    0.0351,
//                               0.0757,    0.5693,    0.1032,    0.0261,    0.0421,   -0.0184,    0.1285,    0.0437,    0.0335,
//                               0.3237,    0.1032,    0.3641,    0.0402,    0.0247,    0.0102,    0.0560,    0.0733,    0.0379,
//                               0.0458,    0.0261,    0.0402,    0.0448,    0.0236,    0.0027,    0.0193,    0.0167,    0.0143,
//                               0.0249,    0.0421,    0.0247,    0.0236,    0.0732,   -0.0006,    0.0137,    0.0141,    0.0222,
//                               0.0017,   -0.0184,    0.0102,    0.0027,   -0.0006,    0.0277,   -0.0009,    0.0039,    0.0031,
//                               0.0458,    0.1285,    0.0560,    0.0193,    0.0137,   -0.0009,    0.0715,    0.0160,    0.0179,
//                               0.0946,    0.0437,    0.0733,    0.0167,    0.0141,    0.0039,    0.0160,    0.0486,    0.0202,
//                               0.0351,    0.0335,    0.0379,    0.0143,    0.0222,    0.0031,    0.0179,    0.0202,    0.0236;

//   torque_model_large_.Sigma <<3.8540,    1.2100,    1.5035,    0.3556,    0.6036,   -0.0359,   -0.2067,    0.9505,    0.4476,
//                               1.2100,    2.6122,    1.0508,    0.0580,    0.4573,   -0.0523,    0.1062,    0.2737,    0.1214,
//                               1.5035,    1.0508,    2.8728,    0.1283,    0.3947,   -0.0202,    0.3997,    0.3967,    0.0474,
//                               0.3556,    0.0580,    0.1283,    0.3421,   -0.1360,    0.0472,    0.0656,   -0.0622,   -0.1021,
//                               0.6036,    0.4573,    0.3947,   -0.1360,    0.8161,   -0.0395,   -0.1359,    0.6037,    0.4991,
//                              -0.0359,   -0.0523,   -0.0202,    0.0472,   -0.0395,    0.0268,   -0.0042,   -0.0193,   -0.0120,
//                              -0.2067,    0.1062,    0.3997,    0.0656,   -0.1359,   -0.0042,    0.5839,   -0.1346,   -0.1823,
//                               0.9505,    0.2737,    0.3967,   -0.0622,    0.6037,   -0.0193,   -0.1346,    0.8136,    0.5459,
//                               0.4476,    0.1214,    0.0474,   -0.1021,    0.4991,   -0.0120,   -0.1823,    0.5459,    0.4627;
  
//   torque_model_small_.pi = 0.5575;
//   torque_model_large_.pi = 0.4425;

//   position_model_zero_.mu<< 0.3965, 0.3424;  
//   position_model_small_.mu <<  1.8437,  0.2280;
//   position_model_medium_.mu << 16.1345,  0.3255;
//   position_model_large_.mu << 21.1101,  0.8137;

//   position_model_zero_.Sigma << 0.0758,    0.0028,
//                                 0.0028,    0.0121;
    
//   position_model_small_.Sigma << 2.1027,    0.0709,
//                                  0.0709,    0.4017;

//   position_model_medium_.Sigma << 15.6573,    0.6824,
//                                   0.6824,    0.8896;

//   position_model_large_.Sigma <<  5.8626,    2.8290,
//                                   2.8290,    1.7458;
  
//   position_model_zero_.pi = 0.436459;
//   position_model_small_.pi = 0.243843;
//   position_model_medium_.pi = 0.233072;
//   position_model_large_.pi = 0.086626;
// }

void AssembleGMMSpiralActionServer::setSucceeded()
{
  //result_.is_completed = true;
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleGMMSpiralActionServer::setAborted()
{
  // result_.is_completed = true;
  as_.setAborted();
  control_running_ = false;
}