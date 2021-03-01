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

  task_arm_origin_ = mu_[goal_->task_arm]->transform_;
  assist_arm_origin_ = mu_[goal_->assist_arm]->transform_;

  lin_vel_ = goal_->linear_vel;
  pitch_ = goal_->pitch;
  mode_ = goal_->mode;
  depth_ = goal_->depth;
  friction_ = goal_->friction;
  spiral_duration_ = goal_->spiral_duration;
  pressing_force_ = goal_->pressing_force;
  range_ = goal_->range;
  twist_duration_ = goal_->twist_duration;
  assist_arm_action_ = goal_->assist_arm_action;

  count_  = 0;
  is_mode_changed_ = true;
  state_ = READY;

  mu_[goal_->task_arm]->task_start_time_ = ros::Time::now();
  mu_[goal_->assist_arm]->task_start_time_ = ros::Time::now();
  mu_[goal_->task_arm]->task_end_time_ = ros::Time(mu_[goal_->task_arm]->task_start_time_.toSec() + spiral_duration_);

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

  T_7A_task_.linear() = flange_to_assembly_quat_task_.toRotationMatrix();
  T_7A_task_.translation() = flange_to_assembly_point_task_;
  T_WA_task_ = task_arm_origin_ * T_7A_task_;

  T_7A_assist_.linear() = flange_to_assembly_quat_assist_.toRotationMatrix();
  T_7A_assist_.translation() = flange_to_assembly_point_assist_;
  T_WA_assist_ = assist_arm_origin_ * T_7A_assist_;

  torque_captured_.resize(14);
  position_captured_.resize(2);
  torque_captured_.setZero();
  position_captured_.setZero();
  torque_init_.resize(14);
  position_init_.resize(2);
  torque_init_.setZero();
  position_init_.setZero();

  // INITIALIZE THE GMM MODELS
  torque_model_small_.mu.resize(14);
  torque_model_small_.Sigma.resize(14,14);  
  torque_model_large_.mu.resize(14);
  torque_model_large_.Sigma.resize(14,14);
  position_model_small_.mu.resize(2);
  position_model_small_.Sigma.resize(2,2);
  position_model_medium_.mu.resize(2);
  position_model_medium_.Sigma.resize(2,2);
  position_model_large_.mu.resize(2);
  position_model_large_.Sigma.resize(2,2);

  torque_model_small_.mu << 0.5970, 0.3177, 0.4884, 0.5291, 0.1868, 0.2249, 0.2364,
                            0.4142, 0.2557, 0.4255, 0.3964, 0.1065, 0.1174, 0.0752;
  torque_model_large_.mu << 1.8305, 0.7929, 1.2729, 0.8895, 0.4602, 0.7847, 0.2213,
                            1.8606, 1.7057, 1.7900, 1.3157, 0.2974, 0.7090, 0.3827;
  torque_model_small_.Sigma <<    0.1937,  0.0542,  0.1527,  0.0577,  0.0201,  0.0208, -0.0001,  0.1468,  0.0528,  0.1514,  0.0887,  0.0295,  0.0172, -0.0052,
                                  0.0542,  0.0759,  0.0877,  0.0162,  0.0136,  0.0067, -0.0049,  0.0701,  0.0496,  0.0733,  0.0522,  0.0167,  0.0065, -0.0035,
                                  0.1527,  0.0877,  0.1914,  0.0724,  0.0200,  0.0129,  0.0012,  0.1628,  0.0593,  0.1650,  0.0831,  0.0300,  0.0127, -0.0074,
                                  0.0577,  0.0162,  0.0724,  0.1523,  0.0074, -0.0239,  0.0098,  0.0775, -0.0006,  0.0714,  0.0157,  0.0056,  0.0037,  0.0017,
                                  0.0201,  0.0136,  0.0200,  0.0074,  0.0239,  0.0067,  0.0063,  0.0192,  0.0144,  0.0217,  0.0214,  0.0058,  0.0036,  0.0004,
                                  0.0208,  0.0067,  0.0129,  -0.0239, 0.0067,  0.0312,  0.0054,  0.0103,  0.0094,  0.0150,  0.0170,  0.0048,  0.0028, -0.0020,
                                 -0.0001, -0.0049,  0.0012,  0.0098,  0.0063,  0.0054,  0.0221,  0.0051, -0.0081,  0.0042, -0.0042, -0.0003, -0.0020,  0.0033,
                                  0.1468,  0.0701,  0.1628,  0.0775,  0.0192,  0.0103,  0.0051,  0.1830,  0.0507,  0.1814,  0.0710,  0.0312,  0.0038, -0.0024,
                                  0.0528,  0.0496,  0.0593,  -0.0006, 0.0144,  0.0094, -0.0081,  0.0507,  0.0647,  0.0586,  0.0706,  0.0169,  0.0136, -0.0050,
                                  0.1514,  0.0733,  0.1650,  0.0714,  0.0217,  0.0150,  0.0042,  0.1814,  0.0586,  0.1838,  0.0812,  0.0328,  0.0063, -0.0039,
                                  0.0887,  0.0522,  0.0831,  0.0157,  0.0214,  0.0170, -0.0042,  0.0710,  0.0706,  0.0812,  0.0977,  0.0221,  0.0206, -0.0046,
                                  0.0295,  0.0167,  0.0300,  0.0056,  0.0058,  0.0048, -0.0003,  0.0312,  0.0169,  0.0328,  0.0221,  0.0084,  0.0031, -0.0007,
                                  0.0172,  0.0065,  0.0127,  0.0037,  0.0036,  0.0028, -0.0020,  0.0038,  0.0136,  0.0063,  0.0206,  0.0031,  0.0096, -0.0018,
                                 -0.0052, -0.0035, -0.0074,  0.0017,  0.0004, -0.0020,  0.0033, -0.0024, -0.0050, -0.0039, -0.0046, -0.0007, -0.0018,  0.0063;
  torque_model_large_.Sigma <<    1.3073,  0.1941,  0.2434,  0.0539,  0.0009, -0.0185, -0.0243,  0.3661, -0.4106,  0.3603, -0.0066,  0.0294,  0.0280,  0.0022,
                                  0.1941,  0.8860, -0.0622,  0.1289,  0.0631,  0.1081, -0.0033, -0.3312,  0.2671, -0.2610, -0.0738,  0.0026, -0.0577,  0.0579,
                                  0.2434, -0.0622,  2.0211,  0.1137,  0.3403, -0.1855,  0.0286, -0.1959,  1.9697, -0.1800,  1.4845,  0.0816,  0.2619, -0.1527,
                                  0.0539,  0.1289,  0.1137,  0.4664,  0.0909, -0.1357,  0.0176, -0.2365,  0.2251, -0.1818,  0.1738,  0.0520, -0.0150, -0.0548,
                                  0.0009,  0.0631,  0.3403,  0.0909,  0.1467,  0.0426,  0.0180, -0.0727,  0.4601, -0.0606,  0.3908,  0.0142,  0.1157,  0.0079,
                                 -0.0185,  0.1081, -0.1855, -0.1357,  0.0426,  0.4983, -0.0045,  0.5549,  0.0763,  0.5184,  0.0647, -0.0170,  0.2435,  0.2155,
                                 -0.0243, -0.0033,  0.0286,  0.0176,  0.0180, -0.0045,  0.0174, -0.0373,  0.0377, -0.0393,  0.0320, -0.0024,  0.0042, -0.0013,
                                  0.3661, -0.3312, -0.1959, -0.2365, -0.0727,  0.5549, -0.0373,  2.3802, -0.2228,  2.1311, -0.2975,  0.0391,  0.2131,  0.2735,
                                 -0.4106,  0.2671,  1.9697,  0.2251,  0.4601,  0.0763,  0.0377, -0.2228,  3.1305, -0.1450,  1.8505,  0.1384,  0.3366, -0.0771,
                                  0.3603, -0.2610, -0.1800, -0.1818, -0.0606,  0.5184, -0.0393,  2.1311, -0.1450,  2.0326, -0.1979,  0.0816,  0.2119,  0.2566,
                                 -0.0066, -0.0738,  1.4845,  0.1738,  0.3908,  0.0647,  0.0320, -0.2975,  1.8505, -0.1979,  1.6824,  0.0861,  0.4074, -0.0560,
                                  0.0294,  0.0026,  0.0816,  0.0520,  0.0142, -0.0170, -0.0024,  0.0391,  0.1384,  0.0816,  0.0861,  0.0504, -0.0041, -0.0169,
                                  0.0280, -0.0577,  0.2619, -0.0150,  0.1157,  0.2435,  0.0042,  0.2131,  0.3366,  0.2119,  0.4074, -0.0041,  0.2785,  0.1113,
                                  0.0022,  0.0579, -0.1527, -0.0548,  0.0079,  0.2155, -0.0013,  0.2735, -0.0771,  0.2566, -0.0560, -0.0169,  0.1113,  0.1177;

  position_model_small_.mu <<  0.8396,  0.6066;
  position_model_medium_.mu << 5.7878,  0.8697;
  position_model_large_.mu << 18.1468, -0.2689;
  position_model_small_.Sigma <<  0.0452, -0.0092,
                                 -0.0092,  0.0084;
  position_model_medium_.Sigma << 17.2049,  0.0992,
                                   0.0992,  0.2601;
  position_model_large_.Sigma << 0.0452, -0.0092,
                                -0.0092,  0.0084;



  std::cout << "sprial origin: " << task_arm_origin_.translation().transpose() << std::endl;
  if (mode_ == 1)
    std::cout << "single peg in hole" << std::endl;
  if (mode_ == 2)
    std::cout << "dual peg in hole" << std::endl;

  result_.spiral_origin.position.x = T_WA_task_.translation()(0);
  result_.spiral_origin.position.y = T_WA_task_.translation()(1);
  result_.spiral_origin.position.z = T_WA_task_.translation()(2);
  Eigen::Quaterniond temp(T_WA_task_.linear());
  result_.spiral_origin.orientation.x = temp.x();
  result_.spiral_origin.orientation.y = temp.y();
  result_.spiral_origin.orientation.z = temp.z();
  result_.spiral_origin.orientation.w = temp.w();

  std::cout << "pressing_force: " << pressing_force_ << std::endl;
  std::cout << "spiral pitch : " << pitch_ << std::endl;
  std::cout << "save the spiral origin: \n"
            << result_.spiral_origin << std::endl;
  std::cout << "T_7A_task : \n"
            << T_7A_task_.matrix() << std::endl;

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

  Eigen::Vector6d f_star;
  Eigen::Vector3d f_star_motion, f_star_active_force;
  Eigen::Vector3d m_star;
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
  // pos = task_arm_origin_.linear().inverse()*pos; // w.r.t {E} = {A}
  pos = T_WA_task_.linear().inverse()*pos; // w.r.t {A}

  reaction_force.head<3>() = task_arm_current_.linear().inverse() * f_ext.head<3>();
  reaction_force.tail<3>() = task_arm_current_.linear().inverse() * f_ext.tail<3>();

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

    f_star_motion = PegInHole::keepCurrentPose(task_arm_origin_, task_arm_current_, xd, 1000, 10, 2000, 20).head<3>(); //w.r.t {W}
    f_star_active_force.setZero();

    m_star = PegInHole::rotateWithMat(task_arm_origin_, task_arm_current_, xd, task_arm_origin_.linear(), time.toSec(), arm.task_start_time_.toSec(), cnt_max / 1000, 3000, 20);
    
    spiral_elapsed_time_ = 0.0;
    break;

  case EXEC:
    if (timeOut(time.toSec(), arm.task_start_time_.toSec(), spiral_duration_)) //duration wrong??
    {
      std::cout << "Time out" << std::endl;
      setAborted();
    }

    run_time = time.toSec() - arm.task_start_time_.toSec() - time_delay_;
    spiral_elapsed_time_ = run_time;
    if (run_time > 0.5 && detectHole(task_arm_origin_, task_arm_current_, f_ext.head<3>() - accumulated_wrench_.head<3>(), T_WA_task_, friction_))
    {
      std::cout << "HOLE IS DETECTED" << std::endl;
      setSucceeded();
      break;
    }

    f_star = PegInHole::generateSpiralEE2(task_arm_origin_, task_arm_current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_task_, time.toSec(), arm.task_start_time_.toSec(), arm.task_start_time_.toSec() + spiral_duration_);
    f_star_motion = f_star.head<3>();
    f_star_active_force = f_star.tail<3>();
    save_task_arm_command_local << f_star_motion.transpose()<<" "<<f_star_active_force.transpose()<<std::endl;
    f_star_motion = T_WA_task_.linear() * f_star_motion;
    f_star_active_force = T_WA_task_.linear() * f_star_active_force;
    
    
    //signle peg in hole
    if (mode_ == 1)
      m_star = PegInHole::keepCurrentOrientation(task_arm_origin_, task_arm_current_, xd, 2000., 15.0);

    //dual peg in hole
    // if (mode_ == 2)
    // {
    //   Eigen::Vector6d f_spiral;
    //   f_spiral = PegInHole::generateTwistSpiralEE(task_arm_origin_, task_arm_current_, xd, pitch_, lin_vel_, pressing_force_, T_7A_task_, range_, time.toSec(), arm.task_start_time_.toSec(), spiral_duration_, twist_duration_);
    //   f_star = T_WA_task_.linear() * (f_spiral.head<3>());
    //   m_star = T_WA_task_.linear() * f_spiral.tail<3>();
    // }
  }

  f_star_zero.head<3>() = f_star_motion;
  f_star_zero.tail<3>() = m_star;

  f_d.setZero();
  f_d.head<3>() = f_star_active_force;

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() * (arm.modified_lambda_matrix_ * f_star_zero + f_d);
  
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
  T_7A = assist_arm_current_.inverse() * T_WA_task_;

  rot = dyros_math::getPhi(assist_arm_current_.linear(), assist_arm_origin_.linear());
  rot = assist_arm_origin_.linear().inverse()*rot;
  pos = assist_arm_current_.translation() - assist_arm_origin_.translation(); // w.r.t {B}
  // pos = assist_arm_origin_.linear().inverse()*pos; // w.r.t {E}
  pos = T_WA_assist_.linear().inverse()*pos;

  if (assist_arm_action_ == 1) // hold
  {
    f_star = PegInHole::keepCurrentPosition(assist_arm_origin_, assist_arm_current_, xd, 1000, 20);
  }
  else if (assist_arm_action_ == 2) // press
  {
    f_star = PegInHole::pressCubicEE(assist_arm_origin_, assist_arm_current_, xd, T_WA_assist_, 3.0, time.toSec(), arm.task_start_time_.toSec(), 2.0);
    f_star = T_WA_assist_.linear() * f_star;
  }

  m_star = PegInHole::keepCurrentOrientation(assist_arm_origin_, assist_arm_current_, xd, 3000, 15);

  reaction_force.head<3>() = assist_arm_current_.linear().inverse() * f_ext.head<3>();
  reaction_force.tail<3>() = assist_arm_current_.linear().inverse() * f_ext.tail<3>();

  reaction_force_sum = sqrt(pow(reaction_force(0), 2) + pow(reaction_force(1), 2));
  // if (time.toSec() > arm.task_start_time_.toSec() + 5.0)
  // {
  //   if (checkForceLimit(reaction_force_sum, friction_))
  //   {
  //     setSucceeded();
  //     std::cout << "HOLE IS DETECTED!" << std::endl;
  //     std::cout << "reaction_force: " << reaction_force.transpose() << std::endl;
  //   }
  // }

  // f_star.setZero();
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

bool AssembleGMMSpiralActionServer::estimateContactState()
{
  int n_laps;
  Estimator::LABEL torque_label, position_label;
  Estimator::CONTACT_STATE CS;
  double t_2pi, t_4pi;
  Eigen::VectorXd torque_input, position_input;

  t_2pi = pow((2 * M_PI), 2) * pitch_ / (4 * lin_vel_ * M_PI);
  t_4pi = pow((4 * M_PI), 2) * pitch_ / (4 * lin_vel_ * M_PI);

  torque_input.resize(14);
  position_input.resize(2);
  torque_input.setZero();
  position_init_.setZero();

  if (spiral_elapsed_time_ > t_2pi && spiral_elapsed_time_ <= t_4pi)
  {
    torque_init_ += torque_captured_;
    position_init_ += position_captured_;

    if ( t_4pi - spiral_elapsed_time_ < 0.001)
    {
      torque_init_ = torque_init_ / ((t_4pi - t_2pi) * 1000);
      position_init_ = position_init_ / ((t_4pi - t_2pi) * 1000);
      std::cout << "=====================" << std::endl;
      std::cout << "torque init for the task arm   : " << torque_init_.head<7>().transpose() << std::endl;
      std::cout << "torque init for the assist arm : " << torque_init_.tail<7>().transpose() << std::endl;
      std::cout << "position init for the both arm : " << position_init_.transpose() << std::endl;
    }
  }

  n_laps = Estimator::countCurrentSpiralLaps(pitch_, lin_vel_, spiral_elapsed_time_);

  if (n_laps <= 2)
  {
    CS = SEARCH;     
    torque_label = READY_FOR_ESTIMATION;
    position_label = READY_FOR_ESTIMATION;
  } 
    
  else
  {
    for (int i = 0; i < 14; i++)
    {
      torque_input(i) = abs(torque_captured_(i) - torque_init_(i));
    } 
    
    position_input = (position_captured_ - position_init_);

    torque_label = Estimator::torqueEstimator(torque_input, torque_model_small_, torque_model_large_);
    position_label = Estimator::positionEstimator(position_input, position_model_small_, position_model_medium_, position_model_large_);
    CS = Estimator::contactStateTable(torque_label, position_label);

    // When the peg is inserted without search motion
    // if(position_input(0) < -0.001) CS = INSERTION;
  }    

  save_contact_estimation << (int) CS << std::endl;
  save_captured_torque << torque_captured_.transpose()<<std::endl;
  save_captured_position << position_captured_.transpose()<<std::endl;
  // std::cout<<" ================================ "<<std::endl;
  // std::cout<<" torque   : "<< torque_label<<std::endl;
  // std::cout<<" position : " << position_label<<std::endl;
  // if (CS == SEARCH)           std::cout << "Current Contact State : "<< CS << std::endl;
  // else if (CS == CROSSING)      std::cout << "Current Contact State : "<< CS << std::endl;
  // else if (CS == INSERTION)   std::cout << "Current Contact State : "<< CS << std::endl;
  // else if (CS == DEVIATION)   std::cout << "Current Contact State : "<< CS << std::endl;
    

  return true;
}

void AssembleGMMSpiralActionServer::setSucceeded()
{
  result_.is_completed = true;
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleGMMSpiralActionServer::setAborted()
{
  // result_.is_completed = true;
  as_.setAborted(result_);
  control_running_ = false;
}