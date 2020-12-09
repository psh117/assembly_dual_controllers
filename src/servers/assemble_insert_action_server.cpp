#include <assembly_dual_controllers/servers/assemble_insert_action_server.h>

AssembleInsertActionServer::AssembleInsertActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleInsertActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleInsertActionServer::preemptCallback, this));
  as_.start();
}

void AssembleInsertActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleInsert goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleInsertActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  total_action_start_time_ = ros::Time::now().toSec();
  origin_ = mu_[goal_->arm_name]->transform_;
  
  duration_ = goal_->duration;
  insertion_force_ = goal_->insertion_force;
  std::cout<<"insertion_force: "<<insertion_force_<<std::endl;
  
  wiggle_motion_ = goal_->wiggle_motion;
  wiggle_motion_z_axis_ = goal_->wiggle_motion_z_axis;
  yawing_motion_ = goal_->yawing_motion;

  connector_type_ = goal_ -> connector_type;
  bolting_minimum_depth_ = goal_->bolting_minimum_depth;

  yawing_angle_ = goal_->yawing_angle * DEG2RAD; //radian
  init_yaw_angle_ = mu_[goal_->arm_name]->q_(6,0); //radian
  bolting_vel_threshold_ = goal_->bolting_vel_threshold;
  time_limit_ = goal_->time_limit;
  if(time_limit_ == 0)
  {
    std::cout<<"Set Time Limit As Default Value, 1000000"<<std::endl;
    time_limit_ = 1000000.0;
  }
  std::cout<<goal_->arm_name<<std::endl;


  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"init_yaw_angle: "<<init_yaw_angle_<<std::endl;
  // std::cout<<"init_yaw_goal: "<<yawing_angle_<<std::endl;
  // std::cout<<"sum: "<<yawing_angle_ + init_yaw_angle_<<std::endl;
  
  if (yawing_angle_ + init_yaw_angle_ > yaw_up_limit_) 
  {
    std::cout<<"\nover joint upper limit: "<<yaw_up_limit_<<std::endl;
    yawing_angle_ = yaw_up_limit_ - 0.1 - init_yaw_angle_;
  }
  else if (yawing_angle_ + init_yaw_angle_ < yaw_low_limit_ )
  {
    std::cout<<"\nunder joint lower limit: "<<yaw_low_limit_<<std::endl;
    yawing_angle_ = yaw_low_limit_ + 0.1 - init_yaw_angle_;
  }
  // std::cout<<"\nfinal_yaw_goal: "<<yawing_angle_<<std::endl;
  // std::cout<<"sum: "<<yawing_angle_ + init_yaw_angle_<<std::endl;
  // std::cout<<"-------------------------------------"<<std::endl;

  flange_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  flange_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  flange_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  flange_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  flange_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  flange_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  flange_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;

  T_7A_.linear() = flange_to_assembly_quat_.toRotationMatrix();
  T_7A_.translation() = flange_to_assembly_point_;
 
  T_WA_ = origin_*T_7A_;

  wiggle_angle_ = 2.5*DEG2RAD;
  wiggle_angular_vel_ = 4*M_PI; // 2pi / s
  
  // To define parameters of wiggle motion along z-axis
  if(init_yaw_angle_ + wiggle_angle_ >= yaw_up_limit_)
  {
    wiggle_z_axis_param_.a = 1;
    wiggle_z_axis_param_.b = init_yaw_angle_ - wiggle_angle_;
    wiggle_z_axis_param_.t_offset = 2*M_PI/wiggle_angular_vel_/4;
    std::cout<<"near upper joint limit"<<std::endl;
  }
  else if(init_yaw_angle_ - wiggle_angle_ <= yaw_low_limit_)
  {
    wiggle_z_axis_param_.a = -1;
    wiggle_z_axis_param_.b = init_yaw_angle_ + wiggle_angle_;
    wiggle_z_axis_param_.t_offset = 2*M_PI/wiggle_angular_vel_/4;
    std::cout<<"near lower joint limit"<<std::endl;
  }
  else
  {
    wiggle_z_axis_param_.a = 1;
    wiggle_z_axis_param_.b = 0;
    wiggle_z_axis_param_.t_offset = 0;
  }


  if(wiggle_motion_ && yawing_motion_) std::cout<<"wiggle & yawing motion is activated"<<std::endl;
  else if(wiggle_motion_) std::cout<<"wiggle motion is activated"<<std::endl;
  else if(yawing_motion_) std::cout<<"yawing motion is activated"<<std::endl;
  else  std::cout<<"no optional motions"<<std::endl;

  // Eigen::Isometry3d T_AE;
  // Eigen::Vector3d P_ae;
  // double dis;
  // T_AE = T_7A_.inverse();
  // P_ae = T_AE.translation();
  // dis = sqrt(P_ae(0)*P_ae(0) + P_ae(1)*P_ae(1));

  // if(dis <= 0.005)
  // {
  //   mode_ = SIMPLE;
  //   std::cout<<"Insert the part in the simple method"<<std::endl;
  // } 
  // else
  // {
  //   mode_ = COMPLEX;
  //   std::cout<<"Insert the part in the complex method"<<std::endl;
  // } 

  U_EA_ = PegInHole::getNormalVector(T_7A_.translation());
  U_dir_ = T_7A_.linear().col(2);

  std::cout<<"------------------"<<std::endl;
  if(connector_type_ == 1) std::cout<<"BOLT INSERT STARTS"<<std::endl;
  q_init_ = mu_[goal_->arm_name]->q_;
  q_null_target_ = getNullSpaceJointTarget(rbdl_panda_.getJointLimit(), q_init_);
  std::cout<<"q_init_        : "<< q_init_.transpose()*RAD2DEG<<std::endl;
  std::cout<<"q_null_target_ : "<< q_null_target_.transpose()*RAD2DEG<<std::endl;

  count_ = 1;
  bolting_stop_count_ = 0;
  
  insert_force_error_.setZero();

  control_running_ = true;

}

void AssembleInsertActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleInsertActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;
    
  if (!as_.isActive())
    return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleInsertActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleInsertActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
      return false; 
      
  // auto & mass = arm.mass_matrix_;
  // auto & rotation = arm.rotation_;
  // auto & position = arm.position_;
  auto & jacobian = arm.jacobian_;
  // auto & tau_measured = arm.tau_measured_;
  // auto & gravity = arm.gravity_;
  auto & xd = arm.xd_; //velocity
  
  Eigen::Vector3d f_star, m_star, m_wig, m_yaw;;
  Eigen::Vector3d m_dir; 
  Eigen::Vector6d f_star_zero;
  Eigen::Vector3d p_init_a, p_cur_a;
  
  double run_time;
  double bolting_vel;
  int max_bolting_stop_count = 800;

  current_ = arm.transform_;
  run_time = time.toSec() - arm.task_start_time_.toSec();

  switch (connector_type_)
  {
    case 0: // pin, bracket
      if(timeOut(time.toSec(), arm.task_start_time_.toSec(), duration_ +0.01))
      {
        std::cout<<"INSERTION IS DONE"<<std::endl;
        std::cout<<"run time: "<< run_time<<std::endl;
        savePosture();
        setSucceeded();
      }

      f_star = PegInHole::pressCubicEE(origin_, current_, xd, T_WA_, insertion_force_, time.toSec(), arm.task_start_time_.toSec(), duration_/2, 1.0);

      f_star = T_WA_.linear()*f_star;     
      insert_force_error_.head<3>() += f_star -arm.f_ext_.head<3>();  
  
      break;
    
    case 1:

      p_init_a = T_7A_.inverse().translation(); //{E} wrt {A}
      p_cur_a = (T_WA_.inverse() * current_).translation();
      insertion_depth_ = abs(p_cur_a(2) - p_init_a(2));
            
      bolting_vel = (T_WA_.linear().inverse()*arm.xd_lpf_.head<3>())(2);      
      bolting_vel = dyros_math::lowPassFilter(bolting_vel, bolting_vel_prev_, 0.001, 2.0);
      bolting_vel_prev_ = bolting_vel;
      
      if(run_time > 0.5 && insertion_depth_ >= bolting_minimum_depth_ && abs(bolting_vel) <= bolting_vel_threshold_)
      {
        std::cout << "BOLTING IS DONE" << std::endl;
        std::cout << "displacement: " << abs(p_cur_a(2) - p_init_a(2)) << std::endl;
        std::cout << "run time : " << run_time << std::endl;
        std::cout<< "position error : "<< (current_.translation() - origin_.translation()).transpose()<<std::endl;
        setSucceeded();
      }
      if(run_time > time_limit_)
      {
        std::cout<<"TIME OUT"<<std::endl;
        setAborted();
      }

      // if(abs(bolting_vel) <= bolting_vel_threshold_)  bolting_stop_count_++;
      // else                                            bolting_stop_count_ = 0;

  
      f_star = PegInHole::pressCubicEE(origin_, current_, xd, T_WA_, insertion_force_, time.toSec(), arm.task_start_time_.toSec(), duration_/2);
      f_star = T_WA_.linear()*f_star;
      
      // Eigen::Vector3d phi;
      // Eigen::Matrix3d target_rot;
      // target_rot << 0, 0.7902, 0.6128, 0, -0.6128, 0.7902, 1.0000, 0, 0;

      // phi = dyros_math::getPhi(current_.linear(), target_rot);
      
      // std::cout<<"=================="<<std::endl;
      // std::cout<<"phi : "<< phi.transpose()<<std::endl;
      // std::cout<<"target rot : \n"<<target_rot<<std::endl;
      // std::cout<<"current rot : \n"<<current_.linear()<<std::endl;
      // std::cout<<bolting_vel<<" ," <<bolting_stop_count_<<std::endl;
      save_insert_pose_data << (T_WA_.linear().inverse()*arm.position_).transpose()<<" "<<(T_WA_.linear().inverse()*arm.position_lpf_).transpose()<<std::endl;
      save_insertion_vel << (T_WA_.linear().inverse()*xd.head<3>()).transpose()<<" "<<(T_WA_.linear().inverse()*arm.xd_lpf_.head<3>()).transpose()<<std::endl;
      // save_rotation_error << phi.transpose()<<std::endl;
  }

  
  if(wiggle_motion_ && yawing_motion_)
  {
    m_wig = PegInHole::generateWiggleMotionEE(origin_, current_, T_7A_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec());
    m_yaw = PegInHole::generateYawingMotionEE(origin_, current_, T_7A_, xd, yawing_angle_, duration_, time.toSec(), arm.task_start_time_.toSec());
    m_star = m_wig + m_yaw;
    m_star = T_WA_.linear()*m_star;
  }
  else if(yawing_motion_ )
  {
    m_star = PegInHole::generateYawingMotionEE(origin_, current_, T_7A_, xd, yawing_angle_, duration_, time.toSec(), arm.task_start_time_.toSec());
    m_star = T_WA_.linear()*m_star;
  }
  else if(wiggle_motion_)
  {
    m_star = PegInHole::generateWiggleMotionEE(origin_, current_, T_7A_, xd, wiggle_angle_, wiggle_angular_vel_, time.toSec(), arm.task_start_time_.toSec());
    m_star = T_WA_.linear()*m_star;
  }
  else if(wiggle_motion_z_axis_)
  {
    m_star = PegInHole::generateWiggleMotionEE_Zaxis(origin_, current_, T_7A_, xd, wiggle_angle_, wiggle_angular_vel_,
                                                    wiggle_z_axis_param_.a, wiggle_z_axis_param_.b, wiggle_z_axis_param_.t_offset, 
                                                    time.toSec(), arm.task_start_time_.toSec());
    m_star = T_WA_.linear()*m_star;
  }
  else  m_star = keepCurrentOrientation(origin_, current_ , xd, 2000, 15);

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  
  // f_star_zero.setZero();

  // tau_null_cmd_ = PegInHole::nullSpaceJointTorque(arm.q_, q_init_, q_null_target_, arm.qd_, time.toSec(), total_action_start_time_, duration_*2);
  // std::cout<<"tau_null_cmd : "<< tau_null_cmd_.transpose()<<std::endl;
  // std::cout<< "force cmd : " << f_star.transpose()<<std::endl;
  // std::cout<< "moment cmd : "<< m_star.transpose()<<std::endl;
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * arm.modified_lambda_matrix_*f_star_zero; //+ arm.null_projector_*tau_null_cmd_;
  arm.setTorque(desired_torque);
  save_reaction_force << arm.f_ext_.transpose()<<std::endl;
  
  // insert_pose_data << arm.position_<<std::endl;
  // save_insert_pose_data << p_cur_a.transpose()<<std::endl;
  // std::cout<<"f_star: "<< f_star.transpose()<<std::endl;
  return true;
}

void AssembleInsertActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void AssembleInsertActionServer::setAborted()
{
  as_.setAborted(result_);
  control_running_ = false;
}

void AssembleInsertActionServer::savePosture()
{
  current_;
  
  result_.end_pose.position.x = current_.translation()(0);
  result_.end_pose.position.y = current_.translation()(1);
  result_.end_pose.position.z = current_.translation()(2);
  Eigen::Quaterniond temp(current_.linear());
  result_.end_pose.orientation.x = temp.x();
  result_.end_pose.orientation.y = temp.y();
  result_.end_pose.orientation.z = temp.z();
  result_.end_pose.orientation.w = temp.w();
}