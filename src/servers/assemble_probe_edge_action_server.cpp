#include <assembly_dual_controllers/servers/assemble_probe_edge_action_server.h>

AssembleProbeEdgeActionServer::AssembleProbeEdgeActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&AssembleProbeEdgeActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&AssembleProbeEdgeActionServer::preemptCallback, this));
  as_.start();
}

void AssembleProbeEdgeActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if ( mu_.find(goal_->arm_name) != mu_.end() ) 
  {
    ROS_INFO("AssembleProbeEdge goal has been received.");
  } 
  else 
  {
    ROS_ERROR("[AssembleProbeEdgeActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return ;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();
  
  origin_ = mu_[goal_->arm_name]->transform_;  

  contact_force_ = goal_-> contact_force;
  contact_loss_threshold_ = goal_ -> contact_loss_threshold;
  attraction_force_ = goal_-> attraction_force;
  object_location_(0) = goal_-> object_location.data[0];
  object_location_(1) = goal_-> object_location.data[1];
  probing_speed_ = goal_->probing_speed;

  ee_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  ee_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  ee_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  ee_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  ee_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  ee_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  ee_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;

  T_EA_.linear() = ee_to_assembly_quat_.toRotationMatrix();
  T_EA_.translation() = ee_to_assembly_point_;
  
  T_WA_ = origin_*T_EA_;
 
  if(probe_ft_data.is_open())     probe_ft_data.close();
  if(probe_pr_data.is_open())     probe_pr_data.close();
  if(probe_vel_data.is_open())    probe_vel_data.close();
  if(probe_ft_cmd_data.is_open()) probe_ft_cmd_data.close();
    
  probe_ft_data.open("probe_ft_data.txt");
  probe_pr_data.open("probe_pr_data.txt");
  probe_vel_data.open("probe_vel_data.txt");
  probe_ft_cmd_data.open("probe_ft_cmd_data.txt");

  state_ = PROBE_STATE::READY;
  is_mode_changed_ = true;
  probe_index_ = 0;
  dir_index_ = 0;
  last_probe_index_ = 4;
  probe_origin_ = origin_.translation();
  probe_origin_(2) += 0.005;
  generateProbingSequence(object_location_);

  control_running_ = true;
  
  std::cout<<"PROBING START"<<std::endl;
  // std::cout<<"object_location: "<<object_location_.transpose()<<std::endl;
  // std::cout<<"contact force: "<< contact_force_<<std::endl;
  // std::cout<<"attaction force: "<<attraction_force_<<std::endl;
  // std::cout<<"probing_speed: "<<probing_speed_<<std::endl;
}

void AssembleProbeEdgeActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool AssembleProbeEdgeActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if ( mu_.find(goal_->arm_name) != mu_.end() ) {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  } else {
    ROS_ERROR("[AssembleProbeEdgeActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}


bool AssembleProbeEdgeActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
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
  
  Eigen::Vector3d f_a, f_star, m_star;
  Eigen::Vector3d f_att, f_prob;
  Eigen::Vector6d f_ext, task_vel_a;
  Eigen::Vector3d obj_dir, abs_normal_vector;
  double run_time;
  double speed;
  int dir;

  f_ext = arm.f_ext_;
  current_ = arm.transform_;
  task_vel_a.head<3>() = T_WA_.linear().inverse()*arm.xd_.head<3>();
  task_vel_a.tail<3>() = T_WA_.linear().inverse()*arm.xd_.tail<3>();

  if(is_mode_changed_)
  {
    arm.task_start_time_ = time;
    origin_ = arm.transform_;
    is_mode_changed_ = false;

    
    normal_vector_ << updateNormalVector((PROBE_DIRECTION)probing_sequence_(dir_index_)), 0;
    abs_normal_vector << abs(normal_vector_(0)), abs(normal_vector_(1)), abs(normal_vector_(2));
    selection_matrix_ = Eigen::Matrix3d::Identity() - Eigen::Matrix3d(abs_normal_vector.asDiagonal());
    
    // std::cout<<"dir_index: "<< dir_index_<<std::endl;
    // std::cout<<"normal vector: "<< normal_vector_.transpose()<<std::endl;
    // std::cout<<"abs normal vector: "<< normal_vector_.transpose()<<std::endl;
    // std::cout<<"selection matrix: \n"<< selection_matrix_<<std::endl;
  }

  run_time = time.toSec() - arm.task_start_time_.toSec();
  
  switch (state_)
  {
    case READY:
      obj_dir.setZero();
      obj_dir.head<2>() = object_location_;

      f_star = PegInHole::straightMotionEE(origin_, current_, xd, T_EA_, obj_dir, 0.01, time.toSec(), arm.task_start_time_.toSec());
      f_star = T_WA_.linear() * f_star;
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 200, 5);

      if(Criteria::detectObject(origin_, current_, f_ext.head<3>(), T_WA_, contact_force_)) //when lost contact
      {
        std::cout << "READY TO PROBE" << std::endl;
        state_ = PROBE_ON_THE_EDGE;
        is_mode_changed_ = true;
        init_force_ext_ = f_ext;
        std::cout<<"init_force_ext: "<< init_force_ext_.head<3>().transpose()<<std::endl;
      }
      break;

    case PROBE_ON_THE_EDGE:
      
      // f_ext = f_ext - init_force_ext_;      
      dir = probing_sequence_(dir_index_);
      speed = probing_speed_;
      origin_.translation()(2) = probe_origin_(2);     

      // f_att = generateNormalForce(updateNormalVector((PROBE_DIRECTION)probing_sequence_(dir_index_)), attraction_force_);      
      f_att = generateNormalForce(normal_vector_.head<2>(), attraction_force_);      
      f_prob = generateProbingForce(origin_, current_, xd, T_EA_, dir, speed, time.toSec(), arm.task_start_time_.toSec());
      f_a = f_att + selection_matrix_*f_prob;
      f_star = T_WA_.linear() * f_a;
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 100, 1);
      
      // if(run_time > cnt_max/1000 && !Criteria::detectObject(origin_, current_, f_ext.head<3>(), T_WA_, contact_loss_threshold_)) //when lost contact
      if(run_time > 0.1 && Criteria::contactLossInProbing(task_vel_a.head<3>(), normal_vector_, contact_loss_threshold_))
      {
        std::cout << "LOST CONTACT" << std::endl;
        state_ = LOST_CONTACT;
        is_mode_changed_ = true;
      }

      if (dir_index_ == last_probe_index_ && Criteria::reachGoal2D(probe_origin_, current_.translation(), 0.005, T_WA_))
      {
        state_ = COMPLETE;
        is_mode_changed_ = true;
        std::cout << "Probing is done" << std::endl;
      }
      break;
    
    case LOST_CONTACT:
      f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 1000, 10, 10, 0.1).head<3>();
      m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 1000, 10, 10, 0.1).tail<3>();

      if(run_time > 0.5)
      {
        std::cout<<"Try to contact again"<<std::endl;
        state_ = RECOVER_CONTACT;
        is_mode_changed_ = true;
      }

      break;

    case RECOVER_CONTACT:
      dir = probing_sequence_(dir_index_);
      speed = -probing_speed_/10;
     
      origin_.translation()(2) = probe_origin_(2);     

      // f_att = generateNormalForce(updateNormalVector((PROBE_DIRECTION)probing_sequence_(dir_index_)), attraction_force_);
      f_att = generateNormalForce(normal_vector_.head<2>(), attraction_force_/10);      
      f_prob = generateProbingForce(origin_, current_, xd, T_EA_, dir, speed, time.toSec(), arm.task_start_time_.toSec());
      
      f_a = f_att + selection_matrix_*f_prob;
      f_star = T_WA_.linear() * f_a;
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 200, 1);

      if(run_time > 0.1 && Criteria::detectObject(origin_, current_, f_ext.head<3>(), T_WA_, contact_force_)) //when lost contact
      {
        state_ = PROBE_ON_THE_EDGE;
        std::cout << "Contact!!" << std::endl;
        is_mode_changed_ = true;
        dir_index_++;
        if (dir_index_ >= probing_sequence_.rows())
        {
          dir_index_ = probing_sequence_.rows();      
        }
      }
      // std::cout<<"---------------------"<<std::endl;
      // std::cout<<"normal vector: "<< updateNormalVector((PROBE_DIRECTION)probing_sequence_(dir_index_)).transpose()<<std::endl;
      // std::cout<<"abs_normal vector: "<< normal_vector.transpose()<<std::endl;
      // std::cout<<"f_att: "<<f_att.transpose()<<std::endl;
      // std::cout<<"f_prob: "<<f_prob.transpose()<<std::endl;
      // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
      // std::cout<<"selection matrix: \n"<< selection_matrix<<std::endl;
      break;

    case FAIL:
      std::cout<<"FAIL TO DETECT OBJECT"<<std::endl;
      setAborted();
      break;

    case COMPLETE:
      std::cout<<"FINISHED PROBING EDGES"<<std::endl;
      setSucceeded();
      break;
  }
  
  if(run_time > 100.0) setAborted();

  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
  
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  
  // if(state_ == PROBE_ON_THE_EDGE) f_star_zero_.setZero();  
  // std::cout<<"f_measured: "<<f_measured_.transpose()<<std::endl;
  // std::cout<<"f_filtered: "<<f_ext.transpose()<<std::endl;
  
  // if(state_ == LOST_CONTACT) f_star_zero_.setZero();
  if(state_ == PROBE_ON_THE_EDGE)
  {
    probe_pr_data << (current_ * T_EA_).translation().transpose() << std::endl;
    probe_ft_cmd_data << (T_WA_.inverse()* f_star.head<3>()).transpose() << std::endl;
  }
  probe_ft_data << (T_WA_.inverse()*f_ext.head<3>()).transpose() << std::endl;
  probe_vel_data << task_vel_a.transpose()<<std::endl;
  // if(dir_index_ == 1) f_star_zero_.setZero();
  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero_;
  arm.setTorque(desired_torque);

  
  // std::cout << "---------------------" << std::endl;
  // std::cout << "state: "<<state_<<std::endl;
  // std::cout << "abs_normal vector: " << normal_vector_.transpose() << std::endl;
  // std::cout << "f_att: " << f_att.transpose() << std::endl;
  // std::cout << "f_prob: " << f_prob.transpose() << std::endl;
  // std::cout << "f_a: " << f_a.transpose() << std::endl;
  // std::cout << "f_star: " << f_star.transpose() << std::endl;
  // std::cout << "selection matrix: \n"<< selection_matrix_ << std::endl;
  // probe_ft_data << (f_ext - init_force_ext_).transpose() << std::endl;
  return true;
}

void AssembleProbeEdgeActionServer::setSucceeded()
{
  as_.setSucceeded();
  control_running_ = false;
}
void AssembleProbeEdgeActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}

void AssembleProbeEdgeActionServer::generateProbingSequence(const Eigen::Vector2d &object_location)
{
  if(object_location(0) == 1)        probing_sequence_ << (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT;
  else if(object_location(0) == -1)  probing_sequence_ << (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT;
  else if(object_location(1) == 1)   probing_sequence_ << (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN;
  else if(object_location(1) == -1)  probing_sequence_ << (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP;
}

Eigen::Vector2d AssembleProbeEdgeActionServer::updateNormalVector(PROBE_DIRECTION probing_direction)
{
  Eigen::Vector2d n;
  if(probing_direction == PROBE_DIRECTION::UP)       n <<0, -1;  //Up --> -y
  else if(probing_direction == PROBE_DIRECTION::RIGHT)  n << -1, 0; //RIGHT --> -x
  else if(probing_direction == PROBE_DIRECTION::LEFT)  n << 1, 0;  //LEFT --> +x
  else if(probing_direction == PROBE_DIRECTION::DOWN)  n << 0, 1;  //DOWN --> +y
  return n;
}

Eigen::Vector3d AssembleProbeEdgeActionServer::generateNormalForce(const Eigen::Vector2d &normal_vector,
                                                                   const double force)
{
  Eigen::Vector3d normal_force, v;
  v << normal_vector, 0.0;
  normal_force = v * force;
  return normal_force;
}

Eigen::Vector3d AssembleProbeEdgeActionServer::generateProbingForce(const Eigen::Isometry3d &origin,
                                                                    const Eigen::Isometry3d &current,
                                                                    const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                                    const Eigen::Isometry3d &T_ea,
                                                                    const int probing_dir,
                                                                    const double probing_speed, //set positive
                                                                    const double t,
                                                                    const double t_0)
{
  Eigen::Vector3d target_dir, f_star;
  double speed;
  int dir;

  if (probing_dir == PROBE_DIRECTION::DOWN || probing_dir == PROBE_DIRECTION::RIGHT)
    speed = -probing_speed;
  else
    speed = probing_speed;

  if (probing_dir == PROBE_DIRECTION::LEFT || probing_dir == PROBE_DIRECTION::RIGHT)
    target_dir << 0, 1, 0; // move long y-axis
  else if (probing_dir == PROBE_DIRECTION::UP || probing_dir == PROBE_DIRECTION::DOWN)
    target_dir << 1, 0, 0; //move long x-axis

  f_star = PegInHole::straightMotionEE(origin, current, xd, T_ea, target_dir, speed, t, t_0);
  
  // for(int i = 0; i < 2; i++) f_star(i) = target_dir(i)*f_star(i);
  
  return f_star; //w.r.t Assembly frame s.t., f_a
}
  