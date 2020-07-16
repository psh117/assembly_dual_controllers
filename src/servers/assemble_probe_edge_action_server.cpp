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

  contact_force_ = goal_->contact_force;
    
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
  
  if(probe_ft_data.is_open())  probe_ft_data.close();
  if(probe_ft_data_lpf.is_open()) probe_ft_data_lpf.close();
    
  probe_ft_data.open("probe_ft_data.txt");
  probe_ft_data_lpf.open("probe_ft_data_lpf.txt");

  control_running_ = true;

  state_ =(int)PROBE_STATE::SEARCH_OBJECT;
  search_dir_ = setSearchDirection(search_origin_);
  is_mode_changed_ = true;
  search_index_ = 0;
  probe_index_ = 0;
  blocking_force_ = 12.0;
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
  
  Eigen::Vector3d f_star, m_star;
  Eigen::Vector3d f_att, f_prob;
  Eigen::Vector6d f_star_zero;
  Eigen::Vector6d f_lpf;
  double search_range;
  double search_duration;

  f_lpf = arm.f_measured_filtered_;
  current_ = arm.transform_;
  search_range = 0.05;
  search_duration = 0.5;

  if(is_mode_changed_)
  {
    arm.task_start_time_ = time;
    origin_ = arm.transform_;
    is_mode_changed_ = false
    std::cout<<state_.c_str()<<std::endl;
  }

  switch (state_)
  {
    case SEARCH_OBJECT:
      Eigen::Vector3d target;
      target.setZero();
      target(search_index_) = search_dir_(search_index_)*range;
      target = origin_+target;

      if(detectObject(origin_, cubic, f_lpf.head<3>(), T_WA_, 15.0))
      {
        is_mode_changed_ = true;
        state_ = BACK_TO_ORIGIN;
        detect_object_(search_index_) = 1; //there exist object 
      }

      if(timeOut(time.toSec(), arm.task_start_time_.toSec(), search_duration))
      {
        is_mode_changed_ = true;
        state_ = BACK_TO_ORIGIN;
        detect_object_(search_index_) = 0; //no object
      }

      f_star = PegInHole::threeDofMove(origin_, current_, target, xd, time.toSec(), arm.task_end_time_, search_duration);    
      f_star = T_WA_.linear()*f_star;
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 200, 5);
      break;

    case BACK_TO_ORIGIN:
      Eigen::Vector3d target;
      target.setZero();
      target(search_index_) = -search_dir_(search_index_)*range;
      target = origin_+target;

      if (timeOut(time.toSec(), arm.task_start_time_.toSec(), search_duration + 0.01))
      {
        state_ = SEARCH_OBJECT;
        is_mode_changed_ = true;
        search_index_ ++;
      }

      if(search_index_ == 2)
      {
        int sum = 0;
        sum = detect_object_(0) + detect_object_(1); //detect_objed_ = [0, 0] , [0, 1], [1, 0], [1, 1]
        if(sum == 2) state_ = DETECT_HOLE;
        else if(sum == 1) state_ = PROBE_ON_THE_EDGE;
        else state_ = FAIL;
        probe_origine_ = current_.translation();
      }

      f_star = PegInHole::threeDofMove(origin_, current_, target, xd, time.toSec(), arm.task_end_time_, search_duration);    
      f_star = T_WA_.linear()*f_star;
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 200, 5);
      break;

    case PROBE_ON_THE_EDGE:
      double contact_loss = 11.995; //9.995;
      double speed;
      int dir;

      dir = probing_sequence_(dir_index_);
      speed = 0.1;

      f_att = generateNormalForce(updateNormalVector((PROBE_DIRECTION)probing_sequence_(dir_index_)), 10.0);      
      f_prob = generateProbingForce(origin_, current_, xd, T_EA_, probe_dir, speed, time.toSec(), arm.task_start_time_.toSec());

      f_star = f_att + f_prob;
      f_star = T_WA_.linear() * f_star;
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 200, 5);

      if(!Criteria::detectObject(origin_, current_, f_lpf, T_WA_, contact_loss)) //when lost contact
      {
        std::cout << "LOST CONTACT" << std::endl;
        state_ = LOST_CONTACT;
        is_mode_changed_ = true;
      }

      if (dir_index_ == last_probe_index_ && Criteria::reachGoal3D(probe_origine_, current_.translation(), 0.002, T_WA_))
      {
        control_mode_ = COMPLETE;
        is_mode_changed_ = true;
        std::cout << "Probing is done" << std::endl;
      }
      // std::cout<<"---------------------"<<std::endl;
      // std::cout<<"normal vector: "<< FailRecovery::updateNormalVector((FailRecovery::Direction)probing_sequence_(dir_index_)).transpose()<<std::endl;
      // std::cout<<"f_att: "<<f_att.transpose()<<std::endl;
      // std::cout<<"f_prob: "<<f_prob.transpose()<<std::endl;
      // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
      // std::cout<<"dir_index: "<<dir_index_<<std::endl;

      break;

    case LOST_CONTACT:
      double contact_threshold = 12.0;
      double speed;
      int dir;

      dir = probing_sequence_(dir_index_);
      speed = -0.1;
     
      origin_.translation()(2) = probe_origine_(2);     

      f_att = generateNormalForce(updateNormalVector((PROBE_DIRECTION)probing_sequence_(dir_index_)), 20.0);
      f_prob = generateProbingForce(origin_, current_, xd, T_EA_, probe_dir, speed, time.toSec(), arm.task_start_time_.toSec());

      f_prob.head<2>() = 0.1 * f_prob.head<2>();
      f_star = f_att + f_prob;
      m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 200, 5);

      if(Criteria::detectObject(origin_, current_, f_lpf, T_WA_, contact_threshold)) //when lost contact
      {
        control_mode_ = "probing_edge";
        std::cout << "Contact!!" << std::endl;
        std::cout << contact << std::endl;
        is_mode_changed_ = true;
        dir_index_++;
        if (dir_index_ >= probing_sequence_.rows())
          dir_index_ = probing_sequence_.rows();      
      }
      // std::cout<<"---------------------"<<std::endl;
      // std::cout<<"f_att: "<<f_att.transpose()<<std::endl;
      // std::cout<<"f_prob: "<<f_prob.transpose()<<std::endl;
      // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
      // std::cout<<"contact: "<< contact<<std::endl;
      // std::cout<<"obj_center: "<<obj_info_.head<2>().transpose()<<std::endl;
      // std::cout<<"current_pose: "<<arm_state_[active_arm_].xp.transpose()<<std::endl;
      // std::cout<<"probe direction: "<<dir<<std::endl;
      // std::cout<<"dir_index: "<<dir_index_<<std::endl;
      break;

    case FAIL:
      std::cout<<"FAIL TO DETECT OBJECT"<<std::endl;
      setAborted();
      break;
    case COMPLETE:
      std::cout<<"FINISHED PROBING EDGES"<<std::endl;
      setSucceeded();
    case DETECT_HOLE:
      std::cout<<"PEG IN INSERTED INTO THE HOLE"<<std::endl;
      setSucceeded();
  }
  
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;
  
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  
  // f_star_zero.setZero();  
  // std::cout<<"f_measured: "<<f_measured_.transpose()<<std::endl;
  // std::cout<<"f_filtered: "<<f_lpf.transpose()<<std::endl;

  Eigen::Matrix<double,7,1> desired_torque = jacobian.transpose() * f_star_zero;
  arm.setTorque(desired_torque);

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

Eigen::Vector2d AssembleProbeEdgeActionServer::setSearchDirection(const Eigen::Vector3d &search_origin) //w.r.t assembly frame                                                                  
{
  Eigen::Vector2d search_dir;

  if(search_origin(0) > 0) search_dir(0) = 1.0;
  else search_dir(0) = -1.0;

  if(search_origin(1) > 0) search_dir(1) = 1.0;
  else search_dir(1) = -1.0;

  return search_dir;
}

void AssembleProbeEdgeActionServer::generateProbingSequence(const Eigen::Vector2d search_dir)
{
  if(search_dir(0) == 1)        probing_sequence_ << (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT;
  else if(search_dir(0) == -1)  probing_sequence_ << (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT;
  else if(search_dir(1) == 1)   probing_sequence_ << (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN;
  else if(search_dir(1) == -1)  probing_sequence_ << (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP;
}

Eigen::Vector2d AssembleProbeEdgeActionServer::updateNormalVector(Direction probing_direction)
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
                                                                    PROBE_DIRECTION probing_dir,
                                                                    const double probing_speed, //set positive
                                                                    const double t,
                                                                    const double t_0)
{
  Eigen::Vector3d target_dir, f_star;
  double speed;
  int dir;

  if (probing_dir == Direction::DOWN || probing_dir == Direction::RIGHT)  speed = -probing_speed;
  else                                                                    speed = probing_speed;

  if (probing_dir == Direction::LEFT || probing_dir == Direction::RIGHT) target_dir << 0, 1, 0; // move long y-axis
  else if (probing_dir == Direction::UP || probing_dir == Direction::DOWN) target_dir << 1, 0, 0; //move long x-axis

  f_star = PegInHole::straightMotionEE(origin, current, xd, T_ea, target_dir, speed, t, t_0);
  f_star = K_p * (x_d - current) + K_v * (-current_velocity);
  return f_star; //w.r.t Assembly frame s.t., f_a
}
  