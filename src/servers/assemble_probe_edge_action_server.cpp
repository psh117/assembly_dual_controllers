#include <assembly_dual_controllers/servers/assemble_probe_edge_action_server.h>

AssembleProbeEdgeActionServer::AssembleProbeEdgeActionServer(std::string name, ros::NodeHandle &nh,
                                                             std::map<std::string, std::shared_ptr<FrankaModelUpdater>> &mu)
    : ActionServerBase(name, nh, mu),
      as_(nh, name, false)
{
  as_.registerGoalCallback(boost::bind(&AssembleProbeEdgeActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&AssembleProbeEdgeActionServer::preemptCallback, this));
  as_.start();
}

void AssembleProbeEdgeActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if (mu_.find(goal_->arm_name) != mu_.end())
  {
    ROS_INFO("AssembleProbeEdge goal has been received.");
  }
  else
  {
    ROS_ERROR("[AssembleProbeEdgeActionServer::goalCallback] the name %s is not in the arm list.", goal_->arm_name.c_str());
    return;
  }

  mu_[goal_->arm_name]->task_start_time_ = ros::Time::now();

  World_to_robot_.linear() = Eigen::Matrix3d::Identity();
  if (goal_->arm_name == std::string("panda_right"))      World_to_robot_.translation() << 0.0, -0.3, 0.0;
  else if (goal_->arm_name == std::string("panda_left"))  World_to_robot_.translation() << 0.0, +0.3, 0.0;
  std::cout << "World to robot: \n"<< World_to_robot_.matrix() << std::endl;
  
  origin_ = mu_[goal_->arm_name]->transform_;
  probe_origin_ = origin_;
  contact_force_ = goal_->contact_force;
  contact_loss_threshold_ = goal_->contact_loss_threshold;
  attraction_force_ = goal_->attraction_force;
  object_location_(0) = goal_->object_location.data[0];
  object_location_(1) = goal_->object_location.data[1];
  probing_speed_ = goal_->probing_speed;
  width_ = goal_ ->furniture_width;
  height_ = goal_ ->furniture_height;
  pin_radius_ = goal_ ->pin_radius;
  offset_ = goal_ ->geometric_offset;

  ee_to_assembly_point_(0) = goal_->ee_to_assemble.position.x;
  ee_to_assembly_point_(1) = goal_->ee_to_assemble.position.y;
  ee_to_assembly_point_(2) = goal_->ee_to_assemble.position.z;
  ee_to_assembly_quat_.x() = goal_->ee_to_assemble.orientation.x;
  ee_to_assembly_quat_.y() = goal_->ee_to_assemble.orientation.y;
  ee_to_assembly_quat_.z() = goal_->ee_to_assemble.orientation.z;
  ee_to_assembly_quat_.w() = goal_->ee_to_assemble.orientation.w;
  
  T_EA_.linear() = ee_to_assembly_quat_.toRotationMatrix();
  T_EA_.translation() = ee_to_assembly_point_;
  
  T_WA_ = origin_ * T_EA_;


  state_ = PROBE_STATE::READY;
  is_mode_changed_ = true;
  probe_index_ = 0;
  dir_index_ = 0;
  last_probe_index_ = 4;
  probe_origin_pos_ = origin_.translation();
  probe_origin_quat_ = origin_.linear();
  max_nLoss_ = 2;
  nLoss_ = 0;

  probe_coeff_.resize(max_nLoss_, 2); // for the first order linear regression
  probe_dist_.resize(max_nLoss_);
  probe_coeff_.setZero();
  probe_dist_.setZero();

  generateProbingSequence(object_location_);

  control_running_ = true;

  std::cout << "PROBING START" << std::endl;
  std::cout<<"probing start point: "<< origin_.translation().transpose()<<std::endl;
  std::cout<<"object_location: "<<object_location_.transpose()<<std::endl;
  // std::cout<<"contact force: "<< contact_force_<<std::endl;
  std::cout << "attaction force: " << attraction_force_ << std::endl;
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

  if (mu_.find(goal_->arm_name) != mu_.end())
  {
    computeArm(time, *mu_[goal_->arm_name]);
    return true;
  }
  else
  {
    ROS_ERROR("[AssembleProbeEdgeActionServer::compute] the name %s is not in the arm list.", goal_->arm_name.c_str());
  }

  return false;
}

bool AssembleProbeEdgeActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  if (!as_.isActive())
    return false;

  // auto & mass = arm.mass_matrix_;
  // auto & rotation = arm.rotation_;
  // auto & position = arm.position_;
  auto &jacobian = arm.jacobian_;
  // auto & tau_measured = arm.tau_measured_;
  // auto & gravity = arm.gravity_;
  auto &xd = arm.xd_;
  auto &f_ext = arm.f_ext_;

  Eigen::Vector3d f_a, f_star, f_ext_selected, m_star;
  Eigen::Vector3d f_att, f_prob;
  Eigen::Vector6d task_vel_a;
  Eigen::Vector3d obj_dir, task_displacement;
  double run_time, speed;
  int dir;
  bool speed_violation, force_violation, pos_violation;
  Eigen::Matrix3d kp;
  Eigen::Vector3d save_position, renew_origin, cross_point;

  // f_ext = arm.f_ext_;
  current_ = arm.transform_;
  task_vel_a.head<3>() = T_WA_.linear().inverse() * arm.xd_.head<3>();
  task_vel_a.tail<3>() = T_WA_.linear().inverse() * arm.xd_.tail<3>();

  kp = Eigen::Matrix3d::Identity();

  if (is_mode_changed_)
  {
    arm.task_start_time_ = time;
    origin_ = arm.transform_;
    is_mode_changed_ = false;

    normal_vector_ << updateNormalVector((PROBE_DIRECTION)probing_sequence_(dir_index_)), 0;
    tangent_vector_ << updateTangentVector((PROBE_DIRECTION)probing_sequence_(dir_index_)), 0;
    abs_normal_vector_ << abs(normal_vector_(0)), abs(normal_vector_(1)), abs(normal_vector_(2));
    selection_matrix_ = Eigen::Matrix3d::Identity() - Eigen::Matrix3d(abs_normal_vector_.asDiagonal());

    if (state_ == LOST_CONTACT)
    {
      saveProbingResult(path_x_, path_y_, nLoss_, probe_coeff_, probe_dist_);
      path_x_.clear();
      path_y_.clear();
    }      
  }

  task_displacement = ((origin_ * T_EA_).inverse() * current_ * T_EA_).translation();
  run_time = time.toSec() - arm.task_start_time_.toSec();

  switch (state_)
  {
  case READY:
    obj_dir.setZero();
    obj_dir.head<2>() = object_location_;

    if (Criteria::detectObject(f_ext.head<3>(), T_WA_, contact_force_)) //when lost contact
    {
      std::cout << "READY TO PROBE" << std::endl;
      state_ = PROBE_ON_THE_EDGE;
      is_mode_changed_ = true;
      init_force_ext_ = f_ext;
      std::cout << "init_force_ext: " << init_force_ext_.head<3>().transpose() << std::endl;
      break;
    }

    f_star = PegInHole::straightMotionEE(origin_, current_, xd, T_EA_, obj_dir, 0.01, time.toSec(), arm.task_start_time_.toSec());
    f_star = T_WA_.linear() * f_star;
    m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 2000, 10);

    break;

  case PROBE_ON_THE_EDGE:
    dir = probing_sequence_(dir_index_);
    speed = probing_speed_;

    speed_violation = Criteria::contactLossInProbing(task_vel_a.head<3>(), normal_vector_, contact_loss_threshold_);
    force_violation = !Criteria::contactLossInProbing((T_WA_.linear().inverse() * f_ext.head<3>()), normal_vector_, -0.5);
    pos_violation = Criteria::contactLossInProbing(task_displacement, normal_vector_, 0.01);
    // pos_violation = Criteria::contactLossInProbing(task_displacement, normal_vector_, 1.0);

    // if (dir_index_ == last_probe_index_ && Criteria::reachGoal2D(probe_origin_pos_, current_.translation(), 0.01, T_WA_))
    // {
    //   state_ = COMPLETE;
    //   is_mode_changed_ = true;
    //   std::cout << "Probing is done" << std::endl;
    //   break;
    // }
    
    if (run_time > 0.5 && (speed_violation || force_violation || pos_violation))
    {
      std::cout << "velocity violation  : " << speed_violation << std::endl;
      std::cout << "force violation     : " << force_violation << std::endl;
      std::cout << "pose violation      : " << pos_violation << std::endl;

      std::cout << "LOST CONTACT" << std::endl;
      state_ = LOST_CONTACT;
      is_mode_changed_ = true;
      nLoss_++;
      break;
    }

    f_att = generateNormalForce(attraction_force_, contact_loss_threshold_, xd, T_WA_, normal_vector_, 5, time.toSec(), arm.task_start_time_.toSec());
    f_prob = generateProbingForce(origin_, current_, xd, T_EA_, dir, speed, time.toSec(), arm.task_start_time_.toSec());
    f_a = f_att + selection_matrix_ * f_prob;
    f_star = T_WA_.linear() * f_a; //+ init_force_ext_.head<3>();
    m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 2000, 10);

    // save_position = World_to_robot_ * current_.translation();
    save_position = (T_WA_.inverse()*current_*T_EA_).translation(); // probing path w.r.t {A}
    path_x_.push_back(save_position(0));
    path_y_.push_back(save_position(1));

    
    // std::cout<<"normal vector : "<< normal_vector_.transpose()<<std::endl;
    // std::cout<<"f_att         : "<<f_att.transpose()<<std::endl;
    // std::cout<<"f_prob        : "<<f_prob.transpose()<<std::endl;
    // std::cout<<"f_a           : "<<f_a.transpose()<<std::endl;
    // std::cout<<"f_star        : "<<f_star.transpose()<<std::endl;
    // std::cout<<"origin        : "<< origin_.translation().transpose()<<std::endl;
    // std::cout<<"current_      : "<< current_.translation().transpose()<<std::endl;
    // std::cout<<"---------------------"<<std::endl;
    break;

  case LOST_CONTACT:
    if (run_time > 0.25)
    {
      std::cout << "Try to contact again" << std::endl;
      state_ = RECOVER_CONTACT;
      is_mode_changed_ = true;
      init_force_ext_ = f_ext;
      break;
    }

    f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 1000, 15, 2000, 15).head<3>();
    m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 1000, 15, 2000, 15).tail<3>();

    if (nLoss_ == max_nLoss_)
    {
      state_ = COMPLETE;
      is_mode_changed_ = true;
      std::cout << "Probing is done" << std::endl;
      break;
    }

    break;

  case RECOVER_CONTACT:
    dir = probing_sequence_(dir_index_);
    speed = -probing_speed_ / 2;

    if (run_time > 0.2 && Criteria::detectObject(f_ext.head<3>(), T_WA_, contact_force_)) //when lost contact
    {
      state_ = CONTACT_AGAIN;
      std::cout << "Contact!!" << std::endl;
      is_mode_changed_ = true;
      dir_index_++;
      if (dir_index_ >= probing_sequence_.rows())
      {
        dir_index_ = probing_sequence_.rows();
      }
      break;
    }

    // f_att = generateNormalForce(normal_vector_.head<2>(), attraction_force_ / 5);
    // f_att = generateNormalForce(attraction_force_/3, contact_loss_threshold_, xd, T_WA_, normal_vector_, 50, time.toSec(), arm.task_start_time_.toSec());
    f_prob = generateProbingForce(origin_, current_, xd, T_EA_, dir, speed, time.toSec(), arm.task_start_time_.toSec());
    f_a = f_prob;
    f_star = T_WA_.linear() * f_a;
    m_star = PegInHole::keepCurrentOrientation(origin_, current_, xd, 1500, 10);

    // std::cout<<"---------------------"<<std::endl;
    // std::cout<<"normal vector: "<< updateNormalVector((PROBE_DIRECTION)probing_sequence_(dir_index_)).transpose()<<std::endl;
    // std::cout<<"abs_normal vector: "<< normal_vector.transpose()<<std::endl;
    // std::cout<<"f_att: "<<f_att.transpose()<<std::endl;
    // std::cout<<"f_prob: "<<f_prob.transpose()<<std::endl;
    // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
    // std::cout<<"selection matrix: \n"<< selection_matrix<<std::endl;
    break;

  case CONTACT_AGAIN:
    if (run_time > 0.25)
    {
      std::cout << "Try to probe deges again" << std::endl;
      state_ = PROBE_ON_THE_EDGE;
      is_mode_changed_ = true;
      init_force_ext_ = f_ext;
      break;
    }

    f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 1000, 15, 2000, 15).head<3>();
    m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 1000, 15, 2000, 15).tail<3>();
    
    break;

  case FAIL:
    std::cout << "FAIL TO DETECT OBJECT" << std::endl;
    setAborted();
    break;

  case COMPLETE:
    cross_point = getCrossPoint(probe_coeff_);    
    renew_origin = renewOrigin(probe_origin_, current_, T_EA_, cross_point, probing_sequence_, probe_dist_);
    
    renew_origin_.position.x = renew_origin(0);
    renew_origin_.position.y = renew_origin(1);
    renew_origin_.position.z = renew_origin(2);
    renew_origin_.orientation.x = probe_origin_quat_.x();
    renew_origin_.orientation.y = probe_origin_quat_.y();
    renew_origin_.orientation.z = probe_origin_quat_.z();
    renew_origin_.orientation.w = probe_origin_quat_.w();
    
    result_.renew_origin = renew_origin_;
    std::cout << "FINISHED PROBING EDGES" << std::endl;
    f_star = PegInHole::keepCurrentPose(origin_, current_, xd, 1000, 15, 2000, 15).head<3>();
    m_star = PegInHole::keepCurrentPose(origin_, current_, xd, 1000, 15, 2000, 15).tail<3>();
   
    setSucceeded();   
    break;
  }

  if (run_time > 100.0)
    setAborted();

  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;

  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;

  // if(state_ == COMPLETE)  f_star_zero_.setZero();
  // std::cout<<"f_measured: "<<f_measured_.transpose()<<std::endl;
  // std::cout<<"f_filtered: "<<f_ext.transpose()<<std::endl;

  // if(state_ == RECOVER_CONTACT) f_star_zero_.setZero();
  if (state_ == PROBE_ON_THE_EDGE || state_ == CONTACT_AGAIN)
  {
    // probe_pr_data << (current_ * T_EA_).translation().transpose() << std::endl;
    probe_pr_data << save_position.transpose() << std::endl;
    // probe_pr_data << arm.position_.transpose() << std::endl;
    probe_ft_cmd_data << (T_WA_.inverse() * f_star.head<3>()).transpose() << std::endl;
    // probe_ft_data << (T_WA_.inverse()*f_ext.head<3>()).transpose() << std::endl;
    // probe_ft_data << f.transpose()<<std::endl;
    probe_vel_data << task_vel_a.transpose() << std::endl;

    Eigen::Vector6d f;
    f << f_prob, f_att;
    probe_ft_data << f.transpose() << std::endl;
  }

  // probe_pr_data << (T_WA_.inverse()*current_ * T_EA_).translation().transpose() << std::endl;
  // if(dir_index_ == 1) f_star_zero_.setZero();

  Eigen::Matrix<double, 7, 1> desired_torque = jacobian.transpose() *arm.modified_lambda_matrix_*f_star_zero_;
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
  as_.setSucceeded(result_);
  control_running_ = false;
}

void AssembleProbeEdgeActionServer::setAborted()
{
  as_.setAborted(result_);
  control_running_ = false;
}

void AssembleProbeEdgeActionServer::generateProbingSequence(const Eigen::Vector2d &object_location)
{
  if (object_location(0) == 1)
    probing_sequence_ << (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT;
  else if (object_location(0) == -1)
    probing_sequence_ << (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT;
  else if (object_location(1) == 1)
    probing_sequence_ << (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN;
  else if (object_location(1) == -1)
    probing_sequence_ << (int)PROBE_DIRECTION::UP, (int)PROBE_DIRECTION::RIGHT, (int)PROBE_DIRECTION::DOWN, (int)PROBE_DIRECTION::LEFT, (int)PROBE_DIRECTION::UP;
}

Eigen::Vector2d AssembleProbeEdgeActionServer::updateNormalVector(PROBE_DIRECTION probing_direction)
{
  Eigen::Vector2d n;
  if (probing_direction == PROBE_DIRECTION::UP)
    n << 0, -1; //Up --> -y
  else if (probing_direction == PROBE_DIRECTION::RIGHT)
    n << -1, 0; //RIGHT --> -x
  else if (probing_direction == PROBE_DIRECTION::LEFT)
    n << 1, 0; //LEFT --> +x
  else if (probing_direction == PROBE_DIRECTION::DOWN)
    n << 0, 1; //DOWN --> +y
  return n;
}

Eigen::Vector2d AssembleProbeEdgeActionServer::updateTangentVector(PROBE_DIRECTION probing_direction)
{
  Eigen::Vector2d n;
  if (probing_direction == PROBE_DIRECTION::UP)         n << 1, 0; //Up --> -y    
  else if (probing_direction == PROBE_DIRECTION::RIGHT) n << 0, -1; //RIGHT --> -x    
  else if (probing_direction == PROBE_DIRECTION::LEFT)  n << 0, 1; //LEFT --> +x    
  else if (probing_direction == PROBE_DIRECTION::DOWN)  n << -1, 0; //DOWN --> +y
    
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

Eigen::Vector3d AssembleProbeEdgeActionServer::generateNormalForce(const double force,
                                                                   const double target_vel,
                                                                   const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                                   const Eigen::Isometry3d &T_wa,
                                                                   const Eigen::Vector3d &normal_vector,
                                                                   const double kp,
                                                                   const double t,
                                                                   const double t_0)
{
  Eigen::Vector3d f_a, v_cur_a;
  Eigen::Matrix3d Kp;
  double speed;
  double f_cmd;
  int dir;
  
  Kp = kp * Eigen::Matrix3d::Identity();
  v_cur_a = T_wa.linear().inverse() * xd.head<3>();

  f_cmd = dyros_math::cubic(t, t_0, t_0 + 0.5, 0.0, force, 0.0, 0.0);

  for (int i = 0; i < 3; i++) v_cur_a(i) = abs(normal_vector(i)) * v_cur_a(i);

  f_a = f_cmd * normal_vector + Kp * (target_vel * normal_vector - v_cur_a);
  
  // Eigen::IOFormat clean_fmt(5, 0, ", ", "\n", "[", "]");
  // std::cout<<"target velocity: "<<(target_vel * normal_vector).transpose().format(clean_fmt)<<std::endl;
  // std::cout<<"curr   velocity: "<<(v_cur_a).transpose().format(clean_fmt)<<std::endl;
  // std::cout<<"velocity error : "<<(target_vel * normal_vector - v_cur_a).transpose().format(clean_fmt)<<std::endl;
  // std::cout<<"f_cubic        : "<<f_cmd<<std::endl;
  return f_a; //w.r.t Assembly frame s.t., f_a
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

void AssembleProbeEdgeActionServer::saveProbingResult(std::vector<double> path_x,
                                                      std::vector<double> path_y,
                                                      const int nLoss,
                                                      Eigen::MatrixXd &probe_coeff,
                                                      Eigen::VectorXd &probe_dist)
{
  Eigen::Vector2d p, q, r;
  
  probe_coeff.row(nLoss - 1) = PegInHole::LeastSquareEdgeProbing(path_x, path_y);  
  
  p(0) = path_x.front();
  q(0) = path_x.back();
  
  // p(1) = probe_coeff(nLoss - 1, 0) * p(0) + probe_coeff(nLoss - 1, 1);
  // q(1) = probe_coeff(nLoss - 1, 0) * q(0) + probe_coeff(nLoss - 1, 1);
  p(1) = path_y.front();
  q(1) = path_y.back();
  
  r = q - p;
  
  probe_dist(nLoss - 1) = r.norm();

  // std::cout << "get least square coefficient: " << probe_coeff.row(nLoss - 1) << std::endl;
  // std::cout << "get probing distance: " << probe_dist(nLoss - 1) << std::endl;
}

Eigen::Vector3d AssembleProbeEdgeActionServer::getCrossPoint(const Eigen::MatrixXd &probe_coeff)
{
  Eigen::Vector3d cross_point;
  double a1, b1;
  double a2, b2;
  
  cross_point.setZero();  // w.r.t {A}

  a1 = probe_coeff(0, 0);
  b1 = probe_coeff(0, 1);  
  a2 = probe_coeff(1, 0);
  b2 = probe_coeff(1, 1);

  cross_point(0) = (b2 - b1) / (a1 - a2);
  cross_point(1) = (a1 * b2 - a2 * b1) / (a1 - a2);
  // cross_point(2) = origin.translation()(2);

  return cross_point;
}

Eigen::Vector3d AssembleProbeEdgeActionServer::renewOrigin(const Eigen::Isometry3d &origin,
                                                           const Eigen::Isometry3d &current,
                                                           const Eigen::Isometry3d &T_ea,
                                                           const Eigen::Vector3d &cross_point,
                                                           const Eigen::VectorXd &probing_sequence,
                                                           const Eigen::VectorXd &probing_dist)
{
  int first_dir;
  Eigen::Vector2d dist;
  Eigen::Vector3d renew_origin, param_vec;
  Eigen::Matrix3d dir_mat;
  Eigen::Isometry3d T_start, T_end, T_end_start, T_start_target, T_end_target;
  double w = 1.5;
  

  first_dir = probing_sequence(0);
  dir_mat = Eigen::Matrix3d::Identity();
  
  T_start = origin*T_ea;
  T_end = current*T_ea;

  if(first_dir == PROBE_DIRECTION::UP)
  {
    dir_mat(0,0) = -1; dir_mat(1,1) = -1;
    dist << probing_dist(0), probing_dist(1);
  } 
  else if(first_dir == PROBE_DIRECTION::DOWN)
  {
    dir_mat(0,0) =  1; dir_mat(1,1) =  1;
    dist << probing_dist(0), probing_dist(1);
  }
  else if(first_dir == PROBE_DIRECTION::LEFT)
  {
    dir_mat(0,0) =  1; dir_mat(1,1) = -1;
    dist << probing_dist(1), probing_dist(0);
  }
  else if(first_dir == PROBE_DIRECTION::RIGHT)
  {
    dir_mat(0,0) =  -1; dir_mat(1,1) = 1;
    dist << probing_dist(1), probing_dist(0); 
  } 
  dir_mat(2,2) = 0.0;

  if (dist(0) < height_ * w && dist(1) < height_ * w)
  {
    if (first_dir == PROBE_DIRECTION::UP || first_dir == PROBE_DIRECTION::DOWN)
      param_vec << width_ / 2 + pin_radius_ + offset_, height_ / 2 + pin_radius_ - offset_, 0;
    else if (first_dir == PROBE_DIRECTION::LEFT || first_dir == PROBE_DIRECTION::RIGHT)
      param_vec << height_ / 2 + pin_radius_ - offset_, width_ / 2 + pin_radius_ + offset_, 0;
  }
  else if (dist(0) > height_ * w)
    param_vec << width_ / 2 + pin_radius_ + offset_, height_ / 2 + pin_radius_ - offset_, 0;
  else if (dist(1) > height_ * w)
    param_vec << height_ / 2 + pin_radius_ - offset_, width_ / 2 + pin_radius_ + offset_, 0;

  renew_origin = cross_point + dir_mat*param_vec;                   // w.r.t probing start frame.
  
  // T_start_target.linear() = Eigen::Matrix3d::Identity(); //set the orientation
  T_start_target = Eigen::Matrix4d::Identity();
  T_start_target.translation() = renew_origin;

  T_end_start = T_end.inverse()*T_start;
  T_end_target = T_end_start*T_start_target;
  
  std::cout<<"first dir       : "<<(PROBE_DIRECTION) first_dir<<std::endl;
  std::cout<<"cross_point     : "<< cross_point.transpose()<<std::endl;
  std::cout<<"dir_mat         :\n"<< dir_mat<<std::endl;
  std::cout<<"dist            : "<< dist.transpose()<<std::endl;
  std::cout<<"param_vec       : "<< param_vec.transpose()<<std::endl;
  std::cout<<"renewed origin  : "<< renew_origin.transpose()<<std::endl;
  std::cout<<"T_start_target  : \n"<<T_start_target.matrix()<<std::endl;
  std::cout<<"T_end_start     : \n"<<T_end_start.matrix()<<std::endl;
  std::cout<<"T_end_target    : \n"<<T_end_target.matrix()<<std::endl;

  return T_end_target.translation(); // w.r.t probing end frame
}
 