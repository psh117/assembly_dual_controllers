#include <assembly_dual_controllers/servers/joint_trajectory_action_server.h>

JointTrajectoryActionServer::JointTrajectoryActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
  as_.registerGoalCallback(boost::bind(&JointTrajectoryActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&JointTrajectoryActionServer::preemptCallback, this));
  as_.start();

  std::string joint_name_prefix = "_joint";
  for (auto iter = mu_.begin(); iter != mu_.end(); iter++)
  {
    std::vector<std::string> joint_names;
    for(int i=1; i<=7; i++)
    {
      std::string joint_name = iter->first + joint_name_prefix + std::to_string(i);
      std::cout << joint_name;
      joint_names.push_back(joint_name);
    }
    joint_names_[iter->first] = joint_names;
        
    Eigen::VectorXd kp(7), kv(7);
    kp << 800, 800, 800, 800, 500, 400, 300;
    kv << 10, 10, 10, 10, 5, 5, 3;
    arm_gain_map_[iter->first] = std::make_pair(kp,kv);
  }
  server_  = nh_.advertiseService(name + "_gain_set", &JointTrajectoryActionServer::setTarget, this);
}

void JointTrajectoryActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if(goal_->trajectory.joint_names.size() == 0)
  {
    ROS_WARN("[JointTrajectoryActionServer::goalCallback] Joint trajectory goal but no data has been received. Just passing it.");
    as_.setAborted();
    control_running_ = false;
    return; 
  }
  ROS_INFO("[JointTrajectoryActionServer::goalCallback] Joint trajectory goal has been received.");

  start_time_ = ros::Time::now();
  auto now_time = ros::WallTime::now();
  ROS_INFO("start time = %lf %lf", start_time_.toSec(), now_time.toSec());
  writeDebugInfos("goalCallback", "goall received");
  DEBUG_FILE("start_time_: " << start_time_.toSec());
  // start_time_ = ros::Time::now();

  bool find_any_arm = false;
  
  for (auto iter = mu_.begin(); iter != mu_.end(); iter++)
  {
    active_arms_[iter->first] = false;
    bool find_arm = false;
    find_any_arm = true;
    // Find 'panda_left' in 'panda_left_joint1' 
    // ROS_INFO("%s ",goal_->trajectory.joint_names[0].c_str());
    // ROS_INFO("%s ",iter->first.c_str());
    // ROS_INFO("%d ", goal_->trajectory.joint_names[0].find(iter->first));
    for (int i=0; i<goal_->trajectory.joint_names.size(); i++)
    {
      if (goal_->trajectory.joint_names[i] == *joint_names_[iter->first].begin())
      {
        start_index_map_[iter->first] = i;
        find_arm = true;
        break;
      }
    }
    if(find_arm == true)
    {
      active_arms_[iter->first] = true;
      traj_running_ = true;
    }
  }

  q_desired_.resize(goal_->trajectory.joint_names.size());
  for(int i=0; i<q_desired_.size(); i++)
  {
    q_desired_(i) = goal_->trajectory.points[0].positions[i];
  }
  DEBUG_FILE("q_desired_: " << q_desired_.transpose());

  qd_desired_.setZero(goal_->trajectory.joint_names.size());
  qdd_desired_.setZero(goal_->trajectory.joint_names.size());
  // trajectory_data_.resize(goal_->trajectory.joint_names.size(), goal_->trajectory.points.size());

  if (find_any_arm == false)
  {
    ROS_ERROR("[JointTrajectoryActionServer::goalCallback] the name %s is not in the arm list.", goal_->trajectory.joint_names[0].c_str());
    as_.setAborted();
    control_running_ = false;
    return;
  }

  for (int i=0; i<goal_->trajectory.joint_names.size(); i++)
  {
    for (auto iter = mu_.begin(); iter != mu_.end(); iter++)
    {
      bool is_found = false;
      for (int j=0; j<joint_names_[iter->first].size(); j++)
      {
        if ( goal_->trajectory.joint_names[i] == joint_names_[iter->first].at(j) )
        {
          is_found = true;
          joint_map_[i] = std::make_pair(iter->first, j);       
          break;
        }
      }
      if (is_found)
        break;
    }
  }

  auto as_joint_size = goal_->trajectory.points[0].positions.size();

  feedback_.joint_names.resize(as_joint_size);
  feedback_.actual.positions.resize(as_joint_size);
  feedback_.actual.velocities.resize(as_joint_size);
  feedback_.actual.accelerations.resize(as_joint_size);

  control_running_ = true;
}
void JointTrajectoryActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}


bool JointTrajectoryActionServer::compute(ros::Time time)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if(!traj_running_)  // wait for acceptance of the goal, active but not accepted
    return false; 
  
  for (auto & arm : active_arms_)
  {
    if (arm.second == true)
    {
      computeArm(time, *mu_[arm.first], arm.first);
    }
  }
  return false;
}


bool JointTrajectoryActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm, const std::string & arm_name)
{
  int start_index = start_index_map_[arm_name];
  Eigen::Matrix<double, 7, 1> q_desired, qd_desired, qdd_desired, tau_cmd;
  auto total_time = goal_->trajectory.points.back().time_from_start;
  ros::Duration passed_time = time - start_time_;

  Eigen::Vector3d position_now;

  bool value_updated = false;
  for (int j = start_index; j < start_index+7; j++){ 
    for (int i = 0; i < goal_->trajectory.points.size()-1; i++)
    {
      auto & cur_point = goal_->trajectory.points[i];
      auto & next_point = goal_->trajectory.points[i+1];
      if ((passed_time >= cur_point.time_from_start) && (passed_time <= next_point.time_from_start))
      {
        position_now = dyros_math::quinticSpline(passed_time.toSec(), cur_point.time_from_start.toSec(), next_point.time_from_start.toSec(),  
        cur_point.positions[j], cur_point.velocities[j], cur_point.accelerations[j], 
        next_point.positions[j], next_point.velocities[j], next_point.accelerations[j]);

        value_updated = true;
        break;
      }
    }

    if (value_updated == false)
    {
      position_now(0) = goal_->trajectory.points.back().positions[j];
      position_now(1) = goal_->trajectory.points.back().velocities[j];
      position_now(2) = goal_->trajectory.points.back().accelerations[j];
    }
    
    q_desired(j-start_index) = position_now(0);
    qd_desired(j-start_index) = position_now(1);
    qdd_desired(j-start_index) = position_now(2);

    feedback_.actual.positions[j] = position_now(0);
    feedback_.actual.velocities[j] = position_now(1);
    feedback_.actual.accelerations[j] = position_now(2);
  }

  Eigen::Matrix<double, 7,7> kp, kv;
  // Eigen::Matrix<double, 7, 1> kp_d, kd_d;
  // kp_d << 800, 800, 800, 800, 500, 400, 300;
  // kd_d << 10, 10, 10, 10, 5, 5, 3;
  
  kp = arm_gain_map_[arm_name].first.asDiagonal();
  kv = arm_gain_map_[arm_name].second.asDiagonal();
  // kp = Eigen::Matrix<double, 7,7>::Identity() * 800;
  // kv = Eigen::Matrix<double, 7,7>::Identity() * 10;

  // kp(6,6) = 300;
  // kv(6,6) = 5.0;

  Eigen::Matrix<double,7,1> desired_torque;
  
  Eigen::Matrix<double, 7,7> kpp = Eigen::Matrix<double, 7,7>::Identity() * 0.2;
  kpp(6,6) = 0.05;
  desired_torque = kpp * qdd_desired + (kp*(q_desired - arm.q_) + kv*(qd_desired - arm.qd_)) + arm.coriolis_;
  

  feedback_.actual.time_from_start = passed_time;
  feedback_.header.seq=feedback_header_stamp_;
  feedback_header_stamp_++;
  
  // std::cout << "s_idx : "  << start_index << std::endl;
  // std::cout << "q : " << arm.q_.transpose() << std::endl;
  // std::cout << "qd: " << q_desired.transpose() << std::endl;
  // std::cout << "dt: " << desired_torque.transpose() << std::endl;
  
  // as_.publishFeedback(feedback_);
  if (++ print_count_ > iter_per_print_)
  {
    Eigen::IOFormat tab_format(Eigen::FullPrecision, 0, "\t", "\n");
    debug_file_.precision(std::numeric_limits< double >::digits10);
    DEBUG_FILE(arm.arm_name_ << '\t' 
            << time.toSec() << '\t' 
            << arm.q_.transpose().format(tab_format) << '\t'
            << arm.qd_.transpose().format(tab_format) << '\t' 
            << arm.tau_ext_filtered_.transpose().format(tab_format) << '\t' 
            << arm.mob_torque_.transpose().format(tab_format) << '\t' 
            << q_desired.transpose().format(tab_format) << '\t' 
            << qd_desired.transpose().format(tab_format) << '\t' 
            << qdd_desired.transpose().format(tab_format));
    print_count_ = 0;
  }
  if(time.toSec() > (start_time_.toSec() +  total_time.toSec() + 0.5))
  {
    Eigen::Vector7d goal_q;
    goal_q = Eigen::Vector7d::Map(goal_->trajectory.points.back().positions.data() + start_index);
    std::cout << "goal_q: " << goal_q.transpose() << std::endl;
    std::cout << "joint_trajectory_done " << start_time_.toSec() << " to " << total_time.toSec() << std::endl;
    double diff = (goal_q-arm.q_).norm();
    if ( diff > 0.1 )
    {
      std::cout << "joint diff to goal is too high. assuming aborted" << std::endl;
      std::cout << "diff: " << diff << std::endl;
      std::cout << "goal_q: " << goal_q.transpose() << std::endl;
      std::cout << "arm.q_: " << arm.q_.transpose() << std::endl;
      as_.setAborted();
      control_running_ = false;
    }
    if (as_.isActive())
    {
      as_.setSucceeded();
      control_running_ = false;
    }
    traj_running_ = false;
    arm.setInitialValues(goal_q);
    arm.idle_controlled_ = true;
    arm.setTorque(desired_torque, true);
    return true;
  }

  arm.setTorque(desired_torque);
  return true;
}


bool JointTrajectoryActionServer::setTarget(assembly_msgs::SetTrajectoryFollowerGain::Request  &req,
                                            assembly_msgs::SetTrajectoryFollowerGain::Response &res)
{

  auto it = arm_gain_map_.find(req.arm_name);
  if (it == arm_gain_map_.end())
  {
    ROS_WARN("arm name %s is not in the arm_gain_map_. ignore", req.arm_name.c_str());
    res.is_succeed = false;
    return true;
  }

  if (req.p_gain.size() != 7 || req.d_gain.size() != 7)
  {
    ROS_INFO("req.p_gain != 7, resetting the gains");

    Eigen::VectorXd kp(7), kv(7);
    kp << 800, 800, 800, 800, 500, 400, 300;
    kv << 10, 10, 10, 10, 5, 5, 3;
    arm_gain_map_[req.arm_name] = std::make_pair(kp,kv);
  }
  else
  {
    arm_gain_map_[req.arm_name].first = Eigen::VectorXd::Map(req.p_gain.data(),7);
    arm_gain_map_[req.arm_name].second = Eigen::VectorXd::Map(req.d_gain.data(),7);
  }
  res.is_succeed = true;
  return true;
}
