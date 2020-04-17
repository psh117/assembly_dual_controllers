#include <assembly_dual_controllers/servers/joint_trajectory_action_server.h>

JointTrajectoryActionServer::JointTrajectoryActionServer(std::string name, ros::NodeHandle &nh, 
                                std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu)
: ActionServerBase(name,nh,mu),
as_(nh,name,false)
{
  as_.registerGoalCallback(boost::bind(&JointTrajectoryActionServer::goalCallback, this));
  as_.registerPreemptCallback(boost::bind(&JointTrajectoryActionServer::preemptCallback, this));
  as_.start();
}

void JointTrajectoryActionServer::goalCallback()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  if(goal_->trajectory.joint_names.size() == 0)
  {
    ROS_WARN("[JointTrajectoryActionServer::goalCallback] Joint trajectory goal but no data has been received. Just passing it.");
    as_.setAborted();
    return; 
  }
  ROS_INFO("[JointTrajectoryActionServer::goalCallback] Joint trajectory goal has been received.");

  start_time_ = ros::Time::now();
  auto now_time = ros::WallTime::now();
  ROS_INFO("start time = %lf %lf", start_time_.toSec(), now_time.toSec());
  // start_time_ = ros::Time::now();

  bool find_arm = false;
  for (auto iter = mu_.begin(); iter != mu_.end(); iter++)
  {
    // Find 'panda_left' in 'panda_left_joint1' 
    // ROS_INFO("%s ",goal_->trajectory.joint_names[0].c_str());
    // ROS_INFO("%s ",iter->first.c_str());
    // ROS_INFO("%d ", goal_->trajectory.joint_names[0].find(iter->first));
    if (goal_->trajectory.joint_names[0].find(iter->first) != std::string::npos)
    {
      ROS_INFO("[JointTrajectoryActionServer::goalCallback] target arm: %s.", iter->first.c_str());
      active_arm_ = iter->first;
      traj_running_ = true;
      find_arm = true;
    }
  }

  if (find_arm == false)
  {
    ROS_ERROR("[JointTrajectoryActionServer::goalCallback] the name %s is not in the arm list.", goal_->trajectory.joint_names[0].c_str());
    as_.setAborted();
    return;
  }

  auto as_joint_size = goal_->trajectory.points[0].positions.size();

  feedback_.joint_names.resize(as_joint_size);
  feedback_.actual.positions.resize(as_joint_size);
  feedback_.actual.velocities.resize(as_joint_size);
  feedback_.actual.accelerations.resize(as_joint_size);
}
void JointTrajectoryActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
}


bool JointTrajectoryActionServer::compute(ros::Time time)
{
  if (!as_.isActive())
      return false; 
  
  if(!traj_running_)  // wait for acceptance of the goal, active but not accepted
    return false; 
  
  computeArm(time, *mu_[active_arm_]);
  return false;
}


bool JointTrajectoryActionServer::computeArm(ros::Time time, FrankaModelUpdater &arm)
{
  Eigen::Matrix<double, 7, 1> q_desired, qd_desired, qdd_desired, tau_cmd;
  auto total_time = goal_->trajectory.points.back().time_from_start;
  ros::Duration passed_time = time - start_time_;

  Eigen::Vector3d position_now;

  bool value_updated = false;
  for (int j = 0; j < 7; j++){ 
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
    
    q_desired(j) = position_now(0);
    qd_desired(j) = position_now(1);
    qdd_desired(j) = position_now(2);

    feedback_.actual.positions[j] = position_now(0);
    feedback_.actual.velocities[j] = position_now(1);
    feedback_.actual.accelerations[j] = position_now(2);
  }

  Eigen::Matrix<double, 7,7> kp, kv;
  
  kp = Eigen::Matrix<double, 7,7>::Identity() * 600;
  kv = Eigen::Matrix<double, 7,7>::Identity() * 10;

  kp(6,6) = 200;
  kv(6,6) = 4.0;

  Eigen::Matrix<double,7,1> desired_torque;
  
  Eigen::Matrix<double, 7,7> kpp = Eigen::Matrix<double, 7,7>::Identity() * 0.2;
  kpp(6,6) = 0.05;
  desired_torque = kpp * qdd_desired + (kp*(q_desired - arm.q_) + kv*(qd_desired - arm.qd_)) + arm.coriolis_;
  

  feedback_.actual.time_from_start = passed_time;
  feedback_.header.seq=feedback_header_stamp_;
  feedback_header_stamp_++;
  
  // std::cout << "q : " << arm.q_.transpose() << std::endl;
  // std::cout << "qd: " << q_desired.transpose() << std::endl;
  // std::cout << "dt: " << desired_torque.transpose() << std::endl;
  
  arm.setTorque(desired_torque);
  // as_.publishFeedback(feedback_);

  if(time.toSec() > (start_time_.toSec() +  total_time.toSec() + 0.5))
  {
    ROS_INFO("joint_trajectory_done %lf %lf", start_time_.toSec(), total_time.toSec() );
    as_.setSucceeded();
    traj_running_ = false;
  }
  return true;
}
