#include <assembly_controllers/joint_trajectory_action_server.h>

JointTrajectoryActionServer::JointTrajectoryActionServer(std::string name, ros::NodeHandle &nh, 
                                std::shared_ptr<FrankaModelUpdater> &mu)
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

  ROS_INFO("Joint trajectory goal has been received.");

  start_time_ = ros::Time::now();

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


bool JointTrajectoryActionServer::getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque)
{
  if (!as_.isActive())
    return false;

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
  
  kp = Eigen::Matrix<double, 7,7>::Identity() * 800;
  kv = Eigen::Matrix<double, 7,7>::Identity() * 15;
  kp(6,6) = 600;
  kv(6,6) = 5;
  torque = (kp*(q_desired - mu_->q_) + kv*(qd_desired - mu_->qd_)) + mu_->coriolis_;

  feedback_.actual.time_from_start = passed_time;
  feedback_.header.seq=feedback_header_stamp_;
  feedback_header_stamp_++;
  as_.publishFeedback(feedback_);

  if(time.toSec() > (start_time_.toSec() +  total_time.toSec() + 0.5))
  {
    as_.setSucceeded();
  }

  return true;
}