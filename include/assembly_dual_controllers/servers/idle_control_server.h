#pragma once

#include <ros/ros.h>

#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/model/franka_model_updater.h>
#include <assembly_msgs/IdleControl.h>

#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>

#include <Eigen/Dense>
#include <map>

using namespace dyros_math;

class IdleControlServer
{
  ros::NodeHandle & nh_; 
  std::map<std::string, std::shared_ptr<FrankaModelUpdater> > mu_;
  std::map<std::string, assembly_msgs::IdleControl::Request > params_;
  
  ros::ServiceServer server_ ;

public:
  IdleControlServer(const std::string &name, ros::NodeHandle &nh,
                  std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

  bool compute(ros::Time time);
  void computeArm(ros::Time time, FrankaModelUpdater &arm, assembly_msgs::IdleControl::Request &param);

private:
  bool setTarget(assembly_msgs::IdleControl::Request  &req,
                 assembly_msgs::IdleControl::Response &res);
};