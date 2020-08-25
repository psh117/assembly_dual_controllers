#pragma once

#include <ros/ros.h>

#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/model/franka_model_updater.h>
#include <assembly_msgs/GraspUpdate.h>

#include <Eigen/Dense>
#include <map>

using namespace dyros_math;

class GraspUpdateServer
{
  ros::NodeHandle & nh_; 
  std::map<std::string, std::shared_ptr<FrankaModelUpdater> > mu_;
  
  ros::ServiceServer server_ ;

public:
  GraspUpdateServer(const std::string &name, ros::NodeHandle &nh,
                  std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);

private:
  bool updater(assembly_msgs::IdleControl::Request  &req,
                 assembly_msgs::IdleControl::Response &res);

};