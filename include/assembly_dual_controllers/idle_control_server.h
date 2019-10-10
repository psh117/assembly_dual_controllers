#pragma once

#include <ros/ros.h>

#include <assembly_dual_controllers/dyros_math.h>
#include <assembly_dual_controllers/franka_model_updater.h>
#include <assembly_msgs/IdleControl.h>

#include <Eigen/Dense>
#include <map>

using namespace dyros_math;

class IdleControlServer
{
  ros::NodeHandle & nh_; 
  std::map<std::string, std::shared_ptr<FrankaModelUpdater> > mu_;
  ros::ServiceServer server_ ;

public:
  IdleControlServer(const std::string &name, ros::NodeHandle &nh,
                  std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu) :
  nh_(nh), mu_(mu)
  {
    server_  = nh_.advertiseService(name, &IdleControlServer::setTarget, this);
  }

  bool compute(ros::Time time)
  {
    for (auto & pair : mu_)
    {
      auto & arm = *(pair.second);
      if (!arm.target_updated_)
      {
        
      }
    }
  }

private:
  bool setTarget(assembly_msgs::IdleControl::Request  &req,
                 assembly_msgs::IdleControl::Response &res)
  {
    return true;
  }

};