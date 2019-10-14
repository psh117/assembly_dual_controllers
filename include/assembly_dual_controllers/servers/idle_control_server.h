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

  // struct IdleControlSettings
  // {
  //   double kp;
  //   double kd;
  // };

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
      if (arm.target_updated_)
      {
        arm.idle_controlled_ = false;
      }
      else
      {
        if (arm.idle_controlled_ == false)
        {
          // initialize
          arm.setInitialTransform();
        }
        computeArm(time, arm, params_[pair.first]);
      }
    }
  }
  
  void computeArm(ros::Time time, FrankaModelUpdater &arm, assembly_msgs::IdleControl::Request &param)
  {
    switch (param.mode)
    {
      case assembly_msgs::IdleControl::Request::DISABLED:
      {
        arm.setTorque(Eigen::Vector7d::Zero(), true);
      }
      case assembly_msgs::IdleControl::Request::JOINT_SPACE:
      {
        
        // arm.setTorque(~~~, true); // <<- idle_control = true
      }
      case assembly_msgs::IdleControl::Request::TASK_SPACE:
      {
        // default gain (gains are not set)
        if (param.p_gain <= 0.01)
        {
          param.p_gain = 5000.;
        } 
        if (param.d_gain <= 0.01)
        {
          param.d_gain = 100.;
        }
        
        auto fstar = PegInHole::keepCurrentStateGain(
          arm.initial_transform_.translation(), arm.initial_transform_.linear(), 
          arm.rotation_, arm.position_, arm.xd_, param.p_gain, param.d_gain);
      
        arm.setTorque(arm.jacobian_.transpose() * fstar, true);
        arm.idle_controlled_ = true;
      }
      default:
        arm.setTorque(Eigen::Vector7d::Zero(), true);
    }
  }

private:
  bool setTarget(assembly_msgs::IdleControl::Request  &req,
                 assembly_msgs::IdleControl::Response &res)
  {
    return true;
  }

};