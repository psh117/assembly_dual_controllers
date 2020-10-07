#pragma once

#include <actionlib/server/simple_action_server.h>
#include <assembly_dual_controllers/utils/model/franka_model_updater.h>
#include <assembly_dual_controllers/robot_model/franka_panda_model.h>
#include <Eigen/Dense>
#include <map>

class ActionServerBase
{
  protected:
    //actionlib::SimpleActionServer<sth> as_;
    
    //assembly_msgs::sthFeedback feedback_;
    //assembly_msgs::sthResult result_;
    //assembly_msgs::sthGoalConstPtr goal_;

    int feedback_header_stamp_ {0}; 

    std::string action_name_;

    ros::Time start_time_;
  
    ros::NodeHandle & nh_; 
    bool control_running_ {false}; // multi-threading error prevention
    
    // std::shared_ptr<std::map<std::string, FrankaDataContainer>> mu_;
    std::map<std::string, std::shared_ptr<FrankaModelUpdater> > mu_;
    // std::shared_ptr<FrankaModelUpdater> mu_r_;
    // std::shared_ptr<FrankaModelUpdater> mu_l_;
    
    FrankaPandaModel rbdl_panda_;
    
    ActionServerBase(std::string name, ros::NodeHandle &nh,
      std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu) :
  action_name_(name), nh_(nh), mu_(mu)  {}

    virtual void goalCallback() = 0;
    virtual void preemptCallback() = 0;

  public:
    virtual bool compute(ros::Time time) = 0;
    // virtual bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque_main, Eigen::Matrix<double, 7, 1> & torque_sub) = 0; //command to robot
    // virtual Eigen::Matrix<double, 6, 1> mainArm(ros::Time time);
    // virtual Eigen::Matrix<double, 6, 1> subArm(ros::Time time);
    // virtual void taskArmDefine(const std::string left_arm_id, const std::string right_arm_id, std::string main_arm_id, std::string sub_arm_id);
    // virtual void parametersAllocator();
    // virtual void readState();
    // virtual void initParameters();
};
