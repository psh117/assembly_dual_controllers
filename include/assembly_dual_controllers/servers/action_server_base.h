#pragma once

#include <actionlib/server/simple_action_server.h>
#include <assembly_dual_controllers/utils/model/franka_model_updater.h>
#include <assembly_dual_controllers/robot_model/franka_panda_model.h>
#include <Eigen/Dense>
#include <map>


#define DEBUG_FILE(text) \
if(debug_file_.is_open()) \
{ \
  debug_file_ << text << std::endl;\
}

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
    
    std::ofstream debug_file_;
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
    virtual void signalAbort(bool is_aborted)
    {
      setAborted();
    }

    virtual void openDebugFile(const std::string& prefix)
    {
      const auto now = std::chrono::system_clock::now();
      const auto now_time_t = std::chrono::system_clock::to_time_t(now);
      const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
      std::stringstream ss;
      ss << prefix << action_name_ << '_'
         << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %a %T")
         << '.' << std::setfill('0') << std::setw(3) << now_ms.count() << ".txt";
        // std::cout << "[sss]: " << ss.str() << std::endl;
      debug_file_.open(ss.str());
    }

  protected:
    int iter_per_print_ {10};
    int print_count_ {0};
    
    void writeDebugInfos(const std::string &title, const std::string &context)
    {
      const auto now = std::chrono::system_clock::now();
      const auto now_time_t = std::chrono::system_clock::to_time_t(now);
      const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now.time_since_epoch()) % 1000;
      debug_file_ << "["
          << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %a %T")
          << '.' << std::setfill('0') << std::setw(3) << now_ms.count() << "]-";
      
      debug_file_ << "[" << title << "]: " << context << std::endl;
      debug_file_.precision(4);
      debug_file_ << std::setfill(' ');
    }
    virtual void setSucceeded() {};
    virtual void setAborted() {};
    // virtual bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque_main, Eigen::Matrix<double, 7, 1> & torque_sub) = 0; //command to robot
    // virtual Eigen::Matrix<double, 6, 1> mainArm(ros::Time time);
    // virtual Eigen::Matrix<double, 6, 1> subArm(ros::Time time);
    // virtual void taskArmDefine(const std::string left_arm_id, const std::string right_arm_id, std::string main_arm_id, std::string sub_arm_id);
    // virtual void parametersAllocator();
    // virtual void readState();
    // virtual void initParameters();
};
