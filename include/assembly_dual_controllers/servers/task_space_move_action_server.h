#pragma once
#define dof 7
#include <eigen_conversions/eigen_msg.h>

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_msgs/TaskSpaceMoveAction.h>

using namespace dyros_math;
class TaskSpaceMoveActionServer : public ActionServerBase
{
public:
  // Joint Trajectory Execution Server (Moveit Planning)
  actionlib::SimpleActionServer<assembly_msgs::TaskSpaceMoveAction> as_;

  assembly_msgs::TaskSpaceMoveFeedback feedback_;
  assembly_msgs::TaskSpaceMoveResult result_;
  assembly_msgs::TaskSpaceMoveGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

  bool control_running {false}; // multi-threading error prevention
  
  std::string active_arm_;

  Eigen::VectorXd q_desired_;
  Eigen::VectorXd qd_desired_;
  Eigen::VectorXd qdd_desired_;

  Eigen::Isometry3d target_pose_;

  TaskSpaceMoveActionServer(std::string name, ros::NodeHandle &nh, 
                          std::map<std::string, std::shared_ptr<FrankaModelUpdater> > &mu);
  // bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque) override; //command to robot
  bool compute(ros::Time time) override;
  bool computeArm(ros::Time time, FrankaModelUpdater &arm);


  struct subTaskState{
        Eigen::MatrixXd J, JT, J_bar, J_barT;
        Eigen::VectorXd f, fi;

        double h;

        double h1, h2;

        Eigen::MatrixXd lambda, lambda_inv, NT, N;
        Eigen::VectorXd T;

        void resize(int n, int dof_)
        {
            J.resize(n, dof_);
            JT.resize(dof_, n);

            NT.resize(dof_, dof_);
            N.resize(dof_, dof_);
            lambda.resize(n, n);
            lambda_inv.resize(n, n);

            f.resize(n);
            fi.resize(n);
        }

        void setActive(double low, double high, double current)
        {
            if(current > high) { h = 1.0; }
            else if (current < low) { h = 0.0; }
            else { h = (current - low) / (high - low) + 0.0; }
        }

        void setActive2(double low, double high, double current, bool j1)
        {
            const double buffer = 5.0 * M_PI / 180.0;

            if (j1){
            if(current > high) { h1 = 1.0; }
            else if (current > high - buffer &&
                     current  < high)
            { h1 = 0.5 + 0.5 * sin(M_PI / buffer * (current - high + buffer) - M_PI / 2); }
            else if (current > low + buffer &&
                     current < high - buffer)
            { h1 = 0.0; }
            else if (current > low &&
                     current < low + buffer)
            { h1 = 0.5 + 0.5 * sin(M_PI / buffer * (current - low + buffer) - M_PI / 2); }
            else { h1 = 1.0; }
            
            }
            else{

            if(current > high) { h2 = 1.0; }
            else if (current > high - buffer &&
                     current  < high)
            { h2 = 0.5 + 0.5 * sin(M_PI / buffer * (current - high + buffer) - M_PI / 2); }
            else if (current > low + buffer &&
                     current < high - buffer)
            { h2 = 0.0; }
            else if (current > low &&
                     current < low + buffer)
            { h2 = 0.5 + 0.5 * sin(M_PI / buffer * (current - low + buffer) - M_PI / 2); }
            else { h2 = 1.0; }
            }
        }

        void setActive3(double low, double high, double current)
        {
            double a,b;
            if (current < low)
            {
                h = 0.0;
            }
            else if (current > high)
            {
                h = 1.0;
            }
            else
            {
                a = -2.0 / pow(high - low, 3);
                b = 3.0 / pow(high - low, 2);
                h = a * pow(current - low, 3) + b * pow(current - low, 2);
            }
        }
        void setPotential_2(double low, double high, double current)
        {
            f(0) = 0.0;

            const double n = 100.0;
            const double offset = 5.0 * M_PI / 180.0;

            double low_buffer = low + offset;
            double high_buffer = high - offset;

            // Check whether there is overlap
            if (current < low_buffer)
            {
               f(0) = n * ( low_buffer - current );
            }
            else if (current > high_buffer)
            {
                f(0) = n * ( high_buffer - current );
            }
            else
            {
                f(0) = 0.0;
            }
        }
        void setPotential_3(double low, double high, double current_q, double current_qdot, bool j1)
        {
            f(0) = 0.0;
            f(1) = 0.0;

            const double kp = 700.0, kv = 2.0*sqrt(700.0);
            const double offset = 5.0 * M_PI / 180.0;

            double low_buffer = low + offset;
            double high_buffer = high - offset;

            if (j1)
            {
            // Chec\ whether there is overlap
            if (current_q < low_buffer)
            {
               f(0) = kp * ( low_buffer - current_q ) - kv * current_qdot;
            }
            else if (current_q > high_buffer)
            {
                f(0) = kp * ( high_buffer - current_q ) - kv *current_qdot;
            }
            else
            {
                f(0) = 0.0;
            }
            
            }
            else{
                            // Chec\ whether there is overlap
            if (current_q < low_buffer)
            {
               f(1) = kp * ( low_buffer - current_q ) - kv * current_qdot;
            }
            else if (current_q > high_buffer)
            {
                f(1) = kp * ( high_buffer - current_q ) - kv *current_qdot;
            }
            else
            {
                f(1) = 0.0;
            }
            }
        }
    } subTask_[2];

    // For convinient
#define _task1 subTask_[0]
#define _task2 subTask_[1]

protected:
  void setSucceeded() override {};
  void setAborted() override {};
};
