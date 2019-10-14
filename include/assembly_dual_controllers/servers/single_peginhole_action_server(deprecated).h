#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <cstdlib>
#include <ctime>

#include <assembly_dual_controllers/servers/action_server_base.h>
#include <assembly_msgs/SinglePegInHoleAction.h>
#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <assembly_dual_controllers/utils/control/criteria.h>

using namespace PegInHole;
using namespace Criteria;

static constexpr uint32_t HashCode(const char *p){

    return *p ? static_cast<uint32_t>(*p) + 33 * HashCode(p + 1) :  5381;
}

class SinglePegInHoleActionServer : public ActionServerBase
{
        actionlib::SimpleActionServer<assembly_msgs::SinglePegInHoleAction> as_;

        assembly_msgs::SinglePegInHoleFeedback feedback_;
        assembly_msgs::SinglePegInHoleResult result_;
        assembly_msgs::SinglePegInHoleGoalConstPtr goal_;
        
        void goalCallback() override;
        void preemptCallback() override;
        void initParameters() override;

        Eigen::Matrix<double, 6, 1> mainArm(ros::Time time) override;
        Eigen::Matrix<double, 6, 1> subArm(ros::Time time) override;
    
    public:
    
        SinglePegInHoleActionServer(std::string name, ros::NodeHandle &nh,
            std::shared_ptr<FrankaModelUpdater> &mu_r, std::shared_ptr<FrankaModelUpdater> &mu_l);

        void approach(const Eigen::Vector3d pos_ref, const Eigen::Matrix3d rot_ref, const Eigen::Matrix<double, 6, 1> vel_ref, const double f_ref, ros::Time time);
        void spiral(const Eigen::Vector3d pos_ref, const Eigen::Matrix3d rot_ref, const Eigen::Matrix<double, 6, 1> vel_ref, const double f_ref, ros::Time time);
        void insertion(const Eigen::Vector3d pos_ref, const Eigen::Matrix3d rot_ref, ros::Time time);
        void inspection(const Eigen::Vector3d pos_ref, const Eigen::Matrix3d rot_ref, const Eigen::Matrix<double, 6, 1> vel_ref);
        void retreat(const Eigen::Vector3d pos_ref, const Eigen::Matrix3d rot_ref, const Eigen::Matrix<double, 6, 1> vel_ref, ros::Time time);
        void crossSearch(const Eigen::Vector3d pos_ref, const Eigen::Matrix3d rot_ref, const Eigen::Matrix<double, 6, 1> vel_ref, ros::Time time);

        void hold(const Eigen::Vector3d pos_ref, const Eigen::Matrix3d rot_ref, const Eigen::Matrix<double, 6, 1> vel_ref);

        void setForStage(const Eigen::Vector3d pos_ref, const Eigen::Matrix3d rot_ref);
        void completeStage();

        void parametersAllocator() override;
        void taskArmDefine(const std::string left_arm_id, const std::string right_arm_id, std::string main_arm_id, std::string sub_arm_id) override;
        void readState() override;
        bool getTarget(ros::Time time, Eigen::Matrix<double, 7, 1> & torque_main, Eigen::Matrix<double, 7, 1> & torque_sub) override;

        
    private:

        Eigen::Vector3d pos_r_; //current position of right arm
        Eigen::Vector3d pos_l_; //current position of left arm

        Eigen::Matrix<double, 6, 1> vel_r_; //linear + angular
        Eigen::Matrix<double, 6, 1> vel_l_;

        Eigen::Matrix<double, 6, 1> f_sensing_r_;
        Eigen::Matrix<double, 6, 1> f_sensing_l_;
        
        Eigen::Matrix<double, 6, 7> J_r_;
        Eigen::Matrix<double, 6, 7> J_l_;
        Eigen::Matrix<double, 6, 7> J_main_;
        Eigen::Matrix<double, 6, 7> J_sub_;        
        Eigen::Matrix<double, 7, 6> J_bar_r_;
        Eigen::Matrix<double, 7, 6> J_bar_l_;        
        Eigen::Matrix<double, 7, 6> J_bar_main_;
        Eigen::Matrix<double, 7, 6> J_bar_sub_;

        Eigen::Matrix<double, 3, 3> rot_r_;
        Eigen::Matrix<double, 3, 3> rot_l_;
        Eigen::Matrix<double, 3, 3> init_rot_;                

        Eigen::Vector3d f_star_;
        Eigen::Vector3d m_star_;
        Eigen::Matrix<double, 6, 1>f_star_zero_;

        Eigen::Matrix<double, 6, 1> f_star_zero_r_;
        Eigen::Matrix<double, 6, 1> f_star_zero_l_;

        Eigen::Vector3d origin_;
        
        std::string task_type_;
        std::string left_arm_id_;
        std::string right_arm_id_;
        
        Eigen::Vector3d pos_ref_main_;
        Eigen::Vector3d pos_ref_sub_;
        Eigen::Matrix3d rot_ref_main_;
        Eigen::Matrix3d rot_ref_sub_;
        Eigen::Matrix<double, 6, 1> vel_ref_main_;
        Eigen::Matrix<double, 6, 1> vel_ref_sub_;
        Eigen::Matrix<double, 6, 1> f_ref_main_;
        Eigen::Matrix<double, 6, 1> f_ref_sub_;
        
        bool is_first_;
        
        double spiral_duration_;
            
        double f_xy_r_;
        double f_xy_l_;
        double f_xy_main_;
        double f_xy_sub_;

        int stage_;

        // std::shared_ptr<std::map<std::string, FrankaDataContainer>> mu_;
        
};



			  

			  

			  
