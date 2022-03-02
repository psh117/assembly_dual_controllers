#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <assembly_dual_controllers/utils/dyros_math.h>

using namespace dyros_math;

namespace Estimator
{
    enum CONTACT_STATE : int
    {
        READY_CS = 0,
        SEARCH = 1,
        CROSSING = 2,
        TRANSITION = 3,
        INSERTION = 4,
        DEVIATION = 5
    };

    enum TORQUE_LABEL : int
    {    
        READY_TORQUE = 0, 
        SMALL = 1,        
        LARGE = 2
    };
    
    enum POS_VEL_LABEL : int
    {        
        READY_POS_VEL = 0,
        SURFACE = 1,
        SHALLOW = 2,
        FLOATING = 3,
        DEEP = 4
    };

    enum TORQUE_LABEL_TRIPLE : int
    {    
        READY_TORQUE_T = 0,
        SMALL_T = 1,        
        MEDIUM_T = 2,
        LARGE_T = 3
    };
    
    enum POS_VEL_LABEL_TRIPLE : int
    {        
        READY_PV_T=0,
        SURFACE_T = 1,
        SHALLOW_T = 2,
        FLOATING_T = 3,
        MODERATE_T = 4,
        DEEP_T = 5
    };

    struct GMM_model
    {
        Eigen::VectorXd mu;
        Eigen::MatrixXd Sigma;        
        double pi;
    };

    int countCurrentSpiralLaps(const double pitch,
                               const double lin_vel,
                               const double t);

    double getGaussianDistribution(const Eigen::VectorXd &x,
                                   const Eigen::VectorXd &mu,
                                   const Eigen::MatrixXd &Sigma);

    Estimator::TORQUE_LABEL torqueEstimator(const Eigen::VectorXd &torque_data,
                                            const struct GMM_model model_1,
                                            const struct GMM_model model_2);

    Estimator::POS_VEL_LABEL positionEstimator(const Eigen::VectorXd &position_data,
                                               const struct GMM_model model_surface,
                                               const struct GMM_model model_shallow,
                                               const struct GMM_model model_floating,
                                               const struct GMM_model model_deep);

    Estimator::CONTACT_STATE contactStateTable(const Estimator::TORQUE_LABEL torque_label,
                                               const Estimator::POS_VEL_LABEL position_label);


    // For triple arm case
    Estimator::TORQUE_LABEL_TRIPLE torqueEstimatorTriple(const Eigen::VectorXd &torque_data,
                                                  const struct GMM_model model_1,
                                                  const struct GMM_model model_2,
                                                  const struct GMM_model model_3);

    Estimator::POS_VEL_LABEL_TRIPLE positionEstimatorTriple(const Eigen::VectorXd &position_data,
                                                     const struct GMM_model model_surface,
                                                     const struct GMM_model model_shallow,
                                                     const struct GMM_model model_floating,
                                                     const struct GMM_model model_moderate,
                                                     const struct GMM_model model_deep);

    Estimator::CONTACT_STATE contactStateTableTriple(const Estimator::TORQUE_LABEL_TRIPLE torque_label,
                                                     const Estimator::POS_VEL_LABEL_TRIPLE position_label);

    class MultiSampling
    {
    public:
        MultiSampling()
        {
            count_insertion_ = 0;
            count_deviation_ = 0;
            deviation_ratio_ = 0;
            multi_sampling_flag_ = false;
            multi_sampling_start_theta_ = 0.0; // spiral angle  
            multi_sampling_start_time_ = 0.0;         
        };
        ~MultiSampling(){};

        int count_insertion_;
        int count_deviation_;
        double deviation_ratio_;
        bool multi_sampling_flag_;
        double multi_sampling_start_theta_; // spiral angle
        double multi_sampling_start_time_;
        
    };

}; // namespace Estimator