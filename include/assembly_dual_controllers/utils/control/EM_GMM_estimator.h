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
        SEARCH = 1,
        CROSSING = 2,
        TRANSITION = 3,
        INSERTION = 4,
        DEVIATION = 5
    };

    enum LABEL : int
    {
        READY_FOR_ESTIMATION = 0,
        ZERO = 1,
        SMALL = 2,
        MEDIUM = 3,
        LARGE = 4
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

    Estimator::LABEL torqueEstimator(const Eigen::VectorXd &torque_data,
                                     const struct GMM_model model_1,
                                     const struct GMM_model model_2);

    Estimator::LABEL positionEstimator(const Eigen::VectorXd &position_data,
                                       const struct GMM_model model_zero,             
                                       const struct GMM_model model_small,
                                       const struct GMM_model model_medium,
                                       const struct GMM_model model_large);

    Estimator::CONTACT_STATE contactStateTable(const Estimator::LABEL torque_label,
                                               const Estimator::LABEL position_label);
}; // namespace Estimator