#include <assembly_dual_controllers/utils/control/EM_GMM_estimator.h>

int Estimator::countCurrentSpiralLaps(const double pitch,
                                      const double lin_vel,
                                      const double t)
{
    
    int n_laps;
    double theta;
    
    theta = sqrt(4*lin_vel*M_PI*t/pitch);       
    n_laps = theta/(2*M_PI) + 1;
    
    return n_laps;
}

double Estimator::getGaussianDistribution(const Eigen::VectorXd &x,
                                          const Eigen::VectorXd &mu, 
                                          const Eigen::MatrixXd &Sigma)
{
    int row, col;
    double a, p;
    Eigen::MatrixXd b;
    Eigen::VectorXd dX;

    row = x.rows();
    col = x.cols();

    dX.resize(row);
    dX = x - mu;

    a = pow(2*M_PI, row/2)*sqrt(Sigma.determinant());
    b.resize(1,1);

    b = (dX.transpose()*Sigma.inverse()*dX)/2;

    p = 1/a*exp(-b(0,0));

    // std::cout<<" ===================== "<<std::endl;
    // std::cout<<"row : "<< row <<std::endl;
    // std::cout<<"col : "<< col <<std::endl;
    // std::cout<<"x   : "<< x.transpose()<<std::endl;
    // std::cout<<"mu  : "<< mu.transpose()<<std::endl;
    // std::cout<<"dX  : "<< dX.transpose()<<std::endl;
    // std::cout<<"a   : "<< a << std::endl;
    // std::cout<<"b   : "<< b(0,0) << std::endl;
    // std::cout<<"p   : "<< p << std::endl;
    // std::cout<<"det : "<< Sigma.determinant()<<std::endl;

    return p;
}

Estimator::LABEL Estimator::torqueEstimator(const Eigen::VectorXd &torque_data,
                                            const struct GMM_model model_1,
                                            const struct GMM_model model_2)
{
    Eigen::Vector2d p;
    Eigen::Vector2d responsibility;
    Estimator::LABEL torque_state;
    double summation;

    p(0) = Estimator::getGaussianDistribution(torque_data, model_1.mu, model_1.Sigma);
    p(1) = Estimator::getGaussianDistribution(torque_data, model_2.mu, model_2.Sigma);

    summation = model_1.pi*p(0) + model_2.pi*p(1);

    responsibility(0) = model_1.pi*p(0)/summation;
    responsibility(1) = model_1.pi*p(1)/summation;

    if (responsibility(0) >= responsibility(1))
        torque_state = SMALL;
    else
        torque_state = LARGE;

    return torque_state;
}

Estimator::LABEL Estimator::positionEstimator(const Eigen::VectorXd &position_data,
                                              const struct GMM_model model_zero,
                                              const struct GMM_model model_small,
                                              const struct GMM_model model_medium,
                                              const struct GMM_model model_large)
{
    Eigen::Vector4d p, responsibility;
    Estimator::LABEL position_state;
    double summation, max_val;
    int max_index;

    p(0) = Estimator::getGaussianDistribution(position_data, model_zero.mu, model_zero.Sigma);
    p(1) = Estimator::getGaussianDistribution(position_data, model_small.mu, model_small.Sigma);
    p(2) = Estimator::getGaussianDistribution(position_data, model_medium.mu, model_medium.Sigma);
    p(3) = Estimator::getGaussianDistribution(position_data, model_large.mu, model_large.Sigma);

    summation = model_zero.pi*p(0) + model_small.pi*p(1) + model_medium.pi*p(2) + model_large.pi*p(3);
    
    responsibility(0) = model_zero.pi*p(0)/summation;
    responsibility(1) = model_small.pi*p(1)/summation;
    responsibility(2) = model_medium.pi*p(2)/summation;
    responsibility(3) = model_large.pi*p(3)/summation;
    
    max_val = responsibility.maxCoeff();
    for(int i = 0; i < 4; i++)
    {
        if(responsibility(i) == max_val)    max_index = i;             
    }
    
    position_state = Estimator::LABEL (max_index+1);

    std::cout<<"====================="<<std::endl;
    std::cout<<p.transpose()<<std::endl;
    std::cout<<position_state<<std::endl;

    return position_state;
}

Estimator::CONTACT_STATE Estimator::contactStateTable(const Estimator::LABEL torque_label,
                                                      const Estimator::LABEL position_label)
{
    Estimator::CONTACT_STATE contact_state;

    if (torque_label == SMALL && position_label == ZERO)
        contact_state = SEARCH;
    else if (torque_label == SMALL && position_label == SMALL)
        contact_state = SEARCH;
    else if (torque_label == SMALL && position_label == MEDIUM)
        contact_state = TRANSITION;
    else if (torque_label == SMALL && position_label == LARGE)
        contact_state = DEVIATION;
    else if (torque_label == LARGE && position_label == ZERO)
        contact_state = SEARCH;
    else if (torque_label == LARGE && position_label == SMALL)
        contact_state = CROSSING;
    else if (torque_label == LARGE && position_label == MEDIUM)
        contact_state = TRANSITION;
    else if (torque_label == LARGE && position_label == LARGE)
        contact_state = INSERTION;
    return contact_state;
}
