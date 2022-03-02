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

Estimator::TORQUE_LABEL Estimator::torqueEstimator(const Eigen::VectorXd &torque_data,
                                                   const struct GMM_model model_1,
                                                   const struct GMM_model model_2)
{
    Eigen::Vector2d p;
    Eigen::Vector2d responsibility;
    Estimator::TORQUE_LABEL torque_state;
    double summation;

    p(0) = Estimator::getGaussianDistribution(torque_data, model_1.mu, model_1.Sigma);
    p(1) = Estimator::getGaussianDistribution(torque_data, model_2.mu, model_2.Sigma);

    summation = model_1.pi * p(0) + model_2.pi * p(1);

    responsibility(0) = model_1.pi * p(0) / summation;
    responsibility(1) = model_1.pi * p(1) / summation;

    if (responsibility(0) >= responsibility(1))
        torque_state = SMALL;
    else
        torque_state = LARGE;

    return torque_state;
}

Estimator::POS_VEL_LABEL Estimator::positionEstimator(const Eigen::VectorXd &position_data,
                                                      const struct GMM_model model_surface,
                                                      const struct GMM_model model_shallow,
                                                      const struct GMM_model model_floating,
                                                      const struct GMM_model model_deep)
{
    Eigen::Vector4d p, responsibility;
    Estimator::POS_VEL_LABEL position_state;
    double summation, max_val;
    int max_index;

    p(0) = Estimator::getGaussianDistribution(position_data, model_surface.mu, model_surface.Sigma);
    p(1) = Estimator::getGaussianDistribution(position_data, model_shallow.mu, model_shallow.Sigma);
    p(2) = Estimator::getGaussianDistribution(position_data, model_floating.mu, model_floating.Sigma);
    p(3) = Estimator::getGaussianDistribution(position_data, model_deep.mu, model_deep.Sigma);

    summation = model_surface.pi*p(0) + model_shallow.pi*p(1) + model_floating.pi*p(2) + model_deep.pi*p(3);
    
    responsibility(0) = model_surface.pi*p(0)/summation;
    responsibility(1) = model_shallow.pi*p(1)/summation;
    responsibility(2) = model_floating.pi*p(2)/summation;
    responsibility(3) = model_deep.pi*p(3)/summation;
    
    max_val = responsibility.maxCoeff();
    for(int i = 0; i < 4; i++)
    {
        if(responsibility(i) == max_val)    max_index = i;             
    }
    
    position_state = Estimator::POS_VEL_LABEL (max_index+1);

    // std::cout<<"====================="<<std::endl;
    // std::cout<<p.transpose()<<std::endl;
    // std::cout<<position_state<<std::endl;

    return position_state;
}

Estimator::CONTACT_STATE Estimator::contactStateTable(const Estimator::TORQUE_LABEL torque_label,
                                                      const Estimator::POS_VEL_LABEL position_label)
{
    Estimator::CONTACT_STATE contact_state;

    if (torque_label == SMALL && position_label == SURFACE)
        contact_state = SEARCH;
    else if (torque_label == SMALL && position_label == SHALLOW)
        contact_state = SEARCH;
    else if (torque_label == SMALL && position_label == FLOATING)
        contact_state = TRANSITION;
    else if (torque_label == SMALL && position_label == DEEP)
        contact_state = DEVIATION;
    else if (torque_label == LARGE && position_label == SURFACE)
        contact_state = SEARCH;
    else if (torque_label == LARGE && position_label == SHALLOW)
        contact_state = CROSSING;
    else if (torque_label == LARGE && position_label == FLOATING)
        contact_state = TRANSITION;
    else if (torque_label == LARGE && position_label == DEEP)
        contact_state = INSERTION;
    return contact_state;
}

// FOR TRIPLE ARM CASE
Estimator::TORQUE_LABEL_TRIPLE Estimator::torqueEstimatorTriple(const Eigen::VectorXd &torque_data,
                                                                const struct GMM_model model_1,
                                                                const struct GMM_model model_2,
                                                                const struct GMM_model model_3)
{
    Eigen::Vector3d p;
    Eigen::Vector3d r; // responsibility
    Estimator::TORQUE_LABEL_TRIPLE torque_state;
    double summation, max_val;
    int max_index;

    p(0) = Estimator::getGaussianDistribution(torque_data, model_1.mu, model_1.Sigma);
    p(1) = Estimator::getGaussianDistribution(torque_data, model_2.mu, model_2.Sigma);
    p(2) = Estimator::getGaussianDistribution(torque_data, model_3.mu, model_3.Sigma);

    summation = model_1.pi * p(0) + model_2.pi * p(1) + model_3.pi * p(2);

    r(0) = model_1.pi * p(0) / summation;
    r(1) = model_2.pi * p(1) / summation;
    r(2) = model_3.pi * p(2) / summation;
    max_val = r.maxCoeff();
    for (int i = 0; i < 3; i++)
    {
        if (r(i) == max_val)
            max_index = i;
    }

    torque_state = Estimator::TORQUE_LABEL_TRIPLE(max_index + 1);

    return torque_state;
}

Estimator::POS_VEL_LABEL_TRIPLE Estimator::positionEstimatorTriple(const Eigen::VectorXd &position_data,
                                                                   const struct GMM_model model_surface,
                                                                   const struct GMM_model model_shallow,
                                                                   const struct GMM_model model_floating,
                                                                   const struct GMM_model model_moderate,
                                                                   const struct GMM_model model_deep)
{
    Eigen::Vector5d p, r;
    Estimator::POS_VEL_LABEL_TRIPLE position_state;
    double summation, max_val;
    int max_index;

    p(0) = Estimator::getGaussianDistribution(position_data, model_surface.mu, model_surface.Sigma);
    p(1) = Estimator::getGaussianDistribution(position_data, model_shallow.mu, model_shallow.Sigma);
    p(2) = Estimator::getGaussianDistribution(position_data, model_floating.mu, model_floating.Sigma);
    p(3) = Estimator::getGaussianDistribution(position_data, model_moderate.mu, model_moderate.Sigma);
    p(4) = Estimator::getGaussianDistribution(position_data, model_deep.mu, model_deep.Sigma);


    summation = (model_surface.pi * p(0) + model_shallow.pi * p(1) +
                model_floating.pi * p(2) + model_moderate.pi * p(3) + 
                model_deep.pi * p(4));

    r(0) = model_surface.pi * p(0) / summation;
    r(1) = model_shallow.pi * p(1) / summation;
    r(2) = model_floating.pi * p(2) / summation;
    r(3) = model_moderate.pi * p(3) / summation;
    r(4) = model_deep.pi * p(4) / summation;
    // std::cout<<"--------------------"<<std::endl;
    // std::cout<<"surface   det : "<< model_surface.Sigma.determinant()<<std::endl;
    // std::cout<<"shallow   det : "<< model_shallow.Sigma.determinant()<<std::endl;
    // std::cout<<"floating  det : "<< model_floating.Sigma.determinant()<<std::endl;
    // std::cout<<"moderate  det : "<< model_moderate.Sigma.determinant()<<std::endl;
    // std::cout<<"deep      det : "<< model_deep.Sigma.determinant()<<std::endl;
    // std::cout<<"p : "<<p.transpose()<<std::endl;
    // std::cout<<"r: "<< r.transpose()<<std::endl;
    max_val = r.maxCoeff();
    for (int i = 0; i < 5; i++)
    {
        if (r(i) == max_val)
            max_index = i;
    }

    position_state = Estimator::POS_VEL_LABEL_TRIPLE(max_index + 1);

    // std::cout<<"====================="<<std::endl;
    // std::cout<<p.transpose()<<std::endl;
    // std::cout<<position_state<<std::endl;

    return position_state;
}

Estimator::CONTACT_STATE Estimator::contactStateTableTriple(const Estimator::TORQUE_LABEL_TRIPLE torque_label,
                                                            const Estimator::POS_VEL_LABEL_TRIPLE position_label)
{
    Estimator::CONTACT_STATE contact_state;

    if (torque_label == SMALL_T && position_label == SURFACE_T)
        contact_state = SEARCH;
    else if (torque_label == SMALL_T && position_label == SHALLOW_T)
        contact_state = SEARCH;
    else if (torque_label == SMALL_T && position_label == FLOATING_T)
        contact_state = TRANSITION;
    else if (torque_label == SMALL_T && position_label == MODERATE_T)
        contact_state = DEVIATION;
    else if (torque_label == SMALL_T && position_label == DEEP_T)
        contact_state = DEVIATION;

    else if (torque_label == MEDIUM_T && position_label == SURFACE_T)
        contact_state = SEARCH;
    else if (torque_label == MEDIUM_T && position_label == SHALLOW_T)
        contact_state = CROSSING;
    else if (torque_label == MEDIUM_T && position_label == FLOATING_T)
        contact_state = TRANSITION;
    else if (torque_label == MEDIUM_T && position_label == MODERATE_T)
        contact_state = DEVIATION; //INSERTION;
    else if (torque_label == MEDIUM_T && position_label == DEEP_T)
        contact_state = INSERTION;

    else if (torque_label == LARGE_T && position_label == SURFACE_T)
        contact_state = SEARCH;
    else if (torque_label == LARGE_T && position_label == SHALLOW_T)
        contact_state = CROSSING;
    else if (torque_label == LARGE_T && position_label == FLOATING_T)
        contact_state = TRANSITION;
    else if (torque_label == LARGE_T && position_label == MODERATE_T)
        contact_state = DEVIATION;
    else if (torque_label == LARGE_T && position_label == DEEP_T)
        contact_state = DEVIATION;

    return contact_state;
}