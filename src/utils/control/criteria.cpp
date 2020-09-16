#include <assembly_dual_controllers/utils/control/criteria.h>
bool Criteria::reachGoal2D(const Eigen::Vector3d &p,
                           const Eigen::Vector3d &q,
                           const double threshold,
                           const Eigen::Isometry3d &T_wa)
{
    //ignore the last component(insertion direction) of the vectors!! only 2d!
    Eigen::Vector3d u, v;
    double dis;
    bool result;

    u = T_wa.linear() * p;
    v = T_wa.linear() * q;

    for (int i = 0; i < 2; i++)
    {
        dis += pow(p(i) - q(i), 2);
    }
    dis = sqrt(dis);

    if (dis < threshold)
        result = true;
    else
        result = false;
    return result;
}
bool Criteria::reachGoal3D(const Eigen::Vector3d &p,
                           const Eigen::Vector3d &q,
                           const double threshold,
                           const Eigen::Isometry3d &T_wa)
{
    Eigen::Vector3d u, v;
    double dis;
    bool result;

    u = T_wa.linear() * p;
    v = T_wa.linear() * q;

    for (int i = 0; i < 3; i++)
    {
        dis += pow(p(i) - q(i), 2);
    }
    dis = sqrt(dis);

    if (dis < threshold)
        result = true;
    else
        result = false;
    return result;
}

bool Criteria::checkContact(const Eigen::Vector3d &force,
                            const Eigen::Isometry3d &T_wa,
                            const double threshold)
{
    // force wrt {W}
    bool result;
    Eigen::Vector3d f_a;
    double contact_force;

    f_a = T_wa.linear().inverse() * force;
    // contact_force = abs(f_a(2)); // z- axis is always assembly direction wrt {A}
    contact_force = f_a(2);
    if (contact_force >= threshold)
    {
        std::cout << "contact_force: " << contact_force << std::endl;

        result = true; //detect contact!
    }

    else
        result = false;

    return result;
}

bool Criteria::detectHole(const Eigen::Isometry3d &origin,
                          const Eigen::Isometry3d &current,
                          const Eigen::Vector3d &force,
                          const Eigen::Isometry3d &T_wa,
                          const double friction)
{
    bool result;
    Eigen::Vector3d f_a;
    double f;

    f_a = T_wa.linear().inverse() * force;
    f = sqrt(pow(f_a(0), 2) + pow(f_a(1), 2)); // reaction force from X-Y plane wrt {A}

    if (f >= friction)
    {
        result = true;
        std::cout << "f_a: " << f << std::endl;
    }
    else
        result = false;

    return result;
}

bool Criteria::detectObject(const Eigen::Vector3d &force,
                            const Eigen::Isometry3d &T_wa,
                            const double blocking_force)
{
    bool result;
    Eigen::Vector3d f_a;
    double f;

    f_a = T_wa.linear().inverse() * force;
    f = sqrt(pow(f_a(0), 2) + pow(f_a(1), 2)); // reaction force from X-Y plane wrt {A}

    if (f >= blocking_force)
    {
        result = true;
        std::cout<<"f_detection : "<<f<<std::endl;
    }
    else
        result = false;

    return result;
}

bool Criteria::checkContact(const double current_force,
                            const double threshold)
{
    double contact_force = threshold; //-6.0
    bool result;

    if (current_force <= contact_force)
        result = true; //contact is detected
    else
        result = false;
    return result;
}

// bool Criteria::checkDisplacement(const Eigen::Isometry3d &origin,
//                                  const Eigen::Isometry3d &current,
//                                  const Eigen::Isometry3d &T_wa,
//                                  const double depth) //0.270
// {
//     bool result;
//     Eigen::Vector3d p, f_a;
//     double dz, f;

//     p = origin.translation() - current.translation();
//     p = T_wa.linear().inverse() * p;

//     dz = p(2); // assembly direction is always z-axis wrt {A}

//     // if(dz )
// }

// bool Criteria::detectHole(const double origin,
//                 const double current_position,
//                 const double friction,
//                 const double depth_threshold,
//                 const double force_threshold)
// {
//     double dz;
//     bool result; // true = success, false = fail

//     dz = origin - current_position;

//     // Keep considering conditions, which is || or &&
//     if (dz >= depth_threshold && friction >= force_threshold)
//     {
//         //std::cout<<"Z_l + F_L"<<std::endl;
//         result = true; //z_l + f_l
//     }
//     else if (dz >= depth_threshold && friction < force_threshold)
//     {
//         //std::cout<<"Z_l + F_s"<<std::endl;
//         result = true; //z_l + f_s
//     }
//     else if (dz < depth_threshold && friction >= force_threshold)
//     {
//         //std::cout<<"Z_s + F_l"<<std::endl;
//         //std::cout<<dz<<"    "<<friction<<std::endl;
//         result = true; //z_s + f_l
//     }
//     else
//         result = false; //z_s + f_s
//     return result;
// }

double Criteria::judgeInsertion(const double origin,
                                const double current_position,
                                const double current_velocity,
                                const double friction,
                                const double target_depth)
{
    double velocity_threshold = 0.01; // fix it later
    double force_threshold = 15.0;
    double depth_threshold;
    double dz;
    double result; //1.0 = success, 0.0 = ?, -1.0 = fail(dead or fake)

    depth_threshold = target_depth * 0.8; //
    dz = origin - current_position;
    result = 0.0;

    if (abs(current_velocity) <= velocity_threshold)
    {
        if (dz < depth_threshold && friction <= force_threshold)
        {
            std::cout << "keep inserting" << std::endl;
            result = 0.0; //not to consider
        }
        else if (dz < depth_threshold && friction > force_threshold)
        {
            std::cout << "fake" << std::endl;
            result = -1.0; // fake
        }
        else if ((depth_threshold <= dz && dz <= target_depth) && friction <= force_threshold)
        {
            std::cout << "dead" << std::endl;
            result = -1.0; // dead
        }
        else if ((depth_threshold <= dz && dz <= target_depth) && friction > force_threshold)
        {
            std::cout << "success" << std::endl;
            result = 1.0; // success
        }
        else
        {
            std::cout << "dead" << std::endl;
            result = -1.0; // dead
        }
    }

    else
    {
        if (dz >= target_depth)
            result = -1.0;
    }
}

bool Criteria::timeOut(const double current_time,
                       const double start_time,
                       const double duration)
{
    double running_time;
    bool is_timeout;

    running_time = current_time - start_time;

    if (running_time >= duration)
        is_timeout = true;
    else
        is_timeout = false;

    return is_timeout;
}

bool Criteria::checkForceLimit(const double f,
                               const double threshold)
{
    bool is_done;

    if (f >= threshold)
        is_done = true;
    else
        is_done = false;

    return is_done;
}

bool Criteria::checkForceLimit(const Eigen::Vector3d &force,
                               const double threshold)
{
    double f;

    f = sqrt(pow(force(0), 2) + pow(force(1), 2) + pow(force(2), 2));
    // std::cout<<"current force: "<<force_sqrt_<<std::endl;
    // std::cout<<"threshold force: "<<threshold<<std::endl;
    if (f >= threshold)
        return true;
    else
        return false;
}

bool Criteria::checkMomentLimit(const std::vector<double> m1,
                                const std::vector<double> m2,
                                const std::vector<double> m3,
                                const int swing_dir,
                                const double threshold)
{
    bool is_done;
    int size;
    double sum;
    double avg;

    std::vector<double> m;

    if (swing_dir == 1)
        m = m1;
    if (swing_dir == 2)
        m = m2;
    if (swing_dir == 3)
        m = m3;

    size = int(m.size());

    for (int i = 0; i < size; i++)
    {
        sum += fabs(double(m[i]));
    }

    avg = sum / size;

    if (avg >= threshold)
        is_done = true;
    else
        is_done = false;

    // std::cout<<"size : "<<size<<std::endl;
    // std::cout<<"sum : "<<sum<<std::endl;
    std::cout << "moment avg : " << avg << std::endl;
    // std::cout<<"is_done : "<<is_done<<std::endl;

    return is_done;
}

bool Criteria::contactLossInProbing(const Eigen::Vector3d &vel,
                                    const Eigen::Vector3d &obj_dir,
                                    const double threshold)
{
    double selected_vel;
    bool result;

    selected_vel = abs(obj_dir.dot(vel));

    if (selected_vel >= threshold)
        result = true; // loss contact!
    else
        result = false;

    return result;
}

bool Criteria::holdHeavyMass(const Eigen::Vector3d &input_force,
                             const double threshold)
{
    double m;
    bool result;

    m = sqrt(pow(input_force(0),2) + pow(input_force(1),2) + pow(input_force(2),2));

    if (m >= threshold)
        result = true;
    else
        result = false;

    std::cout << "The holding mass is : " << m / 10 << std::endl;
    return result;
}

//  double Criteria::fuzzyLogic(const double origin, const double vel, const double dis, const double force)
// {
//     Eigen::Vector2d v;
//     Eigen::Matrix<double, 5, 1> z;
//     Eigen::Vector3d f;

//     double u;

//     v = velocityInput(vel);
//     z = displacementInput(origin, dis);
//     f = forceInput(force);

//     u = fuzzyOutput(v,z,f);
//     // std::cout<<"---------------------------------------"<<std::endl;
//     return u;
// }

// bool Criteria::checkDisplacement(const double origin, const double cur_position, const double threshold)
// {
//     bool is_done;

//     double del;

//     del = abs(cur_position - origin);

//     if (del < threshold)
//         is_done = true;
//     else
//         is_done = false;

//     return is_done;
// }
