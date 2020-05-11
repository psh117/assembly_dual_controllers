#ifndef CRITERIA2_H
#define CRITERIA2_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Geometry>

#include <assembly_dual_controllers/utils/dyros_math.h>

using namespace dyros_math;
using namespace Eigen;
// using namespace FuzzyLogic;

namespace Criteria2
{
    static bool checkContact(const Eigen::Vector3d &force, const Isometry3d &T_wa, const double threshold)
    {
        // force wrt {W}
        bool result;
        Eigen::Vector3d f_a;
        double contact_force;

        f_a = T_wa.linear().inverse()*force;
        contact_force = f_a(2); // z- axis is always assembly direction wrt {A}

        if(contact_force >= threshold) result = true; //detect contact!
        else result = false;

        return result;
    }

    static bool detectHole(const Isometry3d &origin, const Isometry3d &current, const Eigen::Vector3d &force,
            const Isometry3d &T_wa, const double depth, const double friction)
    {
        bool result;
        Eigen::Vector3d p, f_a;      
        double dz, f;

        p = origin.translation() - current.translation();
        p = T_wa.linear().inverse()*p;
        
        dz = p(2); // assembly direction is always z-axis wrt {A}

        f_a = T_wa.linear().inverse()*force;
        f = sqrt(pow(f_a(0),2) + pow(f_a(1),2)); // reaction force from X-Y plane wrt {A}

        if(dz <= depth && f <= friction) result = false; // any hole is not detected
        else result = true; // a hole is detected

        return result;     
    }

    static bool holeVerify(const Eigen::Vector3d &force, const Isometry3d &T_wa, const double threshold)
    {
        // force wrt {W}
        bool result;
        Eigen::Vector3d f_a;
        double f;
        
        f_a = T_wa.linear().inverse()*force;

        f = sqrt(pow(f_a(0),2) + pow(f_a(1),2)); // reaction force from X-Y plane wrt {A}

        if(f >= threshold) result = true; //detect contact!
        else result = false;

        return result;
    }

    static bool timeOut(const double current_time, const double start_time, const double duration)
    {
        double running_time;
        bool is_timeout;

        running_time = current_time - start_time;

        if(running_time >= duration) is_timeout = true;
        else is_timeout = false;
        
        return is_timeout;
    }
    
    // static double fuzzyLogic(const double origin, const double vel, const double dis, const double force)
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
};

#endif // CRITERIA_H