#ifndef CRITERIA_H
#define CRITERIA_H

#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <assembly_dual_controllers/utils/dyros_math.h>

using namespace dyros_math;
// using namespace FuzzyLogic;

namespace Criteria
{
bool checkContact(const Eigen::Vector3d &force,
                  const Eigen::Isometry3d &T_wa,
                  const double threshold);

bool detectHole(const Eigen::Isometry3d &origin,
                const Eigen::Isometry3d &current,
                const Eigen::Vector3d &force,
                const Eigen::Isometry3d &T_wa,
                const double friction);

bool checkContact(const double current_force,
                  const double threshold);

bool checkDisplacement(const double origin,
                       const double current_position,
                       const double threshold); // the distance from end-effecto to the edge of the component

bool detectHole(const double origin,
                const double current_position,
                const double friction,
                const double depth_threshold,
                const double force_threshold);

double judgeInsertion(const double origin,
                      const double current_position,
                      const double current_velocity,
                      const double friction,
                      const double target_depth);

bool timeOut(const double current_time,
             const double start_time,
             const double duration);

bool checkForceLimit(const double f, const double threshold);

bool checkMomentLimit(const std::vector<double> m1,
                      const std::vector<double> m2,
                      const std::vector<double> m3,
                      const int swing_dir,
                      const double threshold);

//  double fuzzyLogic(const double origin, const double vel, const double dis, const double force)

bool checkDisplacement(const Eigen::Isometry3d &origin,
                       const Eigen::Isometry3d &current,
                       const Eigen::Isometry3d &T_wa,
                       const double depth);
}; 

#endif // CRITERIA_H