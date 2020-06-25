#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <assembly_dual_controllers/utils/dyros_math.h>
#include <cstdlib>
#include <ctime>

namespace PegInHole
{
Eigen::Vector3d straightMove(const Eigen::Vector3d origin,
                             const Eigen::Vector3d current_position,
                             const Eigen::Matrix<double, 6, 1> current_velocity,
                             const int dir,
                             const double speed,
                             const double current_time,
                             const double init_time);

Eigen::Vector3d oneDofMove(const Eigen::Vector3d origin,
                           const Eigen::Vector3d current_position,
                           const double target_position,
                           const Eigen::Matrix<double, 6, 1> current_velocity,
                           const double current_time,
                           const double init_time,
                           const double duration,
                           const double desired_speed, //only positive value!!!
                           const int direction);       // 0 -> x, 1 -> y, 2 -> z //DESIRED DIRECTION!!

Eigen::Vector3d twoDofMove(const Eigen::Vector3d origin,
                           const Eigen::Vector3d current_position,
                           const Eigen::Vector3d target_position,
                           const Eigen::Matrix<double, 6, 1> current_velocity,
                           const double current_time,
                           const double init_time,
                           const double duration,
                           const double desired_speed,
                           const int direction); // 0 -> x, 1 -> y, 2 -> z //NOT DESIRED DIRECTION!!

Eigen::Vector3d oneDofMoveEE(const Eigen::Vector3d origin,
                             const Eigen::Matrix3d init_rot,
                             const Eigen::Vector3d current_position,
                             const Eigen::Matrix<double, 6, 1> current_velocity,
                             const double current_time,
                             const double init_time,
                             const double duration,
                             const double target_distance, // + means go forward, - mean go backward
                             const int direction);         // 0 -> x_ee, 1 -> y_ee, 2 -> z_ee //DESIRED DIRECTION W.R.T END EFFECTOR!!

Eigen::Matrix<double, 6, 1> keepCurrentState(const Eigen::Vector3d initial_position,
                                             const Eigen::Matrix3d initial_rotation,
                                             const Eigen::Vector3d position,
                                             const Eigen::Matrix3d rotation,
                                             const Eigen::Matrix<double, 6, 1> current_velocity,
                                             const double kp = 5000, const double kv = 100);

Eigen::Vector3d generateSpiral(const Eigen::Vector3d origin,
                               const Eigen::Vector3d current_position,
                               const Eigen::Matrix<double, 6, 1> current_velocity,
                               const double pitch,
                               const double lin_vel,
                               const int dir, //the direction where a peg is inserted
                               const double current_time,
                               const double init_time,
                               const double spiral_duration);

Eigen::Matrix<double, 3, 1> generateSpiralWithRotation(const Eigen::Matrix3d initial_rotation_M,
                                                       const Eigen::Matrix3d rotation_M,
                                                       const Eigen::Matrix<double, 3, 1> current_angular_velocity,
                                                       const double current_time,
                                                       const double init_time,
                                                       const double duration,
                                                       const double direction, //0 = the first motion, 1 = CCW, 2 == CW
                                                       const double axis);     // 0 = x-axis, 1 = y-axis, 2 = z-axis

Eigen::Matrix<double, 3, 1> generateSpiralWithRotationGainEe(const Eigen::Matrix3d initial_rotation_M,
                                                             const Eigen::Matrix3d rotation_M,
                                                             const Eigen::Matrix<double, 3, 1> current_angular_velocity,
                                                             const double current_time,
                                                             const double init_time,
                                                             const double duration,
                                                             const double direction, //0 = the first motion, 1 = CCW, 2 == CW
                                                             const double axis,      // 0 = x-axis, 1 = y-axis, 2 = z-axis
                                                             const double kp,
                                                             const double kv);

Eigen::Matrix<double, 3, 1> generateSpiralWithRotationEe(const Eigen::Matrix3d initial_rotation_M,
                                                         const Eigen::Matrix3d rotation_M,
                                                         const Eigen::Matrix<double, 3, 1> current_angular_velocity,
                                                         const double current_time,
                                                         const double init_time,
                                                         const double duration,
                                                         const double direction, //0 = the first motion, 1 = CCW, 2 == CW
                                                         const double axis,
                                                         const int rand); // 0 = x-axis, 1 = y-axis, 2 = z-axis

Eigen::Vector3d keepOrientationPerpenticular(const Eigen::Matrix3d initial_rotation_M,
                                             const Eigen::Matrix3d rotation_M,
                                             const Eigen::Matrix<double, 6, 1> current_velocity,
                                             const double duration,
                                             const double current_time,
                                             const double init_time);

Eigen::Vector3d keepOrientationPerpenticularOnlyXY(const Eigen::Matrix3d initial_rotation_M,
                                                   const Eigen::Matrix3d rotation_M,
                                                   const Eigen::Matrix<double, 6, 1> current_velocity,
                                                   const double duration,
                                                   const double current_time,
                                                   const double init_time);

Eigen::Vector3d rotateOrientationPerpenticular(const Eigen::Matrix3d initial_rotation_M,
                                               const Eigen::Matrix3d rotation_M,
                                               const Eigen::Matrix<double, 6, 1> current_velocity,
                                               const double duration,
                                               const double current_time,
                                               const double init_time);

// deprecated
Eigen::Vector3d keepCurrentOrientation(const Eigen::Matrix3d initial_rotation_M,
                                       const Eigen::Matrix3d rotation_M,
                                       const Eigen::Matrix<double, 6, 1> current_velocity,
                                       const double kp = 200,
                                       const double kd = 5);

Eigen::Vector3d rotateWithGlobalAxis(const Eigen::Matrix3d initial_rotation_M,
                                     const Eigen::Matrix3d rotation_M,
                                     const Eigen::Matrix<double, 6, 1> current_velocity,
                                     const double goal,
                                     const double init_time,
                                     const double current_time,
                                     const double end_time,
                                     const int dir); // 0 = x-axis, 1 = y-axis, 2 = z-axis

Eigen::Vector3d rotateWithEeAxis(const Eigen::Matrix3d initial_rotation_M,
                                 const Eigen::Matrix3d rotation_M,
                                 const Eigen::Matrix<double, 6, 1> current_velocity,
                                 const double goal,
                                 const double init_time,
                                 const double current_time,
                                 const double end_time,
                                 const int dir); // 0 = x-axis, 1 = y-axis, 2 = z-axis

double calculateFriction(const int dir, const Eigen::Vector3d f_measured, double friction);

Eigen::Vector3d computeNomalVector(const Eigen::Vector3d p1,
                                   const Eigen::Vector3d p2,
                                   const Eigen::Vector3d p3);

Eigen::Vector3d getTiltDirection(const Eigen::Isometry3d T_ea);

bool setTilt(const Eigen::Isometry3d T_ea, const double threshold);

Eigen::Vector6d tiltMotion(const Eigen::Isometry3d origin, const Eigen::Isometry3d current,
                           const Eigen::Ref<const Eigen::Vector6d> &xd, const Eigen::Isometry3d T_ea,
                           const Eigen::Vector3d tilt_axis, //wrt {A}
                           const double tilt_angle,
                           const double t, const double t_0, const double duration);
// bool::judgeHeavyMass(const double t, const double )

Eigen::Vector3d straightMoveEE(const Eigen::Isometry3d &origin,
                               const Eigen::Isometry3d &current,
                               const Eigen::Ref<const Eigen::Vector6d> &xd,
                               const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                               const Eigen::Isometry3d &T_ad,
                               const double speed,
                               const double t,
                               const double t_0);

Eigen::Vector3d threeDofMove(const Eigen::Isometry3d &origin,
                             const Eigen::Isometry3d &current,
                             const Eigen::Vector3d target,
                             const Eigen::Ref<const Eigen::Vector6d> &xd,
                             const double t,
                             const double t_0,
                             const double duration);

Eigen::Vector6d keepCurrentPose(const Eigen::Isometry3d &origin,
                                const Eigen::Isometry3d &current,
                                const Eigen::Ref<const Eigen::Vector6d> &xd,
                                const double kp = 200,
                                const double kv = 5,
                                const double kp_o = 200,
                                const double kv_o = 5,
                                const Eigen::Ref<const Eigen::Matrix6d>& lambda = Eigen::Matrix6d::Identity());

Eigen::Vector3d keepCurrentPosition(const Eigen::Isometry3d &origin,
                                    const Eigen::Isometry3d &current,
                                    const Eigen::Ref<const Eigen::Vector6d> &xd,
                                    const double kp = 5000,
                                    const double kv = 100);

Eigen::Vector3d keepCurrentOrientation(const Eigen::Isometry3d &origin,
                                       const Eigen::Isometry3d &current,
                                       const Eigen::Ref<const Eigen::Vector6d> &xd,
                                       const double kp = 200,
                                       const double kv = 5);

Eigen::Vector3d pressEE(const Eigen::Isometry3d &origin,
                        const Eigen::Isometry3d &current,
                        const Eigen::Ref<const Eigen::Vector6d> &xd,
                        const double force,
                        const Eigen::Isometry3d &T_ea); //the direction where a peg is inserted, wrt {E} .i.e., T_ga

Eigen::Vector3d generateSpiralEE(const Eigen::Isometry3d &origin,
                                 const Eigen::Isometry3d &current,
                                 const Eigen::Ref<const Eigen::Vector6d> &xd,
                                 const double pitch,
                                 const double lin_vel,
                                 const double force,
                                 const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                 const double t,
                                 const double t_0,
                                 const double duration);

Eigen::Vector3d rotateWithAseemblyDir(const Eigen::Isometry3d &origin,
                                      const Eigen::Isometry3d &current,
                                      const Eigen::Ref<const Eigen::Vector6d> &xd,
                                      const Eigen::Isometry3d T_ea,
                                      const double target_angle,
                                      const double t,
                                      const double t_0,
                                      const double duration);

Eigen::Vector3d rotateWithMat(const Eigen::Isometry3d &origin,
                              const Eigen::Isometry3d &current,
                              const Eigen::Ref<const Eigen::Vector6d> &xd,
                              const Eigen::Matrix3d target_rot, //wrt {E}
                              const double t,
                              const double t_0,
                              const double duration);

Eigen::Vector3d generateWiggleMotionEE(const Eigen::Isometry3d &origin,
                                       const Eigen::Isometry3d &current,
                                       const Eigen::Isometry3d &T_ea,
                                       const Eigen::Ref<const Eigen::Vector6d> &xd,
                                       const double angle,
                                       const double w, //rad/s
                                       const double t,
                                       const double t_0);

Eigen::Vector3d generateYawingMotionEE(const Eigen::Isometry3d &origin,
                                       const Eigen::Isometry3d &current,
                                       const Eigen::Isometry3d &T_ea,
                                       const Eigen::Ref<const Eigen::Vector6d> &xd,
                                       const double yawing_angle,
                                       const double duration,
                                       const double t,
                                       const double t_0);


Eigen::Vector3d generateTwistSearchMotionEE(const Eigen::Isometry3d &origin,
                                            const Eigen::Isometry3d &current,
                                            const Eigen::Isometry3d &T_ea,
                                            const Eigen::Ref<const Eigen::Vector6d> &xd,
                                            const double angle,
                                            const double t,
                                            const double t_0,
                                            const double duration);

Eigen::Vector3d generateRotationSearchMotionEE(const Eigen::Isometry3d &origin,
                                               const Eigen::Isometry3d &current,
                                               const Eigen::Isometry3d &T_ea,
                                               const Eigen::Ref<const Eigen::Vector6d> &xd,
                                               const double angle,
                                               const double t,
                                               const double t_0,
                                               const double duration);


}; // namespace PegInHole
