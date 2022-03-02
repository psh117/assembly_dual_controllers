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
    Eigen::Vector3d oneDofMoveEE(const Eigen::Isometry3d &origin,
                                 const Eigen::Isometry3d &current,
                                 const Eigen::Matrix<double, 6, 1> &xd,
                                 const Eigen::Isometry3d &T_ea,
                                 const double t,
                                 const double t_0,
                                 const double duration,
                                 const double target_distance, // + means go forward, - mean go backward
                                 const int direction);         // 0 -> x_ee, 1 -> y_ee, 2 -> z_ee //DESIRED DIRECTION W.R.T END EFFECTOR!!

    Eigen::Vector3d twoDofMoveEE(const Eigen::Isometry3d &origin,
                                 const Eigen::Isometry3d &current,
                                 const Eigen::Matrix<double, 6, 1> &xd,
                                 const Eigen::Isometry3d &T_ea,
                                 const double t,
                                 const double t_0,
                                 const double duration,
                                 const Eigen::Vector3d &target_position,
                                 const int direction);

    Eigen::Matrix<double, 6, 1> keepCurrentState(const Eigen::Vector3d initial_position,
                                                 const Eigen::Matrix3d initial_rotation,
                                                 const Eigen::Vector3d position,
                                                 const Eigen::Matrix3d rotation,
                                                 const Eigen::Matrix<double, 6, 1> current_velocity,
                                                 const double kp = 700, const double kv = 40);

    Eigen::Vector3d generateSpiral(const Eigen::Vector3d origin,
                                   const Eigen::Vector3d current_position,
                                   const Eigen::Matrix<double, 6, 1> current_velocity,
                                   const double pitch,
                                   const double lin_vel,
                                   const int dir, //the direction where a peg is inserted
                                   const double current_time,
                                   const double init_time,
                                   const double spiral_duration);

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

    Eigen::Vector3d getNormalForce(const Eigen::Vector3d &normal_vector,
                                   const double force);

    // deprecated

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
                               const double t,
                               const double t_0,
                               const double duration,
                               const double kp = 700.0,
                               const double kv = 40.0);

    Eigen::Vector3d pinMoveEE(const Eigen::Isometry3d &origin,
                              const Eigen::Isometry3d &current,
                              const Eigen::Ref<const Eigen::Vector6d> &xd,
                              const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                              const Eigen::Isometry3d &T_ad,
                              const double speed,
                              const double t,
                              const double t_0);

    Eigen::Vector3d straightMotionEE(const Eigen::Isometry3d &origin,
                                     const Eigen::Isometry3d &current,
                                     const Eigen::Ref<const Eigen::Vector6d> &xd,
                                     const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                     const Eigen::Vector3d &target_dir,
                                     const double speed,
                                     const double t,
                                     const double t_0,
                                     const double kp = 700,
                                     const double kv = 20);

    Eigen::Vector3d straightMotion(const Eigen::Isometry3d &origin,
                                   const Eigen::Isometry3d &current,
                                   const Eigen::Ref<const Eigen::Vector6d> &xd,
                                   const Eigen::Vector3d &target_dir,
                                   const double speed,
                                   const double t,
                                   const double t_0,
                                   const double kp = 700,
                                   const double kv = 20);

    Eigen::Vector3d approachComponentEE(const Eigen::Isometry3d &origin,
                                        const Eigen::Isometry3d &current,
                                        const Eigen::Ref<const Eigen::Vector6d> &xd,
                                        const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                        const double speed,
                                        const double t,
                                        const double t_0,
                                        const double kp = 700,
                                        const double kv = 40);

    Eigen::Vector3d threeDofMoveEE(const Eigen::Isometry3d &origin,
                                   const Eigen::Isometry3d &current,
                                   const Eigen::Vector3d &target,
                                   const Eigen::Ref<const Eigen::Vector6d> &xd,
                                   const Eigen::Isometry3d &T_ea,
                                   const double t,
                                   const double t_0,
                                   const double duration,
                                   const double kp = 700,
                                   const double kv = 10);

    Eigen::Vector3d threeDofMove(const Eigen::Isometry3d &origin,
                                 const Eigen::Isometry3d &current,
                                 const Eigen::Vector3d &target,
                                 const Eigen::Ref<const Eigen::Vector6d> &xd,
                                 const double t,
                                 const double t_0,
                                 const double duration,
                                 const double kp = 700,
                                 const double kv = 40);

    Eigen::Vector6d keepCurrentPose(const Eigen::Isometry3d &origin,
                                    const Eigen::Isometry3d &current,
                                    const Eigen::Ref<const Eigen::Vector6d> &xd,
                                    const double kp = 700,
                                    const double kv = 40,
                                    const double kp_o = 2000,
                                    const double kv_o = 15,
                                    const Eigen::Ref<const Eigen::Matrix6d> &lambda = Eigen::Matrix6d::Identity());

    Eigen::Vector3d keepCurrentPosition(const Eigen::Isometry3d &origin,
                                        const Eigen::Isometry3d &current,
                                        const Eigen::Ref<const Eigen::Vector6d> &xd,
                                        const double kp = 700,
                                        const double kv = 40);

    Eigen::Vector3d keepCurrentOrientation(const Eigen::Isometry3d &origin,
                                           const Eigen::Isometry3d &current,
                                           const Eigen::Ref<const Eigen::Vector6d> &xd,
                                           const double kp = 2000,
                                           const double kv = 15);

    Eigen::Vector3d pressEE(const double force, 
                            const Eigen::Ref<const Eigen::Vector6d> &xd,
                            const Eigen::Isometry3d &T_wa,
                            const double speed, 
                            const double kp = 100);

    Eigen::Vector3d pressCubicEE(const Eigen::Isometry3d &origin,
                                const Eigen::Isometry3d &current,
                                const Eigen::Ref<const Eigen::Vector6d> &xd,
                                const Eigen::Isometry3d &T_wa,
                                const double force,
                                const double t,
                                const double t_0,
                                const double duration,
                                const double f_init = 0.0,
                                const double kp = 800,
                                const double kv = 20);

    Eigen::Vector6d pressCubicEE2(const Eigen::Isometry3d &origin,
                                  const Eigen::Isometry3d &current,
                                  const Eigen::Ref<const Eigen::Vector6d> &xd,
                                  const Eigen::Isometry3d &T_wa,
                                  const double force,
                                  const double t,
                                  const double t_0,
                                  const double duration,
                                  const double f_init = 0.0,
                                  const double kp = 800,
                                  const double kv = 20);

    Eigen::Vector6d generateTwistSpiralEE(const Eigen::Isometry3d &origin,
                                          const Eigen::Isometry3d &current,
                                          const Eigen::Ref<const Eigen::Vector6d> &xd,
                                          const double pitch,
                                          const double lin_vel,
                                          const double force,
                                          const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                          const double angle,
                                          const double t,
                                          const double t_0,
                                          const double spiral_duration,
                                          const double revolve_duration);

    Eigen::Vector6d generateTwistSpiralEE_savedata(const Eigen::Isometry3d &origin,
                                                   const Eigen::Isometry3d &current,
                                                   const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                   const double pitch,
                                                   const double lin_vel,
                                                   const double force,
                                                   const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                                   const double angle,
                                                   const double t,
                                                   const double t_0,
                                                   const double spiral_duration,
                                                   const double revolve_duration);
    
    Eigen::Vector3d generateSpiralEE(const Eigen::Isometry3d &origin,
                                     const Eigen::Isometry3d &current,
                                     const Eigen::Ref<const Eigen::Vector6d> &xd,
                                     const double pitch,
                                     const double lin_vel,
                                     const double force,
                                     const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                     const double t,
                                     const double t_0,
                                     const double duration,
                                     const double f_gain = 5.0,
                                     const bool set_tilt = false,
                                     const double kp = 1000.0,
                                     const double kv = 20.0);

    Eigen::Vector6d generateSpiralEE2(const Eigen::Isometry3d &origin,
                                      const Eigen::Isometry3d &current,
                                      const Eigen::Ref<const Eigen::Vector6d> &xd,
                                      const double pitch,
                                      const double lin_vel,
                                      const double force,
                                      const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                      const double t,
                                      const double t_0,
                                      const double duration,
                                      const double f_gain = 5.0,
                                      const bool set_tilt = false,
                                      const double kp = 1000.0,
                                      const double kv = 20.0);

    Eigen::Vector6d generateSpiralEE_datasave(const Eigen::Isometry3d &origin,
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
                                  const double duration,
                                  const double kp = 2000,
                                  const double kv = 15);

    Eigen::Vector3d generateWiggleMotionEE(const Eigen::Isometry3d &origin,
                                           const Eigen::Isometry3d &current,
                                           const Eigen::Isometry3d &T_ea,
                                           const Eigen::Ref<const Eigen::Vector6d> &xd,
                                           const double angle,
                                           const double w, //rad/s
                                           const double t,
                                           const double t_0,
                                           const double kp = 500.0);

    Eigen::Vector3d generateWiggleMotionEE_Zaxis(const Eigen::Isometry3d &origin,
                                                 const Eigen::Isometry3d &current,
                                                 const Eigen::Isometry3d &T_ea,
                                                 const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                 const double angle,
                                                 const double w, //rad/s
                                                 const int a,
                                                 const double b,
                                                 const double t_offset,
                                                 const double t,
                                                 const double t_0,
                                                 const double k_p = 1000);

    Eigen::Vector3d generateYawingMotionEE(const Eigen::Isometry3d &origin,
                                           const Eigen::Isometry3d &current,
                                           const Eigen::Isometry3d &T_ea,
                                           const Eigen::Ref<const Eigen::Vector6d> &xd,
                                           const double yawing_angle,
                                           const double duration,
                                           const double t,
                                           const double t_0);
                                           
    Eigen::Vector6d generateTwistSearchMotion(const Eigen::Isometry3d &origin,
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
                                                    const double duration,
                                                    const double pressing_force);

    void getCompensationWrench(Eigen::Vector6d &accumulated_wrench,
                               const Eigen::Ref<const Eigen::Vector6d> &measured_wrench,
                               const int num_start,
                               const int num,
                               const int num_max);
                               
    Eigen::Vector3d getNormalVector(const Eigen::Vector3d &v);

    Eigen::MatrixXd LeastSquareEdgeProbing(const std::vector<double> x_vec,
                                           const std::vector<double> y_vec);

    Eigen::Vector7d inspectJointLimit(const Eigen::Vector7d &joint_angle,
                                      const Eigen::Vector7d &joint_angle_ub,
                                      const Eigen::Vector7d &joint_angle_lb,
                                      const double inspection = 5,
                                      const double angle_margin = 10*DEG2RAD);

    Eigen::Vector7d nullSpaceJointTorque(const Eigen::Vector7d &q_curr,
                                         const Eigen::Vector7d &q_init,
                                         const Eigen::Vector7d &q_target,
                                         const Eigen::Vector7d &q_dot,
                                         const double t,
                                         const double t_0,
                                         const double duration,
                                         const double kp = 10,
                                         const double kv = 0.1);

    Eigen::Vector7d getNullSpaceJointTarget(const Eigen::Matrix<double, 7, 2> &joint_limit_info,
                                            const Eigen::Vector7d &q_init);

    Eigen::Vector3d followSphericalCoordinateEE(const Eigen::Isometry3d &origin,
                                                const Eigen::Isometry3d &current,
                                                const Eigen::Isometry3d &target,
                                                const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                const Eigen::Isometry3d &T_7a,
                                                const double t,
                                                const double t_0,
                                                const double duration,
                                                const double radius,
                                                const double kp = 700,
                                                const double kv = 40);

    Eigen::Vector3d generateMoment(const Eigen::Isometry3d &origin,
                                   const Eigen::Isometry3d &current,
                                   const Eigen::Isometry3d &T_7a,
                                   const Eigen::Ref<const Eigen::Vector6d> &xd,
                                   const double angle,
                                   const double t,
                                   const double t_0,
                                   const double duration,
                                   const double kp = 2000,
                                   const double kv = 15);

    Eigen::Vector3d oneDofRotation(const Eigen::Isometry3d &origin,
                                   const Eigen::Isometry3d &current,
                                   const Eigen::Ref<const Eigen::Vector6d> &xd,
                                   const Eigen::Vector3d &m,
                                   const double speed, // + : positive, - : negative
                                   const double dir,    // 0 : x, 1 : y, 2 : z w.r.t flange frame
                                   const double t,
                                   const double t_0,
                                   const double kp = 2000,
                                   const double kv = 15);

    Eigen::Vector3d generatePartialSpiralEE(const Eigen::Isometry3d &origin,
                                            const Eigen::Isometry3d &current,
                                            const Eigen::Ref<const Eigen::Vector6d> &xd,
                                            const double pitch,
                                            const double lin_vel,
                                            const double force,
                                            const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                            const double search_dir,
                                            const double t,
                                            const double t_0,
                                            const double duration,
                                            const double kp = 700,
                                            const double kv = 15);

    Eigen::Vector3d generateForceToDisassembleEE(const Eigen::Isometry3d &origin,
                                                 const Eigen::Isometry3d &current,
                                                 const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                 const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                                 const double force,
                                                 const double t,
                                                 const double t_0,
                                                 const double duration);

    Eigen::Vector3d vibrationForce(const double f_max,
                                   const double t,
                                   const double t_0,
                                   const double duration);

    Eigen::Vector3d generateInsertionForceWithLessFriction(const Eigen::Isometry3d &origin,
                                                           const Eigen::Isometry3d &current,                                                           
                                                           const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                           const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                                           const Eigen::Vector3d &f_ext,
                                                           const Eigen::Vector3d &f_ext_error,
                                                           const double force,
                                                           const double t,
                                                           const double t_0,
                                                           const double duration,
                                                           const double kp = 0.15,
                                                           const double ki = 0.002,
                                                           const double kv = 15.0);

    Eigen::Vector3d generateMomentToDisassembleEE(const Eigen::Isometry3d &origin,
                                                  const Eigen::Isometry3d &current,
                                                  const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                  const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                                  const double momnet,
                                                  const double t,
                                                  const double t_0,
                                                  const double duration);

    Eigen::Vector6d generateRotationToDisassembleEE(const Eigen::Isometry3d &origin,
                                                    const Eigen::Isometry3d &current,
                                                    const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                    const Eigen::Ref<const Eigen::Vector6d> &f_ext,
                                                    const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                                    const double force,
                                                    const double moment,
                                                    const double angle,
                                                    const double t,
                                                    const double t_0,
                                                    const double duration,
                                                    const double kp = 700,
                                                    const double kv = 20,
                                                    const double kwp = 1000,
                                                    const double kwv = 7.5);

    double getSpiralTheta(const double p,
                          const double v,
                          const double t);
    // Eigen::Vector6d detachBoltForce(const Eigen::Isometry3d &origin,
    //                                 const Eigen::Isometry3d &current,
    //                                 const Eigen::Ref<const Eigen::Vector6d> &xd,
    //                                 const Eigen::Ref<const Eigen::Vector6d> &f_ext,
    //                                 const Eigen::Isometry3d &T_7a,
    //                                 const double force,
    //                                 const double angle,
    //                                 const double t,
    //                                 const double t_0,
    //                                 const double duration,
    //                                 const double kp = 700,
    //                                 const double kv = 20);

    // Eigen::Vector6d detachBoltMoment(const Eigen::Isometry3d &origin,
    //                                  const Eigen::Isometry3d &current,
    //                                  const Eigen::Ref<const Eigen::Vector6d> &xd,
    //                                  const Eigen::Ref<const Eigen::Vector6d> &f_ext,
    //                                  const Eigen::Isometry3d &T_7a,
    //                                  const double moment,
    //                                  const double angle,
    //                                  const double t,
    //                                  const double t_0,
    //                                  const double duration,
    //                                  const double kp = 1000,
    //                                  const double kv = 7.5);
}; // namespace PegInHole
