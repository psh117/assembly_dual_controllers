#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>

Eigen::Vector3d PegInHole::oneDofMoveEE(const Eigen::Isometry3d &origin,
                                        const Eigen::Isometry3d &current,
                                        const Eigen::Matrix<double, 6, 1> &xd,
                                        const Eigen::Isometry3d &T_ea,
                                        const double t,
                                        const double t_0,
                                        const double duration,
                                        const double target_distance, // + means go forward, - mean go backward
                                        const int direction)          // 0 -> x_ee, 1 -> y_ee, 2 -> z_ee //DESIRED DIRECTION W.R.T ASSEMBLY FRAME
{
  Eigen::Matrix3d K_p, K_v;
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a, v_cur_a;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d f_a;

  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

  T_wa = origin * T_ea; //T_WA

  p_init_a = T_ea.inverse().translation();

  for (int i = 0; i < 3; i++)
  {
    if (i == direction)
      p_cmd_a(direction) = dyros_math::cubic(t, t_0, t_0 + duration, 0.0, target_distance, 0.0, 0.0);
    else
      p_cmd_a(i) = 0.0;
  }

  p_cmd_a = p_init_a + p_cmd_a;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();
  p_cur_a = (T_wa.inverse() * current).translation();

  f_a = K_p * (p_cmd_a - p_cur_a) + K_v * (-v_cur_a); // <fx, fy, fz>
  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"duration: "<<duration<<std::endl;
  // std::cout<<"origin: "<<origin.translation().transpose()<<std::endl;
  // std::cout<<"current: "<<current.translation().transpose()<<std::endl;
  // std::cout<<"run time: "<<t - t_0<<std::endl;
  // std::cout<<"p_init_a: "<<p_init_a.transpose()<<std::endl;
  // std::cout<<"p_cmd_a: "<<p_cmd_a.transpose()<<std::endl;
  // std::cout<<"p_cur_a: "<<p_cur_a.transpose()<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;
  return f_a;
}

Eigen::Vector3d PegInHole::twoDofMoveEE(const Eigen::Isometry3d &origin,
                                        const Eigen::Isometry3d &current,
                                        const Eigen::Matrix<double, 6, 1> &xd,
                                        const Eigen::Isometry3d &T_ea,
                                        const double t,
                                        const double t_0,
                                        const double duration,
                                        const Eigen::Vector3d &target_position,
                                        const int direction) // 0 -> x, 1 -> y, 2 -> z //NOT DESIRED DIRECTION!!
{
  Eigen::Matrix3d K_p, K_v;
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a, v_cur_a;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d f_a;

  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

  T_wa = origin * T_ea; //T_WA

  p_init_a = T_ea.inverse().translation();

  for (int i = 0; i < 3; i++)
  {
    if (i == direction)
      p_cmd_a(i) = 0.0;
    else
      p_cmd_a(i) = dyros_math::cubic(t, t_0, t_0 + duration, 0.0, target_position(i), 0.0, 0.0);
  }

  p_cmd_a = p_init_a + p_cmd_a;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();
  p_cur_a = (T_wa.inverse() * current).translation();

  f_a = K_p * (p_cmd_a - p_cur_a) + K_v * (-v_cur_a); // <fx, fy, fz>

  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"duration: "<<duration<<std::endl;
  // std::cout<<"origin: "<<origin.translation().transpose()<<std::endl;
  // std::cout<<"current: "<<current.translation().transpose()<<std::endl;
  // std::cout<<"target: "<<target_position.transpose()<<std::endl;
  // std::cout<<"run time: "<<t - t_0<<std::endl;
  // std::cout<<"p_init_a: "<<p_init_a.transpose()<<std::endl;
  // std::cout<<"p_cmd_a: "<<p_cmd_a.transpose()<<std::endl;
  // std::cout<<"p_cur_a: "<<p_cur_a.transpose()<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;

  return f_a;
}

Eigen::Matrix<double, 6, 1> PegInHole::keepCurrentState(const Eigen::Vector3d initial_position,
                                                        const Eigen::Matrix3d initial_rotation,
                                                        const Eigen::Vector3d position,
                                                        const Eigen::Matrix3d rotation,
                                                        const Eigen::Matrix<double, 6, 1> current_velocity,
                                                        const double kp, const double kv)
{
  Eigen::Vector3d desired_position;
  Eigen::Vector3d desired_linear_velocity;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Vector6d f_star_zero;
  Eigen::Matrix3d kp_m;
  Eigen::Matrix3d kv_m;

  kp_m = Eigen::Matrix3d::Identity() * kp;
  kv_m = Eigen::Matrix3d::Identity() * kv;

  desired_position = initial_position;
  desired_linear_velocity.setZero();

  delphi_delta = -0.5 * dyros_math::getPhi(rotation, initial_rotation);

  f_star = kp_m * (desired_position - position) + kv_m * (desired_linear_velocity - current_velocity.head<3>());
  m_star = (1.0) * 300.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  return f_star_zero;
}
Eigen::Vector3d PegInHole::generateSpiral(const Eigen::Vector3d origin,
                                          const Eigen::Vector3d current_position,
                                          const Eigen::Matrix<double, 6, 1> current_velocity,
                                          const double pitch,
                                          const double lin_vel,
                                          const int dir, //the direction where a peg is inserted
                                          const double current_time,
                                          const double init_time,
                                          const double spiral_duration)
{
  // double pitch = 0.0010;
  // double lin_vel = 0.005;
  Eigen::Vector2d start_point;
  Eigen::Vector2d traj;
  Eigen::Vector3d desired_position;
  Eigen::Vector3d desired_linear_velocity;
  Eigen::Vector3d f_star;
  Eigen::Matrix3d K_p;
  Eigen::Matrix3d K_v;

  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

  if (dir == 0)
    start_point << origin(1), origin(2);
  if (dir == 1)
    start_point << origin(0), origin(2);
  if (dir == 2)
    start_point << origin(0), origin(1);

  traj = dyros_math::spiral(current_time, init_time, init_time + spiral_duration, start_point, lin_vel, pitch);

  if (dir == 0)
    desired_position << origin(dir), traj(0), traj(1);
  if (dir == 1)
    desired_position << traj(0), origin(dir), traj(1);
  if (dir == 2)
    desired_position << traj(0), traj(1), origin(dir);

  desired_linear_velocity.setZero();

  f_star = K_p * (desired_position - current_position) + K_v * (desired_linear_velocity - current_velocity.head<3>());

  // std::cout<<"desired_position: "<<desired_position.transpose()<<std::endl;
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  return f_star;
}

Eigen::Vector3d PegInHole::keepOrientationPerpenticular(const Eigen::Matrix3d initial_rotation_M,
                                                        const Eigen::Matrix3d rotation_M,
                                                        const Eigen::Matrix<double, 6, 1> current_velocity,
                                                        const double duration,
                                                        const double current_time,
                                                        const double init_time)
{
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;
  Eigen::Vector5d angle_set_45;
  Eigen::Vector5d angle_set_error;
  Eigen::Vector3d euler_angle;

  double val;
  double e;

  double roll, alpha;
  double pitch, beta;
  double yaw, gamma;

  double min;
  int index;

  euler_angle = dyros_math::rot2Euler(initial_rotation_M);
  roll = euler_angle(0);
  pitch = euler_angle(1);
  yaw = euler_angle(2);

  val = initial_rotation_M(2, 2);
  e = 1.0 - fabs(val);

  // angle_set_45 << -135, -45, 45, 135;
  angle_set_45 << -180.0, -90.0, 0.0, 90.0, 180.0;
  angle_set_45 = angle_set_45 * DEG2RAD;

  if (val > 0 && e <= 0.01) //upward
  {
    roll = 0;
    pitch = 0;

    for (size_t i = 0; i < 5; i++)
    {
      angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(2));
    }

    for (size_t i = 0; i < 5; i++)
    {
      if (angle_set_error(i) == angle_set_error.minCoeff())
        index = i;
    }

    yaw = angle_set_45(index);
  }
  else if (val < 0 && e <= 0.01) //downward
  {
    if (roll > 0)
      roll = 180 * DEG2RAD;
    else
      roll = -180 * DEG2RAD;

    pitch = 0;

    for (size_t i = 0; i < 5; i++)
    {
      angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(2));
    }
    for (size_t i = 0; i < 5; i++)
    {
      if (angle_set_error(i) == angle_set_error.minCoeff())
        index = i;
    }

    yaw = angle_set_45(index);
  }

  else //on xy plane
  {
    roll = euler_angle(0);
    yaw = euler_angle(2);

    for (size_t i = 0; i < 5; i++)
    {
      angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(1));
    }

    for (size_t i = 0; i < 5; i++)
    {
      if (angle_set_error(i) == angle_set_error.minCoeff())
        index = i;
    }

    pitch = angle_set_45(index);
  }

  alpha = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(0), roll, 0, 0);
  beta = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(1), pitch, 0, 0);
  gamma = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(2), yaw, 0, 0);

  target_rotation_M = dyros_math::rotateWithZ(gamma) * dyros_math::rotateWithY(beta) * dyros_math::rotateWithX(alpha);

  delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * 200.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

  return m_star;
}

Eigen::Vector3d PegInHole::keepOrientationPerpenticularOnlyXY(const Eigen::Matrix3d initial_rotation_M,
                                                              const Eigen::Matrix3d rotation_M,
                                                              const Eigen::Matrix<double, 6, 1> current_velocity,
                                                              const double duration,
                                                              const double current_time,
                                                              const double init_time)
{
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;
  Eigen::Vector3d euler_angle;

  double val;
  double e;

  double roll, alpha;
  double pitch, beta;
  double yaw, gamma;

  euler_angle = dyros_math::rot2Euler(initial_rotation_M);
  roll = euler_angle(0);
  pitch = euler_angle(1);
  yaw = euler_angle(2);

  val = initial_rotation_M(2, 2);
  e = 1.0 - fabs(val);

  if (val > 0) //upward
  {
    roll = 0;
    pitch = 0;
  }

  else //downward
  {
    if (roll > 0)
      roll = 180 * DEG2RAD;
    else
      roll = -180 * DEG2RAD;
    pitch = 0;
  }

  alpha = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(0), roll, 0, 0);
  beta = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(1), pitch, 0, 0);
  gamma = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(2), yaw, 0, 0);

  target_rotation_M = dyros_math::rotateWithZ(gamma) * dyros_math::rotateWithY(beta) * dyros_math::rotateWithX(alpha);

  delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * 250.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

  return m_star;
}

Eigen::Vector3d PegInHole::rotateOrientationPerpenticular(const Eigen::Matrix3d initial_rotation_M,
                                                          const Eigen::Matrix3d rotation_M,
                                                          const Eigen::Matrix<double, 6, 1> current_velocity,
                                                          const double duration,
                                                          const double current_time,
                                                          const double init_time)
{
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;
  Eigen::Vector3d euler_angle;

  double val;
  double e;

  double roll, alpha;
  double pitch, beta;
  double yaw, gamma;

  euler_angle = dyros_math::rot2Euler(initial_rotation_M);
  roll = euler_angle(0);
  pitch = euler_angle(1);
  yaw = euler_angle(2);

  val = initial_rotation_M(2, 2);
  e = 1.0 - fabs(val);

  if (val > 0) //upward
  {
    roll = 0;
    pitch = 0;
  }

  else //downward
  {
    if (roll > 0)
      roll = 180 * DEG2RAD;
    else
      roll = -180 * DEG2RAD;
    pitch = 0;
  }

  alpha = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(0), roll, 0, 0);
  beta = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(1), pitch + (3 * M_PI / 180), 0, 0);
  gamma = dyros_math::cubic(current_time, init_time, init_time + duration, euler_angle(2), yaw, 0, 0);

  target_rotation_M = dyros_math::rotateWithZ(gamma) * dyros_math::rotateWithY(beta) * dyros_math::rotateWithX(alpha);

  delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * 250.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

  return m_star;
}

double PegInHole::calculateFriction(const int dir, const Eigen::Vector3d f_measured, double friction)
{
  if (dir == 0)
    friction = sqrt(f_measured(1) * f_measured(1) + f_measured(2) * f_measured(2));
  if (dir == 1)
    friction = sqrt(f_measured(0) * f_measured(0) + f_measured(2) * f_measured(2));
  if (dir == 2)
    friction = sqrt(f_measured(0) * f_measured(0) + f_measured(1) * f_measured(1));

  return friction;
}

Eigen::Vector3d PegInHole::computeNomalVector(const Eigen::Vector3d p1,
                                              const Eigen::Vector3d p2,
                                              const Eigen::Vector3d p3)
{
  Eigen::Vector3d r1;
  Eigen::Vector3d r2;
  Eigen::Vector3d r3;
  Eigen::Vector3d result;

  r1 = p1 - p2;
  r2 = p1 - p3;

  r3 = r1.cross(r2);
  r3 = r3 / sqrt(pow(r3(0), 2) + pow(r3(1), 2) + pow(r3(2), 2));

  result = r3;
  return result;
}

Eigen::Vector3d PegInHole::getTiltDirection(const Eigen::Isometry3d T_ea)
{
  Eigen::Isometry3d T_ae;
  Eigen::Vector3d p;      //position from {A} to {E}
  Eigen::Vector3d o1, o2; // to determin the rotation direction
  Eigen::Matrix3d rot1, rot2;
  Eigen::Vector3d asm_dir;   //always z axis wrt {A}
  Eigen::Vector3d tilt_axis; // return the vector wrt {A}
  double theta;

  theta = 10 * 180 / M_PI;

  asm_dir = Eigen::Vector3d::UnitZ();

  T_ae = T_ea.inverse();

  p = T_ae.translation();

  tilt_axis = asm_dir.cross(p);
  tilt_axis.normalize();

  rot1 = dyros_math::angleaxis2rot(tilt_axis, theta);
  rot2 = dyros_math::angleaxis2rot(-tilt_axis, theta);

  o1 = rot1 * p;
  o2 = rot2 * p;

  // if (abs(o1(2)) >= abs(o2(2)))
  //   tilt_axis = tilt_axis;
  // else
  //   tilt_axis = -tilt_axis;

  if (o1(2) < 0)
    tilt_axis = tilt_axis;

  std::cout << "w.r.t {A}" << std::endl;
  std::cout << "o1: " << o1.transpose() << std::endl;
  std::cout << "o2: " << o2.transpose() << std::endl;
  return tilt_axis;
}

bool PegInHole::setTilt(const Eigen::Isometry3d T_ea, const double threshold)
{
  Eigen::Vector3d proj_vector;
  Eigen::Vector3d p;
  Eigen::Vector3d asm_dir;
  double dis;
  bool set_tilt;

  dis = 0.0;
  p = T_ea.translation();
  asm_dir = T_ea.linear().col(2);

  for (int i = 0; i < 3; i++)
  {
    proj_vector(i) = 1.0 - round(abs(asm_dir(i)));
    double temp = proj_vector(i) * p(i);
    dis += pow(temp, 2);
  }

  dis = sqrt(dis);

  if (dis >= threshold)
    set_tilt = true;
  else
    set_tilt = false;

  // std::cout<<"asm dir: "<<asm_dir.transpose()<<std::endl;
  // std::cout<<"proj vector: "<<proj_vector.transpose()<<std::endl;
  // std::cout<<"p: "<<p.transpose()<<std::endl;
  // std::cout<<"dis: "<<dis<<" threshold: "<<threshold<<std::endl;
  return set_tilt;
}

Eigen::Vector6d PegInHole::tiltMotion(const Eigen::Isometry3d origin, const Eigen::Isometry3d current,
                                      const Eigen::Ref<const Eigen::Vector6d> &xd, const Eigen::Isometry3d T_ea,
                                      const Eigen::Vector3d tilt_axis, //wrt {A}
                                      const double tilt_angle,
                                      const double t, const double t_0, const double duration)
{
  Eigen::Vector3d pos_init;
  Eigen::Matrix3d ori_init;
  Eigen::Vector3d f_star, m_star;
  Eigen::Vector6d f_star_zero;
  Eigen::Matrix3d K_p, K_v;
  Eigen::Vector3d p0_a, p_a, p_e; //wrt {A}
  Eigen::Vector3d target_position, desired_position, position;
  Eigen::Matrix3d R_e;
  Eigen::Matrix3d target_rotation, desired_rotation, rotation;
  Eigen::Vector3d delphi_delta;

  p0_a = T_ea.inverse().translation(); //location of {E} wrt {A}
  p_a = dyros_math::angleaxis2rot(tilt_axis, tilt_angle) * p0_a;
  p_e = T_ea.linear() * p_a + T_ea.translation();
  R_e = dyros_math::angleaxis2rot(T_ea.linear() * tilt_axis, tilt_angle);

  K_p << 3000, 0, 0, 0, 3000, 0, 0, 0, 3000;
  K_v << 100, 0, 0, 0, 100, 0, 0, 0, 100;

  pos_init = origin.translation();
  position = current.translation();
  target_position = origin.linear() * p_e + origin.translation();
  for (int i = 0; i < 3; i++)
  {
    desired_position(i) = dyros_math::cubic(t, t_0, t_0 + duration, pos_init(i), target_position(i), 0.0, 0.0);
  }

  ori_init = origin.linear();
  rotation = current.linear();
  target_rotation = ori_init * R_e;
  desired_rotation = dyros_math::rotationCubic(t, t_0, t_0 + duration, ori_init, target_rotation);

  delphi_delta = -0.5 * dyros_math::getPhi(rotation, desired_rotation);

  f_star = K_p * (desired_position - position) + K_v * (-xd.head<3>());
  m_star = (1.0) * 200.0 * delphi_delta + 5.0 * (-xd.tail<3>());

  f_star_zero << f_star, m_star;

  // std::cout<<"-------------------------------"<<std::endl;
  // std::cout<<"tilt axis: "<<tilt_axis.transpose()<<std::endl;
  // std::cout<<"p0_a: "<<p0_a.transpose()<<std::endl;
  // std::cout<<"p_a: "<<p_a.transpose()<<std::endl;
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  // std::cout<<"pos_init: "<<pos_init.transpose()<<std::endl;
  // std::cout<<"desired_position: "<<desired_position.transpose()<<std::endl;
  // std::cout<<"position: "<<position.transpose()<<std::endl;
  // std::cout<<"targe_position: "<<target_position.transpose()<<std::endl;
  // std::cout<<"target rotation: \n"<<target_rotation<<std::endl;

  return f_star_zero;
}

// bool::judgeHeavyMass(const double t, const double )

Eigen::Vector3d PegInHole::straightMotionEE(const Eigen::Isometry3d &origin,
                                            const Eigen::Isometry3d &current,
                                            const Eigen::Ref<const Eigen::Vector6d> &xd,
                                            const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                            const Eigen::Vector3d &target_dir,
                                            const double speed,
                                            const double t,
                                            const double t_0)
{
  Eigen::Matrix3d K_p, K_v;
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a, v_cmd_a, v_cur_a;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d f_a;

  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

  T_wa = origin * T_ea; //T_WA

  p_init_a = T_ea.inverse().translation();

  v_cmd_a = speed * target_dir;
  p_cmd_a = p_init_a + speed * (t - t_0) * target_dir;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();
  p_cur_a = (T_wa.inverse() * current).translation();

  f_a = K_p * (p_cmd_a - p_cur_a) + K_v * (v_cmd_a - v_cur_a); // <fx, fy, fz>

  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"origin: "<<origin.translation().transpose()<<std::endl;
  // std::cout<<"current: "<<current.translation().transpose()<<std::endl;
  // std::cout<<"run time: "<<t - t_0<<std::endl;
  // std::cout<<"p_init_a: "<<p_init_a.transpose()<<std::endl;
  // std::cout<<"p_cmd_a: "<<p_cmd_a.transpose()<<std::endl;
  // std::cout<<"p_cur_a: "<<p_cur_a.transpose()<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;

  return f_a;
}

Eigen::Vector3d PegInHole::approachComponentEE(const Eigen::Isometry3d &origin,
                                               const Eigen::Isometry3d &current,
                                               const Eigen::Ref<const Eigen::Vector6d> &xd,
                                               const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                               const double speed,
                                               const double t,
                                               const double t_0)
{
  Eigen::Vector3d asm_dir_a;
  Eigen::Matrix3d K_p, K_v;
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a, v_cmd_a, v_cur_a;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d f_a;

  T_wa = origin * T_ea; //T_WA

  p_init_a = T_ea.inverse().translation();

  asm_dir_a << 0.0, 0.0, 1.0; // always z - axis wrt {A}

  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 100, 0, 0, 0, 100, 0, 0, 0, 100;

  v_cmd_a = speed * asm_dir_a;

  p_cmd_a = p_init_a + speed * (t - t_0) * asm_dir_a;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();
  p_cur_a = (T_wa.inverse() * current).translation();

  f_a = K_p * (p_cmd_a - p_cur_a) + K_v * (v_cmd_a - v_cur_a); // <fx, fy, fz>

  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"origin: "<<origin.translation().transpose()<<std::endl;
  // std::cout<<"current: "<<current.translation().transpose()<<std::endl;
  // std::cout<<"run time: "<<t - t_0<<std::endl;
  // std::cout<<"p_init_a: "<<p_init_a.transpose()<<std::endl;
  // std::cout<<"p_cmd_a: "<<p_cmd_a.transpose()<<std::endl;
  // std::cout<<"p_cur_a: "<<p_cur_a.transpose()<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;

  return f_a;
}

//Edited by KimHC 20200623 - additional movement to XY direction
Eigen::Vector3d PegInHole::pinMoveEE(const Eigen::Isometry3d &origin,
                                     const Eigen::Isometry3d &current,
                                     const Eigen::Ref<const Eigen::Vector6d> &xd,
                                     const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                     const Eigen::Isometry3d &T_ad,
                                     const double speed,
                                     const double t,
                                     const double t_0)
{
  Eigen::Vector3d asm_dir_a;
  Eigen::Matrix3d K_p, K_v;
  Eigen::Vector3d p_init_a, goal_ad;
  Eigen::Vector3d p_cmd_a, p_cur_a, v_cmd_a, v_cur_a, p_cmd_a_test;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d f_a;
  double traj_x, traj_y;
  T_wa = origin * T_ea;
  p_init_a = T_ea.inverse().translation();

  asm_dir_a << 0.0, 0.0, 1.0;
  goal_ad = T_ad.translation();

  // no changes made starting from here
  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 2500;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 50;

  traj_x = dyros_math::cubic(t, t_0, t_0 + 1.0, 0.0, goal_ad(0), 0.0, 0.0);
  traj_y = dyros_math::cubic(t, t_0, t_0 + 1.0, 0.0, goal_ad(1), 0.0, 0.0);

  v_cmd_a = speed * asm_dir_a;
  p_cmd_a_test << p_init_a(0) + traj_x, p_init_a(1) + traj_y, p_init_a(2) + speed * (t - t_0) * asm_dir_a(2);

  p_cmd_a = p_init_a + speed * (t - t_0) * asm_dir_a;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();
  p_cur_a = (T_wa.inverse() * current).translation();

  // f_a = K_p * (p_cmd_a - p_cur_a) + K_v * (v_cmd_a - v_cur_a); // <fx, fy, fz>
  f_a = K_p * (p_cmd_a_test - p_cur_a) + K_v * (v_cmd_a - v_cur_a); // <fx, fy, fz>

  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"run time: "<<t - t_0<<std::endl;
  // std::cout<<"goal_ad: "<<goal_ad.transpose()<<std::endl;
  // std::cout<<"p_cmd_a_test: "<<p_cmd_a_test.transpose()<<std::endl;
  // std::cout<<"p_cmd_a: "<<p_cmd_a.transpose()<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;

  return f_a;
}

Eigen::Vector3d PegInHole::threeDofMoveEE(const Eigen::Isometry3d &origin,
                                          const Eigen::Isometry3d &current,
                                          const Eigen::Vector3d &target,
                                          const Eigen::Ref<const Eigen::Vector6d> &xd,
                                          const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                          const double t,
                                          const double t_0,
                                          const double duration)
{
  Eigen::Vector3d asm_dir_a;
  Eigen::Matrix3d K_p, K_v;
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a, v_cur_a;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d f_a;

  K_p << 3000, 0, 0, 0, 3000, 0, 0, 0, 3000;
  K_v << 100, 0, 0, 0, 100, 0, 0, 0, 100;

  T_wa = origin * T_ea;
  p_init_a = T_ea.inverse().translation();

  for (int i = 0; i < 3; i++)
    p_cmd_a(i) = dyros_math::cubic(t, t_0, t_0 + duration, 0.0, target(i), 0.0, 0.0);
  p_cmd_a = p_init_a + p_cmd_a;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();
  p_cur_a = (T_wa.inverse() * current).translation();

  f_a = K_p * (p_cmd_a - p_cur_a) + K_v * (-v_cur_a); // <fx, fy, fz>

  return f_a;
}

Eigen::Vector3d PegInHole::threeDofMove(const Eigen::Isometry3d &origin,
                                        const Eigen::Isometry3d &current,
                                        const Eigen::Vector3d &target,
                                        const Eigen::Ref<const Eigen::Vector6d> &xd,
                                        const double t,
                                        const double t_0,
                                        const double duration)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d desired_position;
  Eigen::Matrix3d K_p, K_v;

  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

  for (int i = 0; i < 3; i++)
  {
    desired_position(i) = dyros_math::cubic(t, t_0, t_0 + duration, origin.translation()(i), target(i), 0.0, 0.0);
  }

  f_star = K_p * (desired_position - current.translation()) + K_v * (-xd.head<3>());

  // std::cout<<"-----------------"<<std::endl;
  // std::cout<<"origin: "<<origin.translation().transpose()<<std::endl;
  // std::cout<<"current: "<<current.translation().transpose()<<std::endl;
  // std::cout<<"target: "<<target.transpose()<<std::endl;
  return f_star;
}

Eigen::Vector6d PegInHole::keepCurrentPose(const Eigen::Isometry3d &origin,
                                           const Eigen::Isometry3d &current,
                                           const Eigen::Ref<const Eigen::Vector6d> &xd,
                                           const double kp,
                                           const double kv,
                                           const double kp_o,
                                           const double kv_o,
                                           const Eigen::Ref<const Eigen::Matrix6d> &lambda)
{
  auto f_star = keepCurrentPosition(origin, current, xd, kp, kv);
  auto m_star = keepCurrentOrientation(origin, current, xd, kp_o, kv_o);

  Eigen::Vector6d result;
  // std::cout <<"fstar: " << f_star.transpose()  << std::endl;
  // std::cout <<"mstar: " << m_star.transpose()  << std::endl;
  result << f_star, m_star;
  // std::cout <<"mstar: " << result.transpose() << std::endl;
  result = lambda * result;
  // std::cout << "lambda: " << std::endl << lambda << std::endl;

  return result;
}
Eigen::Vector3d PegInHole::keepCurrentOrientation(const Eigen::Isometry3d &origin,
                                                  const Eigen::Isometry3d &current,
                                                  const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                  const double kp, const double kv)
{
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;
  Eigen::Matrix3d init_rot, rotation;

  init_rot = origin.linear();
  rotation = current.linear();

  delphi_delta = -0.5 * dyros_math::getPhi(rotation, init_rot);

  m_star = kp * delphi_delta + kv * (-xd.tail<3>());

  return m_star;
}

Eigen::Vector3d PegInHole::keepCurrentPosition(const Eigen::Isometry3d &origin,
                                               const Eigen::Isometry3d &current,
                                               const Eigen::Ref<const Eigen::Vector6d> &xd,
                                               const double kp, const double kv)
{
  Eigen::Vector3d f_star;

  Eigen::Matrix3d kp_m;
  Eigen::Matrix3d kv_m;

  kp_m = Eigen::Matrix3d::Identity() * kp;
  kv_m = Eigen::Matrix3d::Identity() * kv;

  f_star = kp_m * (origin.translation() - current.translation()) + kv_m * (-xd.head<3>());

  return f_star;
}

Eigen::Vector3d PegInHole::pressEE(const double force)

{
  Eigen::Vector3d asm_dir, f_a;

  asm_dir = Eigen::Vector3d::UnitZ();
  f_a = asm_dir * force;

  return f_a;
}

Eigen::Vector6d PegInHole::generateTwistSpiralEE(const Eigen::Isometry3d &origin,
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
                                                 const double revolve_duration)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d pos, pos_target, v_vel, w_vel;
  Eigen::Matrix3d rot, rot_target;
  Eigen::Vector3d delphi_delta; // P_ae
  Eigen::Vector3d f_a, m_a;     // end-effector wrench w.r.t {A}
  Eigen::Vector6d f_star_zero_a;
  Eigen::Matrix3d K_p, K_v;

  double t_e;
  double w;
  double theta, phi;
  double dis;
  double offset_angle;

  K_p = Eigen::Matrix3d::Identity() * 20000;
  K_v = Eigen::Matrix3d::Identity() * 200;

  T_wa = origin * T_ea; //T_WA

  t_e = t - t_0;
  w = 2 * M_PI / revolve_duration;
  theta = angle * sin(w * t_e);

  Eigen::Vector3d pos_Aa;
  Eigen::Matrix3d rot_Aa;
  Eigen::Isometry3d T_Aa, T_a7, T_A7;

  pos_Aa.head<2>() = dyros_math::spiral(t, t_0, t_0 + spiral_duration, Eigen::Vector2d::Zero(), lin_vel, pitch);
  pos_Aa(2) = 0.0;
  rot_Aa = dyros_math::rotateWithZ(theta);

  T_Aa.linear() = rot_Aa;
  T_Aa.translation() = pos_Aa;
  // T_A7 = T_ea.inverse();

  T_A7 = T_Aa * T_ea.inverse(); // cmd for {E} w.r.t {A}

  pos_target = T_A7.translation();
  pos = (T_wa.inverse() * current).translation(); // {E} w.r.t {A}
  v_vel = T_wa.linear().transpose() * xd.head<3>();

  rot_target = T_A7.linear();
  rot = T_wa.linear().inverse() * current.linear();
  w_vel = T_wa.linear().transpose() * xd.tail<3>();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, rot_target);

  f_a = K_p * (pos_target - pos) + K_v * (-v_vel);
  m_a = 1000.0 * delphi_delta + 5.0 * (-w_vel);

  f_star_zero_a << f_a, m_a;

  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"t_e: "<< t_e<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;
  // std::cout<<"m_a: "<<m_a.transpose()<<std::endl;
  // std::cout<<"theta: "<< theta*RAD2DEG<<std::endl;
  // std::cout<<"phi: "<< phi*RAD2DEG<<std::endl;
  // std::cout<<"pos_target: "<< pos_target.transpose()<<std::endl;
  // std::cout<<"pos: "<< pos.transpose()<<std::endl;

  return f_star_zero_a;
}

Eigen::Vector6d PegInHole::generateTwistSpiralEE_savedata(const Eigen::Isometry3d &origin,
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
                                                          const double revolve_duration)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d pos_Aa_cmd, pos_Aa_cur;
  Eigen::Matrix3d rot_Aa_cmd, rot_Aa_cur;
  Eigen::Isometry3d T_Aa_cmd, T_A7_cmd, T_Aa_cur;
  Eigen::Vector6d pos_save;

  double t_e;
  double w;
  double theta, phi;

  T_wa = origin * T_ea; //T_WA

  t_e = t - t_0;
  w = 2 * M_PI / revolve_duration;
  theta = angle * sin(w * t_e);

  pos_Aa_cmd.head<2>() = dyros_math::spiral(t, t_0, t_0 + spiral_duration, Eigen::Vector2d::Zero(), lin_vel, pitch);
  pos_Aa_cmd(2) = 0.0;
  rot_Aa_cmd = dyros_math::rotateWithZ(theta);

  T_Aa_cmd.linear() = rot_Aa_cmd;
  T_Aa_cmd.translation() = pos_Aa_cmd;
  T_A7_cmd = T_Aa_cmd * T_ea.inverse(); // cmd for {E} w.r.t {A}

  T_Aa_cur = T_wa.inverse() * current * T_ea;
  pos_Aa_cur = T_Aa_cur.translation();
  rot_Aa_cur = T_Aa_cur.linear();

  phi = atan2(rot_Aa_cur(1, 0), rot_Aa_cur(0, 0));

  pos_save << pos_Aa_cmd.head<2>(), theta, pos_Aa_cur.head<2>(), phi;

  return pos_save;
}

Eigen::Vector3d PegInHole::generateSpiralEE(const Eigen::Isometry3d &origin,
                                            const Eigen::Isometry3d &current,
                                            const Eigen::Ref<const Eigen::Vector6d> &xd,
                                            const double pitch,
                                            const double lin_vel,
                                            const double force,
                                            const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                            const double t,
                                            const double t_0,
                                            const double duration)
{
  Eigen::Vector2d start_point, traj;
  Eigen::Vector3d f_a, f_motion, f_asm; //wrt {A}
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a, p_cur_w, p_cmd_w;
  Eigen::Vector3d v_cur_a;

  Eigen::Matrix3d K_p, K_v;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d asm_dir;

  K_p = Eigen::Matrix3d::Identity() * 3000;
  K_v = Eigen::Matrix3d::Identity() * 100;
  asm_dir = T_ea.linear().col(2); //w.r.t {E}

  start_point.setZero();

  T_wa = origin * T_ea; //T_WA

  traj = dyros_math::spiral(t, t_0, t_0 + duration, start_point, lin_vel, pitch);

  p_init_a = T_ea.inverse().translation();

  p_cmd_a << traj(0), traj(1), 0.0; //w.r.t {A}
  p_cmd_a = p_init_a + p_cmd_a;

  // p_cur_w = (current * T_ea).translation() - T_wa.translation(); // T_we * T_ea - p_wa
  // p_cur_a = T_wa.linear().transpose() * p_cur_w;
  p_cur_a = (T_wa.inverse() * current).translation(); // {E} w.r.t {A}

  // p_cmd_w = T_wa*p_cmd_a;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();

  f_motion = K_p * (p_cmd_a - p_cur_a) + K_v * (-v_cur_a); // <fx, fy, fz>
  f_asm << 0.0, 0.0, force;                                //<0, 0, fz>

  f_a(0) = f_motion(0);
  f_a(1) = f_motion(1);
  f_a(2) = f_asm(2);

  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"origin: "<<origin.translation().transpose()<<std::endl;
  // std::cout<<"current: "<<current.translation().transpose()<<std::endl;
  // std::cout<<"run time: "<<t - t_0<<std::endl;
  // std::cout<<"p_init_a: "<<p_init_a.transpose()<<std::endl;
  // std::cout<<"p_cmd_a: "<<p_cmd_a.transpose()<<std::endl;
  // std::cout<<"p_cur_a: "<<p_cur_a.transpose()<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;

  return f_a;
}

Eigen::Vector6d PegInHole::generateSpiralEE_datasave(const Eigen::Isometry3d &origin,
                                                     const Eigen::Isometry3d &current,
                                                     const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                     const double pitch,
                                                     const double lin_vel,
                                                     const double force,
                                                     const Eigen::Isometry3d &T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                                     const double t,
                                                     const double t_0,
                                                     const double duration)
{
  Eigen::Vector2d start_point, traj;
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a;
  Eigen::Vector6d pos_save;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d asm_dir;

  asm_dir = T_ea.linear().col(2); //w.r.t {E}

  start_point.setZero();

  T_wa = origin * T_ea; //T_WA

  traj = dyros_math::spiral(t, t_0, t_0 + duration, start_point, lin_vel, pitch);

  p_init_a = T_ea.inverse().translation();

  p_cmd_a << traj(0), traj(1), 0.0; //w.r.t {A}
  p_cur_a = (T_ea.inverse() * ((T_wa.inverse() * current)).inverse()).translation();

  pos_save << p_cmd_a, p_cur_a;

  return pos_save;
}

Eigen::Vector3d PegInHole::rotateWithAseemblyDir(const Eigen::Isometry3d &origin,
                                                 const Eigen::Isometry3d &current,
                                                 const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                 const Eigen::Isometry3d T_ea,
                                                 const double target_angle,
                                                 const double t,
                                                 const double t_0,
                                                 const double duration) //axis_vector wrt EE frame
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d m_a;
  Eigen::Vector3d axis;
  Eigen::Vector3d delphi_delta;
  Eigen::Matrix3d rot_init, rot, target_rot, desired_rot;
  Eigen::Vector3d v_cur_w, v_cur_a;

  T_wa = origin * T_ea;

  axis = Eigen::Vector3d::UnitZ();

  rot_init = T_wa.linear().transpose() * origin.linear();
  // rot_init = Eigen::Matrix3d::Identity();
  target_rot = dyros_math::angleaxis2rot(axis, target_angle) * rot_init; //wrt {A}
  desired_rot = dyros_math::rotationCubic(t, t_0, t_0 + duration, rot_init, target_rot);
  rot = T_wa.linear().transpose() * current.linear();

  // v_cur_w = xd.tail<3>();

  // v_cur_a = T_wa.linear().transpose()*v_cur_w*T_ea.linear();
  v_cur_a = T_wa.linear().transpose() * xd.tail<3>();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, desired_rot);

  m_a = (1.0) * 200.0 * delphi_delta + 5.0 * (-v_cur_a);

  // Eigen::Vector3d euler_init, euler, euler_goal, euler_d;

  // euler_init = dyros_math::rot2Euler(rot_init);
  // euler = dyros_math::rot2Euler(rot);
  // euler_d = dyros_math::rot2Euler(desired_rot);
  // euler_goal = dyros_math::rot2Euler(target_rot);

  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"rot_init: \n"<<rot_init<<std::endl;
  // std::cout<<"run time: "<<t - t_0<< std::endl;
  // std::cout<<"axis: "<<axis.transpose()<<std::endl;
  // std::cout<<"euler_init: "<<euler_init.transpose()*180/M_PI<<std::endl;
  // std::cout<<"euler_goal: "<<euler_goal.transpose()*180/M_PI<<std::endl;
  // std::cout<<"euler_d: "<<euler_d.transpose()*180/M_PI<<std::endl;
  // std::cout<<"euler: "<<euler.transpose()*180/M_PI<<std::endl;
  // std::cout<<"m_a: "<<m_a.transpose()<<std::endl;

  return m_a;
}

Eigen::Vector3d PegInHole::rotateWithMat(const Eigen::Isometry3d &origin,
                                         const Eigen::Isometry3d &current,
                                         const Eigen::Ref<const Eigen::Vector6d> &xd,
                                         const Eigen::Matrix3d target_rot, //R_we
                                         const double t,
                                         const double t_0,
                                         const double duration) //axis_vector wrt EE frame
{
  Eigen::Vector3d m_star;
  Eigen::Vector3d delphi_delta;
  Eigen::Matrix3d rot_init, rot, desired_rot;

  rot_init = origin.linear();
  desired_rot = dyros_math::rotationCubic(t, t_0, t_0 + duration, rot_init, target_rot);
  rot = current.linear();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, desired_rot);

  m_star = (1.0) * 300.0 * delphi_delta + 5.0 * (-xd.tail<3>());

  return m_star;
}

Eigen::Vector3d PegInHole::generateWiggleMotionEE(const Eigen::Isometry3d &origin,
                                                  const Eigen::Isometry3d &current,
                                                  const Eigen::Isometry3d &T_ea,
                                                  const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                  const double angle,
                                                  const double w, //rad/s
                                                  const double t,
                                                  const double t_0)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d x_axis, y_axis, z_axis;
  Eigen::Vector3d m_a, delphi_delta;
  Eigen::Matrix3d rot_init, rot;
  Eigen::Matrix3d target_rot, target_rot_x, target_rot_y, target_rot_z;
  Eigen::Vector3d v_cur_w, v_cur_a;

  T_wa = origin * T_ea;

  x_axis = Eigen::Vector3d::UnitX();
  y_axis = Eigen::Vector3d::UnitY();

  double t_e;
  double theta1, theta2;

  t_e = t - t_0;
  theta1 = angle * sin(w * t_e);
  theta2 = angle * cos(w * t_e);

  rot_init = T_wa.linear().transpose() * origin.linear(); //initial {E} wrt {A}

  // target_rot_x = dyros_math::angleaxis2rot(x_axis, theta1) * rot_init; //wrt {A}
  // target_rot_y = dyros_math::angleaxis2rot(y_axis, theta2) * rot_init; //wrt {A}
  target_rot_x = dyros_math::angleaxis2rot(x_axis, theta1); //wrt {A}
  target_rot_y = dyros_math::angleaxis2rot(y_axis, theta2); //wrt {A}
  target_rot = target_rot_x * target_rot_y * rot_init;

  rot = T_wa.linear().transpose() * current.linear();

  v_cur_a = T_wa.linear().transpose() * xd.tail<3>();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, target_rot);

  m_a = (1.0) * 25.0 * delphi_delta + 0.01 * (-v_cur_a);

  Eigen::Vector3d euler_init, euler, euler_goal;

  euler_init = dyros_math::rot2Euler(rot_init);
  euler = dyros_math::rot2Euler(rot);
  euler_goal = dyros_math::rot2Euler(target_rot);

  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"rot_init: \n"<<rot_init<<std::endl;
  // std::cout<<"run time: "<<t - t_0<< std::endl;
  // std::cout<<"euler_init: "<<euler_init.transpose()*180/M_PI<<std::endl;
  // std::cout<<"euler_goal: "<<euler_goal.transpose()*180/M_PI<<std::endl;
  // std::cout<<"euler: "<<euler.transpose()*180/M_PI<<std::endl;
  // std::cout<<"m_a_wiggle: "<<m_a.transpose()<<std::endl;

  return m_a;
}

Eigen::Vector3d PegInHole::generateWiggleMotionEE_Zaxis(const Eigen::Isometry3d &origin,
                                                        const Eigen::Isometry3d &current,
                                                        const Eigen::Isometry3d &T_ea,
                                                        const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                        const double angle,
                                                        const double w, //rad/s
                                                        const double t,
                                                        const double t_0)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d z_axis;
  Eigen::Vector3d m_a, delphi_delta;
  Eigen::Matrix3d rot_init, rot;
  Eigen::Matrix3d target_rot, target_rot_z;
  Eigen::Vector3d v_cur_w, v_cur_a;

  T_wa = origin * T_ea;

  z_axis = Eigen::Vector3d::UnitZ();
  double t_e;
  double theta;

  t_e = t - t_0;
  theta = angle * sin(w * t_e);
  
  rot_init = T_wa.linear().transpose() * origin.linear(); //initial {E} wrt {A}
  target_rot_z = dyros_math::angleaxis2rot(z_axis, theta); //wrt {A}  
  target_rot = target_rot_z * rot_init;
  rot = T_wa.linear().transpose() * current.linear();

  v_cur_a = T_wa.linear().transpose() * xd.tail<3>();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, target_rot);

  m_a = (1.0) * 50.0 * delphi_delta + 0.01 * (-v_cur_a);

  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"rot_init: \n"<<rot_init<<std::endl;
  // std::cout<<"run time: "<<t - t_0<< std::endl;
  // std::cout<<"euler_init: "<<euler_init.transpose()*180/M_PI<<std::endl;
  // std::cout<<"euler_goal: "<<euler_goal.transpose()*180/M_PI<<std::endl;
  // std::cout<<"euler: "<<euler.transpose()*180/M_PI<<std::endl;
  // std::cout<<"m_a_wiggle: "<<m_a.transpose()<<std::endl;

  return m_a;
}

Eigen::Vector3d PegInHole::generateYawingMotionEE(const Eigen::Isometry3d &origin,
                                                  const Eigen::Isometry3d &current,
                                                  const Eigen::Isometry3d &T_ea,
                                                  const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                  const double yawing_angle,
                                                  const double duration,
                                                  const double t,
                                                  const double t_0)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d z_axis;
  Eigen::Vector3d m_a, delphi_delta;
  Eigen::Matrix3d rot_init, rot;
  Eigen::Matrix3d target_rot;
  Eigen::Vector3d v_cur_a;
  double theta, t_f;
  t_f = t_0 + duration;

  T_wa = origin * T_ea;
  z_axis = Eigen::Vector3d::UnitZ();
  theta = dyros_math::cubic(t, t_0, t_f, 0.0, yawing_angle, 0., 0.);

  rot_init = T_wa.linear().transpose() * origin.linear(); //initial {E} wrt {A}

  target_rot = dyros_math::angleaxis2rot(z_axis, theta) * rot_init;

  rot = T_wa.linear().transpose() * current.linear();

  v_cur_a = T_wa.linear().transpose() * xd.tail<3>();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, target_rot);

  m_a = (1.0) * 250.0 * delphi_delta + 5.0 * (-v_cur_a);

  Eigen::Vector3d euler_init, euler, euler_goal;

  euler_init = dyros_math::rot2Euler(rot_init);
  euler = dyros_math::rot2Euler(rot);
  euler_goal = dyros_math::rot2Euler(target_rot);

  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"rot_init: \n"<<rot_init<<std::endl;
  // std::cout<<"run time: "<<t - t_0<< std::endl;
  // std::cout<<"euler_init: "<<euler_init.transpose()*180/M_PI<<std::endl;
  // std::cout<<"euler_goal: "<<euler_goal.transpose()*180/M_PI<<std::endl;
  // std::cout<<"euler: "<<euler.transpose()*180/M_PI<<std::endl;
  // std::cout<<"m_a: "<<m_a.transpose()<<std::endl;

  return m_a;
}

Eigen::Vector6d PegInHole::generateTwistSearchMotion(const Eigen::Isometry3d &origin,
                                                     const Eigen::Isometry3d &current,
                                                     const Eigen::Isometry3d &T_ea,
                                                     const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                     const double angle,
                                                     const double t,
                                                     const double t_0,
                                                     const double duration)
{
  Eigen::Vector3d pos_init;
  Eigen::Matrix3d ori_init;
  Eigen::Vector3d f_star, m_star;
  Eigen::Vector6d f_star_zero; // w.r.t {W}

  Eigen::Matrix3d K_p, K_v;

  Eigen::Vector3d p0_a, p_a, p_e; //wrt {A}
  Eigen::Vector3d target_position, desired_position, position;
  Eigen::Matrix3d R_e;
  Eigen::Matrix3d target_rotation, desired_rotation, rotation;
  Eigen::Vector3d delphi_delta;

  double t_e;
  double w;
  double theta;

  t_e = t - t_0;
  w = 2 * M_PI / duration;
  theta = angle * sin(w * t_e);

  p0_a = T_ea.inverse().translation(); //location of {E} wrt {A}
  p_a = dyros_math::rotateWithZ(theta) * p0_a;
  p_e = T_ea.linear() * p_a + T_ea.translation();

  R_e = dyros_math::angleaxis2rot(T_ea.linear() * Eigen::Vector3d::UnitZ(), theta);

  K_p << 3000, 0, 0, 0, 3000, 0, 0, 0, 3000;
  K_v << 100, 0, 0, 0, 100, 0, 0, 0, 100;

  pos_init = origin.translation();
  position = current.translation();
  target_position = origin.linear() * p_e + origin.translation();

  // for (int i = 0; i < 3; i++)
  // {
  //   desired_position(i) = dyros_math::cubic(t, t_0, t_0 + duration, pos_init(i), target_position(i), 0.0, 0.0);
  // }

  ori_init = origin.linear();
  rotation = current.linear();
  target_rotation = ori_init * R_e;

  delphi_delta = -0.5 * dyros_math::getPhi(rotation, target_rotation);

  f_star = K_p * (target_position - position) + K_v * (-xd.head<3>());
  m_star = (1.0) * 250.0 * delphi_delta + 5.0 * (-xd.tail<3>());

  f_star_zero << f_star, m_star;

  std::cout << "-------------------------------" << std::endl;
  // std::cout<<"tilt axis: "<<tilt_axis.transpose()<<std::endl;
  // std::cout<<"p0_a: "<<p0_a.transpose()<<std::endl;
  // std::cout<<"p_a: "<<p_a.transpose()<<std::endl;
  // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  // std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  // std::cout<<"pos_init: "<<pos_init.transpose()<<std::endl;
  // std::cout<<"desired_position: "<<desired_position.transpose()<<std::endl;
  // std::cout<<"position: "<<position.transpose()<<std::endl;
  // std::cout<<"targe_position: "<<target_position.transpose()<<std::endl;
  // std::cout<<"target rotation: \n"<<target_rotation<<std::endl;
  return f_star_zero;
}

Eigen::Vector3d PegInHole::generateRotationSearchMotionEE(const Eigen::Isometry3d &origin,
                                                          const Eigen::Isometry3d &current,
                                                          const Eigen::Isometry3d &T_ea,
                                                          const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                          const double angle,
                                                          const double t,
                                                          const double t_0,
                                                          const double duration,
                                                          const double pressing_force)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d f_a;
  Eigen::Vector3d p_init_a, p_cmd_a, p_cur_a, v_cur_a;
  Eigen::Vector3d f_motion, f_asm;
  Eigen::Matrix3d K_p, K_v;
  double theta;
  double dis;
  double force;

  force = pressing_force;

  K_p = Eigen::Matrix3d::Identity() * 5000;
  K_v = Eigen::Matrix3d::Identity() * 200;

  T_wa = origin * T_ea;

  theta = dyros_math::cubic(t, t_0, t_0 + duration, 0.0, angle, 0.0, 0.0);

  p_init_a = T_ea.inverse().translation(); //{E} wrt {A}

  dis = sqrt(pow(p_init_a(0), 2) + pow(p_init_a(1), 2));

  p_cmd_a(0) = dis * sin(theta);
  p_cmd_a(1) = dis * cos(theta);
  p_cmd_a(2) = p_init_a(2);
  // p_cmd_a = dyros_math::rotateWithZ(theta)*p_init_a;

  // p_cmd_a = p_init_a + p_cmd_a;
  p_cur_a = (T_wa.inverse() * current).translation();

  // p_cmd_w = T_wa*p_cmd_a;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();

  f_motion = K_p * (p_cmd_a - p_cur_a) + K_v * (-v_cur_a); // <fx, fy, fz>
  f_asm << 0.0, 0.0, force;                                //<0, 0, fz>

  f_a(0) = f_motion(0);
  f_a(1) = f_motion(1);
  f_a(2) = f_asm(2);

  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"dis: "<<dis<<std::endl;
  // std::cout<<"theta: "<<theta<<std::endl;
  // std::cout<<"p_init_a: "<<p_init_a.transpose()<<std::endl;
  // std::cout<<"p_cmd_a: "<<p_cmd_a.transpose()<<std::endl;
  // std::cout<<"p_cur_a: "<<p_cur_a.transpose()<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;

  return f_a;
}

Eigen::Vector3d PegInHole::getNormalForce(const Eigen::Vector3d &normal_vector, const double force)
{
  Eigen::Vector3d f;

  f = force * normal_vector;
  return f;
}

void PegInHole::getCompensationWrench(Eigen::Vector6d &accumulated_wrench, const Eigen::Vector6d &measured_wrench, const int num_start, const int num, const int num_max)
{
  for (int i = 0; i < 6; i++)
  {
    accumulated_wrench(i) += measured_wrench(i);
  }

  if (num >= num_max - 1)
  {
    accumulated_wrench = accumulated_wrench / (num_max - num_start - 1);
  }
}

Eigen::Vector3d PegInHole::getNormalVector(const Eigen::Vector3d &v)
{
  Eigen::Vector3d unit_vector;
  double norm;

  norm = sqrt(pow(v(0), 2) + pow(v(1), 2) + pow(v(2), 2));

  unit_vector = v / norm;

  return unit_vector;
}