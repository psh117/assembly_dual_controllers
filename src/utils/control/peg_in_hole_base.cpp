#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>

Eigen::Vector3d PegInHole::oneDofMoveEE(const Eigen::Isometry3d &origin,
                                        const Eigen::Isometry3d &current,
                                        const Eigen::Matrix<double, 6, 1> &xd,
                                        const Eigen::Isometry3d &T_7a,
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
  K_p.setIdentity();
  K_v.setIdentity();
  K_p = Eigen::Matrix3d::Identity() * 700;
  K_v = Eigen::Matrix3d::Identity() * 10;
  // K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  // K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

  T_wa = origin * T_7a; //T_WA

  p_init_a = T_7a.inverse().translation();

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
                                        const Eigen::Isometry3d &T_7a,
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
  K_p.setIdentity();
  K_v.setIdentity();
  K_p = Eigen::Matrix3d::Identity() * 700;
  K_v = Eigen::Matrix3d::Identity() * 10;
  // K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  // K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

  T_wa = origin * T_7a; //T_WA

  p_init_a = T_7a.inverse().translation();

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

Eigen::Vector3d PegInHole::getTiltDirection(const Eigen::Isometry3d T_7a)
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

  T_ae = T_7a.inverse();

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

bool PegInHole::setTilt(const Eigen::Isometry3d T_7a, const double threshold)
{
  Eigen::Vector3d proj_vector;
  Eigen::Vector3d p;
  Eigen::Vector3d asm_dir;
  double dis;
  bool set_tilt;

  dis = 0.0;
  p = T_7a.translation();
  asm_dir = T_7a.linear().col(2);

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

Eigen::Vector6d PegInHole::tiltMotion(const Eigen::Isometry3d origin,
                                      const Eigen::Isometry3d current,
                                      const Eigen::Ref<const Eigen::Vector6d> &xd, const Eigen::Isometry3d T_7a,
                                      const Eigen::Vector3d tilt_axis, //wrt {A}
                                      const double tilt_angle,
                                      const double t, 
                                      const double t_0, 
                                      const double duration,
                                      const double kp,
                                      const double kv)
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

  p0_a = T_7a.inverse().translation(); //location of {E} wrt {A}
  p_a = dyros_math::angleaxis2rot(tilt_axis, tilt_angle) * p0_a;
  p_e = T_7a.linear() * p_a + T_7a.translation();
  R_e = dyros_math::angleaxis2rot(T_7a.linear() * tilt_axis, tilt_angle);

  K_p = Eigen::Matrix3d::Identity()*kp;
  K_v = Eigen::Matrix3d::Identity()*kv;

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
  m_star = (1.0) * 2000.0 * delphi_delta + 15.0 * (-xd.tail<3>());

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
                                            const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                            const Eigen::Vector3d &target_dir,
                                            const double speed,
                                            const double t,
                                            const double t_0,
                                            const double kp,
                                            const double kv)
{
  Eigen::Matrix3d K_p, K_v;
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a, v_cmd_a, v_cur_a;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d f_a;
  K_p.setIdentity();
  K_v.setIdentity();
  K_p = Eigen::Matrix3d::Identity() * kp;
  K_v = Eigen::Matrix3d::Identity() * kv;

  T_wa = origin * T_7a; //T_WA

  p_init_a = T_7a.inverse().translation();

  v_cmd_a = speed * target_dir;
  p_cmd_a = p_init_a + speed * (t - t_0) * target_dir;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();
  p_cur_a = (T_wa.inverse() * current).translation();

  // f_a = K_v * (v_cmd_a - v_cur_a)
  f_a = K_p * (p_cmd_a - p_cur_a) + K_v * (v_cmd_a - v_cur_a); // <fx, fy, fz>

  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"origin: "<<origin.translation().transpose()<<std::endl;
  // std::cout<<"current: "<<current.translation().transpose()<<std::endl;
  // std::cout<<"run time: "<<t - t_0<<std::endl;
  // std::cout<<"p_init_a: "<<p_init_a.transpose()<<std::endl;
  // std::cout << "p_cmd_a: " << p_cmd_a.transpose() << std::endl;
  // std::cout << "p_cur_a: " << p_cur_a.transpose() << std::endl;
  // std::cout << "f_a: " << f_a.transpose() << std::endl;

  return f_a;
}

Eigen::Vector3d PegInHole::straightMotion(const Eigen::Isometry3d &origin,
                                          const Eigen::Isometry3d &current,
                                          const Eigen::Ref<const Eigen::Vector6d> &xd,
                                          const Eigen::Vector3d &target_dir,
                                          const double speed,
                                          const double t,
                                          const double t_0,
                                          const double kp,
                                          const double kv)
{
  Eigen::Matrix3d K_p, K_v;
  Eigen::Vector3d p_init, p_cmd, p_cur, v_cmd, v_cur;
  
  Eigen::Vector3d f_a;
  K_p.setIdentity();
  K_v.setIdentity();
  K_p = Eigen::Matrix3d::Identity() * kp;
  K_v = Eigen::Matrix3d::Identity() * kv;

  p_init = origin.translation();

  v_cmd = speed * target_dir;
  p_cmd = p_init + speed * (t - t_0) * target_dir;

  v_cur = xd.head<3>();
  p_cur = current.translation();

  // f_a = K_v * (v_cmd_a - v_cur_a)
  f_a = K_p * (p_cmd - p_cur) + K_v * (v_cmd - v_cur); // <fx, fy, fz>

  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"origin: "<<origin.translation().transpose()<<std::endl;
  // std::cout<<"current: "<<current.translation().transpose()<<std::endl;
  // std::cout<<"run time: "<<t - t_0<<std::endl;
  // std::cout<<"p_init: "<<p_init.transpose()<<std::endl;
  // std::cout<<"p_cmd: "<<p_cmd.transpose()<<std::endl;
  // std::cout<<"p_cur: "<<p_cur.transpose()<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;

  return f_a;
}

Eigen::Vector3d PegInHole::approachComponentEE(const Eigen::Isometry3d &origin,
                                               const Eigen::Isometry3d &current,
                                               const Eigen::Ref<const Eigen::Vector6d> &xd,
                                               const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                               const double speed,
                                               const double t,
                                               const double t_0,
                                               const double kp,
                                               const double kv)
{
  Eigen::Vector3d asm_dir_a;
  Eigen::Matrix3d K_p, K_v;
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a, v_cmd_a, v_cur_a;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d f_a;

  T_wa = origin * T_7a; //T_WA

  p_init_a = T_7a.inverse().translation();

  asm_dir_a << 0.0, 0.0, 1.0; // always z - axis wrt {A}

  K_p = Eigen::Matrix3d::Identity()*kp;
  K_v = Eigen::Matrix3d::Identity()*kv;

  v_cmd_a = speed * asm_dir_a;

  p_cmd_a = p_init_a + speed * (t - t_0) * asm_dir_a;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();
  p_cur_a = (T_wa.inverse() * current).translation();

  f_a = K_p * (p_cmd_a - p_cur_a) + K_v * (v_cmd_a - v_cur_a); // <fx, fy, fz>

  Eigen::Vector3d p_cmd_w;
  p_cmd_w = T_wa.translation() + T_wa.linear()*p_cmd_a;

  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"T_WA: \n"<< T_wa.matrix()<<std::endl;
  // std::cout<<"T_7a: \n"<< T_7a.matrix()<<std::endl;
  // std::cout<<"origin: "<<origin.translation().transpose()<<std::endl;
  // std::cout<<"current: "<<current.translation().transpose()<<std::endl;
  // std::cout<<"run time: "<<t - t_0<<std::endl;
  // std::cout<<"p_init_a: "<<p_init_a.transpose()<<std::endl;
  // std::cout<<"p_cmd_a: "<<p_cmd_a.transpose()<<std::endl;
  // std::cout<<"p_cur_a: "<<p_cur_a.transpose()<<std::endl;
  // std::cout<<"p_cmd_w: "<<p_cmd_w.transpose()<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;
  
  return f_a;
}

//Edited by KimHC 20200623 - additional movement to XY direction
Eigen::Vector3d PegInHole::pinMoveEE(const Eigen::Isometry3d &origin,
                                     const Eigen::Isometry3d &current,
                                     const Eigen::Ref<const Eigen::Vector6d> &xd,
                                     const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
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
  T_wa = origin * T_7a;
  p_init_a = T_7a.inverse().translation();

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
                                          const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                          const double t,
                                          const double t_0,
                                          const double duration,
                                          const double kp,
                                          const double kv)
{
  Eigen::Vector3d asm_dir_a;
  Eigen::Matrix3d K_p, K_v;
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a, v_cur_a;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d f_a;
  
  K_p.setIdentity();
  K_v.setIdentity();
  K_p = Eigen::Matrix3d::Identity() * kp;
  K_v = Eigen::Matrix3d::Identity() * kv;

  T_wa = origin * T_7a;
  p_init_a = T_7a.inverse().translation();

  for (int i = 0; i < 3; i++)
    p_cmd_a(i) = dyros_math::cubic(t, t_0, t_0 + duration, 0.0, target(i), 0.0, 0.0);
  p_cmd_a = p_init_a + p_cmd_a;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();
  p_cur_a = (T_wa.inverse() * current).translation();

  f_a = K_p * (p_cmd_a - p_cur_a) + K_v * (-v_cur_a); // <fx, fy, fz>

  // std::cout<<"p_cur_a : "<< p_cur_a.transpose()<<std::endl;
  
  return f_a;
}

Eigen::Vector3d PegInHole::threeDofMove(const Eigen::Isometry3d &origin,
                                        const Eigen::Isometry3d &current,
                                        const Eigen::Vector3d &target,
                                        const Eigen::Ref<const Eigen::Vector6d> &xd,
                                        const double t,
                                        const double t_0,
                                        const double duration,
                                        const double kp,
                                        const double kv)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d desired_position;
  Eigen::Matrix3d K_p, K_v;

  K_p = Eigen::Matrix3d::Identity()*kp;
  K_v = Eigen::Matrix3d::Identity()*kv;

  for (int i = 0; i < 3; i++)
  {
    desired_position(i) = dyros_math::cubic(t, t_0, t_0 + duration, 0, target(i) - origin.translation()(i), 0.0, 0.0);
  }

  f_star = K_p * (origin.translation() + desired_position - current.translation()) + K_v * (-xd.head<3>());

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

Eigen::Vector3d PegInHole::pressEE(const double force, 
                                    const Eigen::Ref<const Eigen::Vector6d> &xd,
                                    const Eigen::Isometry3d &T_wa,
                                    const double speed, 
                                    const double kp)
{
  Eigen::Vector3d asm_dir, f_a, v_cur_a;
  Eigen::Matrix3d Kp;

  asm_dir = Eigen::Vector3d::UnitZ();
  Kp = kp*Eigen::Matrix3d::Identity();

  v_cur_a = T_wa.linear().inverse() * xd.head<3>();
  
  for (int i = 0; i < 3; i++)   v_cur_a(i) = asm_dir(i) * v_cur_a(i);
  
  f_a = force * asm_dir + Kp * (speed * asm_dir - v_cur_a);
  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;
  // std::cout<<"velocity error : "<< (speed * asm_dir - v_cur_a).transpose()<<std::endl;
  return f_a;
}

Eigen::Vector3d PegInHole::pressCubicEE(const Eigen::Isometry3d &origin,
                                        const Eigen::Isometry3d &current,
                                        const Eigen::Ref<const Eigen::Vector6d> &xd,
                                        const Eigen::Isometry3d &T_wa,
                                        const double force,
                                        const double t,
                                        const double t_0,
                                        const double duration,
                                        const double f_init,
                                        const double kp,
                                        const double kv)
{
  Eigen::Vector3d asm_dir, f_a;
  Eigen::Vector3d p_init_a, p_cur_a, v_cur_a; // EE w.r.t {A}
  Eigen::Vector3d f_motion;
  double f_asm;
  Eigen::Matrix3d K_p, K_v;

  asm_dir = Eigen::Vector3d::UnitZ();

  K_p = kp*Eigen::Matrix3d::Identity();
  K_v = kv*Eigen::Matrix3d::Identity();

  p_init_a = (T_wa.inverse()*origin).translation();
  p_cur_a = (T_wa.inverse()*current).translation();
  v_cur_a = T_wa.linear().inverse()*xd.head<3>();
  
  f_motion = K_p*(p_init_a - p_cur_a) + K_v*(-v_cur_a); // keep current x/y position
  f_motion(2) = 0.0;  
  
  f_asm = dyros_math::cubic(t, t_0, t_0 + duration, f_init, force, 0.0, 0.0);  
  f_asm = f_asm + 50.0 * ( - v_cur_a(2));

  f_a = f_motion + asm_dir * f_asm;

  // std::cout<<"------------------------------"<<std::endl;
  // std::cout<<"init force     : "<<f_init<<std::endl;
  // std::cout<<"force damping  : "<< 100.0 * (- v_cur_a(2))<<std::endl;
  // std::cout<<"assembly force : "<< f_asm<<std::endl;
  
  return f_a;
}

Eigen::Vector6d PegInHole::pressCubicEE2(const Eigen::Isometry3d &origin,
                                         const Eigen::Isometry3d &current,
                                         const Eigen::Ref<const Eigen::Vector6d> &xd,
                                         const Eigen::Isometry3d &T_wa,
                                         const double force,
                                         const double t,
                                         const double t_0,
                                         const double duration,
                                         const double f_init,
                                         const double kp,
                                         const double kv)
{
  Eigen::Vector3d asm_dir;
  Eigen::Vector6d f_a;
  Eigen::Vector3d p_init_a, p_cur_a, v_cur_a; // EE w.r.t {A}
  Eigen::Vector3d f_motion, f_active_force;  
  Eigen::Matrix3d K_p, K_v;

  asm_dir = Eigen::Vector3d::UnitZ();

  K_p = kp*Eigen::Matrix3d::Identity();
  K_v = kv*Eigen::Matrix3d::Identity();

  p_init_a = (T_wa.inverse()*origin).translation();
  p_cur_a = (T_wa.inverse()*current).translation();
  v_cur_a = T_wa.linear().inverse()*xd.head<3>();
  
  f_motion = K_p*(p_init_a - p_cur_a) + K_v*(-v_cur_a); // keep current x/y position
  f_motion(2) = 50.0 * ( - v_cur_a(2)); // velocity feedback for generating force!!
  // f_motion(2) = 0.0;  
  
  f_active_force.setZero();
  f_active_force(2) = dyros_math::cubic(t, t_0, t_0 + duration, f_init, force, 0.0, 0.0);  
  
  f_a.head<3>() = f_motion;
  f_a.tail<3>() = f_active_force;

  // std::cout<<"------------------------------"<<std::endl;
  // std::cout<<"init force     : "<<f_init<<std::endl;
  // std::cout<<"force damping  : "<< 100.0 * (- v_cur_a(2))<<std::endl;
  // std::cout<<"assembly force : "<< f_asm<<std::endl;
  
  return f_a;
}

Eigen::Vector6d PegInHole::generateTwistSpiralEE(const Eigen::Isometry3d &origin,
                                                 const Eigen::Isometry3d &current,
                                                 const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                 const double pitch,
                                                 const double lin_vel,
                                                 const double force,
                                                 const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
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

  K_p = Eigen::Matrix3d::Identity() * 700;
  K_v = Eigen::Matrix3d::Identity() * 10;

  T_wa = origin * T_7a; //T_WA

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
  // T_A7 = T_7a.inverse();

  T_A7 = T_Aa * T_7a.inverse(); // cmd for {E} w.r.t {A}

  pos_target = T_A7.translation();
  pos = (T_wa.inverse() * current).translation(); // {E} w.r.t {A}
  v_vel = T_wa.linear().transpose() * xd.head<3>();

  rot_target = T_A7.linear();
  rot = T_wa.linear().inverse() * current.linear();
  w_vel = T_wa.linear().transpose() * xd.tail<3>();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, rot_target);

  f_a = K_p * (pos_target - pos) + K_v * (-v_vel);
  m_a = 2000.0 * delphi_delta + 15.0 * (-w_vel);

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
                                                          const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
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

  T_wa = origin * T_7a; //T_WA

  t_e = t - t_0;
  w = 2 * M_PI / revolve_duration;
  theta = angle * sin(w * t_e);

  pos_Aa_cmd.head<2>() = dyros_math::spiral(t, t_0, t_0 + spiral_duration, Eigen::Vector2d::Zero(), lin_vel, pitch);
  pos_Aa_cmd(2) = 0.0;
  rot_Aa_cmd = dyros_math::rotateWithZ(theta);

  T_Aa_cmd.linear() = rot_Aa_cmd;
  T_Aa_cmd.translation() = pos_Aa_cmd;
  T_A7_cmd = T_Aa_cmd * T_7a.inverse(); // cmd for {E} w.r.t {A}

  T_Aa_cur = T_wa.inverse() * current * T_7a;
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
                                            const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                            const double t,
                                            const double t_0,
                                            const double duration,
                                            const double f_gain,
                                            const bool set_tilt,
                                            const double kp,
                                            const double kv)
{
  Eigen::Vector2d start_point, traj;
  Eigen::Vector3d f_a, f_motion, f_asm; //wrt {A}
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a, p_cur_w, p_cmd_w;
  Eigen::Vector3d v_cur_a;

  Eigen::Matrix3d K_p, K_v;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d asm_dir;

  K_p = Eigen::Matrix3d::Identity() * kp;
  K_v = Eigen::Matrix3d::Identity() * kv;
  // asm_dir = T_7a.linear().col(2); //w.r.t {E}
  asm_dir = Eigen::Vector3d::UnitZ();
  start_point.setZero();

  T_wa = origin * T_7a; //T_WA

  traj = dyros_math::spiral(t, t_0, t_0 + duration, start_point, lin_vel, pitch);

  p_init_a = T_7a.inverse().translation();
  
  p_cmd_a << traj(0), traj(1), 0.0; //w.r.t {A}
  p_cmd_a = p_init_a + p_cmd_a;

  // p_cur_w = (current * T_7a).translation() - T_wa.translation(); // T_we * T_7a - p_wa
  // p_cur_a = T_wa.linear().transpose() * p_cur_w;
  p_cur_a = (T_wa.inverse() * current).translation(); // {E} w.r.t {A}

  // p_cmd_w = T_wa*p_cmd_a;

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();

  f_motion = K_p * (p_cmd_a - p_cur_a) + K_v * (-v_cur_a); // <fx, fy, fz>
  // f_asm = PegInHole::pressEE(force, xd, T_wa, 0.01, f_gain);
  double f_init = 1.0;
  f_asm = PegInHole::pressCubicEE(origin, current, xd, T_wa, force, t, t_0, 0.5, f_init);
  f_a(0) = f_motion(0);
  f_a(1) = f_motion(1);
  
  if(set_tilt == true)  f_a(2) = f_motion(2);
  else                  f_a(2) = f_asm(2);
  
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

Eigen::Vector6d PegInHole::generateSpiralEE2(const Eigen::Isometry3d &origin,
                                             const Eigen::Isometry3d &current,
                                             const Eigen::Ref<const Eigen::Vector6d> &xd,
                                             const double pitch,
                                             const double lin_vel,
                                             const double force,
                                             const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                             const double t,
                                             const double t_0,
                                             const double duration,
                                             const double f_gain,
                                             const bool set_tilt,
                                             const double kp,
                                             const double kv)
{
  Eigen::Vector2d start_point, traj;
  Eigen::Vector6d f_motion, f_active_force; //wrt {A}
  Eigen::Vector6d f_a; // [f_motion' f_active_force']'
  Eigen::Vector3d p_init_a;
  Eigen::Vector3d p_cmd_a, p_cur_a, p_cur_w, p_cmd_w;
  Eigen::Vector3d v_cur_a;

  Eigen::Matrix3d K_p, K_v;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d asm_dir;

  K_p = Eigen::Matrix3d::Identity() * kp;
  K_v = Eigen::Matrix3d::Identity() * kv;
  
  asm_dir = Eigen::Vector3d::UnitZ();
  start_point.setZero();

  T_wa = origin * T_7a; //T_WA

  traj = dyros_math::spiral(t, t_0, t_0 + duration, start_point, lin_vel, pitch);

  p_init_a = T_7a.inverse().translation();  
  p_cmd_a << traj(0), traj(1), 0.0; //w.r.t {A}
  p_cmd_a = p_init_a + p_cmd_a;
  p_cur_a = (T_wa.inverse() * current).translation(); // {E} w.r.t {A}  
  v_cur_a = T_wa.linear().transpose() * xd.head<3>();

  f_motion.head<3>() = K_p * (p_cmd_a - p_cur_a) + K_v * (-v_cur_a); // <fx, fy, fz>
  f_motion.tail<3>().setZero();
  // f_asm = PegInHole::pressEE(force, xd, T_wa, 0.01, f_gain);

  double f_init = 0.0;  
  f_active_force = PegInHole::pressCubicEE2(origin, current, xd, T_wa, force, t, t_0, 0.5, f_init);
  f_motion(2) = f_active_force(2);

  f_a.head<3>() = f_motion.head<3>();
  f_a.tail<3>() = f_active_force.tail<3>();

  
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
                                                     const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
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

  asm_dir = T_7a.linear().col(2); //w.r.t {E}

  start_point.setZero();

  T_wa = origin * T_7a; //T_WA

  traj = dyros_math::spiral(t, t_0, t_0 + duration, start_point, lin_vel, pitch);

  p_init_a = T_7a.inverse().translation();

  p_cmd_a << traj(0), traj(1), 0.0; //w.r.t {A}
  p_cur_a = (T_7a.inverse() * ((T_wa.inverse() * current)).inverse()).translation();

  pos_save << p_cmd_a, p_cur_a;

  return pos_save;
}

Eigen::Vector3d PegInHole::rotateWithAseemblyDir(const Eigen::Isometry3d &origin,
                                                 const Eigen::Isometry3d &current,
                                                 const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                 const Eigen::Isometry3d T_7a,
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

  T_wa = origin * T_7a;

  axis = Eigen::Vector3d::UnitZ();

  rot_init = T_wa.linear().transpose() * origin.linear();
  // rot_init = Eigen::Matrix3d::Identity();
  target_rot = dyros_math::angleaxis2rot(axis, target_angle) * rot_init; //wrt {A}
  desired_rot = dyros_math::rotationCubic(t, t_0, t_0 + duration, rot_init, target_rot);
  rot = T_wa.linear().transpose() * current.linear();

  // v_cur_w = xd.tail<3>();

  // v_cur_a = T_wa.linear().transpose()*v_cur_w*T_7a.linear();
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
                                         const double duration,
                                         const double kp,
                                         const double kv) //axis_vector wrt EE frame
{
  Eigen::Vector3d m_star;
  Eigen::Vector3d delphi_delta;
  Eigen::Matrix3d rot_init, rot, desired_rot;

  rot_init = origin.linear();
  desired_rot = dyros_math::rotationCubic(t, t_0, t_0 + duration, rot_init, target_rot);
  rot = current.linear();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, desired_rot);

  m_star = kp * delphi_delta + kv * (-xd.tail<3>());

  return m_star;
}

Eigen::Vector3d PegInHole::generateWiggleMotionEE(const Eigen::Isometry3d &origin,
                                                  const Eigen::Isometry3d &current,
                                                  const Eigen::Isometry3d &T_7a,
                                                  const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                  const double angle,
                                                  const double w, //rad/s
                                                  const double t,
                                                  const double t_0,
                                                  const double kp)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d x_axis, y_axis, z_axis;
  Eigen::Vector3d m_a, delphi_delta;
  Eigen::Matrix3d rot_init, rot;
  Eigen::Matrix3d target_rot, target_rot_x, target_rot_y, target_rot_z;
  Eigen::Vector3d v_cur_w, v_cur_a;

  T_wa = origin * T_7a;

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

  m_a = (1.0) * kp * delphi_delta + 0.1 * (-v_cur_a);

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
                                                        const Eigen::Isometry3d &T_7a,
                                                        const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                        const double angle,
                                                        const double w, //rad/s
                                                        const int a,
                                                        const double b,
                                                        const double t_offset,
                                                        const double t,
                                                        const double t_0,
                                                        const double kp)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d z_axis;
  Eigen::Vector3d m_a, delphi_delta;
  Eigen::Matrix3d rot_init, rot;
  Eigen::Matrix3d target_rot, target_rot_z;
  Eigen::Vector3d v_cur_w, v_cur_a;

  T_wa = origin * T_7a;

  z_axis = Eigen::Vector3d::UnitZ();
  double t_e;
  double theta;

  t_e = t - t_0;
  theta = a * angle * sin(w * (t_e + t_offset)) + b;
  
  rot_init = T_wa.linear().transpose() * origin.linear(); //initial {E} wrt {A}
  target_rot_z = dyros_math::angleaxis2rot(z_axis, theta); //wrt {A}  
  target_rot = target_rot_z * rot_init;
  rot = T_wa.linear().transpose() * current.linear();

  v_cur_a = T_wa.linear().transpose() * xd.tail<3>();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, target_rot);

  m_a = (1.0) * kp * delphi_delta + 2.5 * (-v_cur_a);

  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"theta: "<<theta*RAD2DEG<< std::endl;
  // std::cout<<"m_a_wiggle: "<<m_a.transpose()<<std::endl;

  return m_a;
}

Eigen::Vector3d PegInHole::generateYawingMotionEE(const Eigen::Isometry3d &origin,
                                                  const Eigen::Isometry3d &current,
                                                  const Eigen::Isometry3d &T_7a,
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

  T_wa = origin * T_7a;
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
                                                     const Eigen::Isometry3d &T_7a,
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

  p0_a = T_7a.inverse().translation(); //location of {E} wrt {A}
  p_a = dyros_math::rotateWithZ(theta) * p0_a;
  p_e = T_7a.linear() * p_a + T_7a.translation();

  R_e = dyros_math::angleaxis2rot(T_7a.linear() * Eigen::Vector3d::UnitZ(), theta);

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
                                                          const Eigen::Isometry3d &T_7a,
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

  T_wa = origin * T_7a;

  theta = dyros_math::cubic(t, t_0, t_0 + duration, 0.0, angle, 0.0, 0.0);

  p_init_a = T_7a.inverse().translation(); //{E} wrt {A}

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

void PegInHole::getCompensationWrench(Eigen::Vector6d &accumulated_wrench, const Eigen::Ref<const Eigen::Vector6d> &measured_wrench, const int num_start, const int num, const int num_max)
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

Eigen::MatrixXd PegInHole::LeastSquareEdgeProbing(const std::vector<double> x_vec,
                                                  const std::vector<double> y_vec)
{
  int data_size;
  Eigen::MatrixXd X, Y, coeff_vec;

  data_size = x_vec.size();
  
  X.resize(data_size, 2); // for y = ax + b
  Y.resize(data_size, 1);
  
  coeff_vec.resize(2,1);
  
  for(int i = 0; i < data_size; i++)
  {
    X(i,0) = x_vec.at(i);
    X(i,1) = 1.0;
    Y(i) = y_vec.at(i);
  }
  
  coeff_vec = (X.transpose()*X).inverse()*X.transpose()*Y;
 
  return coeff_vec.transpose();
}

Eigen::Vector7d PegInHole::inspectJointLimit(const Eigen::Vector7d &joint_angle,
                                             const Eigen::Vector7d &joint_angle_ub,
                                             const Eigen::Vector7d &joint_angle_lb,
                                             const double inspection,
                                             const double angle_margin) // angle_margin is set to a positive value
{
  Eigen::Vector7d avoid_joint_limit;

  avoid_joint_limit.setZero();

  //check joint limit upper bound
  for(int i = 0; i < inspection; i++)
  {
    double angle;
    angle = joint_angle(i) + angle_margin;
    if(angle >= joint_angle_ub(i)) avoid_joint_limit(i) = -angle_margin;
  }

  //check joint limit lower bound
  for(int i = 0; i < inspection; i ++)
  {
    double angle;
    angle = joint_angle(i) - angle_margin;
    if(angle <= joint_angle_lb(i)) avoid_joint_limit(i) = angle_margin;
  }

  return avoid_joint_limit;
}

Eigen::Vector7d PegInHole::nullSpaceJointTorque(const Eigen::Vector7d &q_cur,
                                                const Eigen::Vector7d &q_init,
                                                const Eigen::Vector7d &q_target,
                                                const Eigen::Vector7d &q_dot,
                                                const double t,
                                                const double t_0,
                                                const double duration,
                                                const double kp,
                                                const double kv)
{
  Eigen::Matrix7d Kp,Kv;
  Eigen::Vector7d tau_0, q_desired;

  Kp = Eigen::Matrix7d::Identity()*kp;
  Kv = Eigen::Matrix7d::Identity()*kv;

  for(int i = 0; i < 7; i++)
  {
    q_desired(i) = dyros_math::cubic(t, t_0, t_0 + duration, q_init(i), q_target(i), 0.0, 0.0);
  }

  tau_0 = Kp*(q_desired - q_cur) + Kv*(-q_dot);

  return tau_0;
}

Eigen::Vector7d PegInHole::getNullSpaceJointTarget(const Eigen::Matrix<double, 7, 2> &joint_limit_info,
                                                   const Eigen::Vector7d &q_init)
{
  int joint_inspection_num;
  double joint_margin;

  Eigen::Vector7d q_margin_vec;
  Eigen::Vector7d joint_limit_ub, joint_limit_lb;
  Eigen::Vector7d q_null_target;

  joint_inspection_num = 5;
  joint_margin = 10 * DEG2RAD;

  q_margin_vec.resize(joint_inspection_num);
  q_margin_vec.setZero();

  joint_limit_lb = joint_limit_info.col(0);
  joint_limit_ub = joint_limit_info.col(1);

  q_margin_vec = PegInHole::inspectJointLimit(q_init, joint_limit_ub, joint_limit_lb, joint_inspection_num, joint_margin);
  q_null_target = q_init;
  q_null_target += q_margin_vec;

  return q_null_target;
}

Eigen::Vector3d PegInHole::followSphericalCoordinateEE(const Eigen::Isometry3d &origin,
                                                       const Eigen::Isometry3d &current,
                                                       const Eigen::Isometry3d &target,
                                                       const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                       const Eigen::Isometry3d &T_7a,
                                                       const double t,
                                                       const double t_0,
                                                       const double duration,
                                                       const double radius,
                                                       const double kp,
                                                       const double kv)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d target_vec;
  Eigen::Vector3d p_init_a, p_cmd_a, p_cur_a, p_target_a, v_cur_a;
  Eigen::Vector3d f_a;
  Eigen::Matrix3d K_p, K_v;  
  double target_theta, target_phi, rho; // parameters for spherical coordinate
  double init_theta, init_phi;
  double theta, phi;
  double margin = 5*DEG2RAD;

  T_wa = origin*T_7a;

  K_p = Eigen::Matrix3d::Identity()*kp;
  K_v = Eigen::Matrix3d::Identity()*kv;

  target_vec = target.translation() - origin.translation();
  // p_target_a = (target*T_7a).inverse().translation();
  p_target_a = (T_wa.inverse()*target).translation();
  p_init_a = T_7a.inverse().translation();

  rho = radius;

  init_theta = acos(p_init_a(2)/rho);
  init_phi = atan2(p_init_a(1), p_init_a(0));

  target_theta = acos(p_target_a(2)/rho) + margin;
  target_phi = atan2(p_target_a(1),p_target_a(0));

  theta = dyros_math::cubic(t, t_0, t_0 + duration, init_theta, target_theta, 0.0, 0.0);
  phi = dyros_math::cubic(t, t_0, t_0 + duration, init_phi, target_phi, 0.0, 0.0);

  p_cmd_a << rho*sin(theta)*cos(phi), rho*sin(theta)*sin(phi), rho*cos(theta);
  // p_cmd_a = p_init_a + p_cmd_a;
  v_cur_a = T_wa.linear().transpose() * xd.head<3>();
  p_cur_a = (T_wa.inverse() * current).translation();
  
  f_a = K_p*(p_cmd_a - p_cur_a) + K_v*(-v_cur_a);

  // std::cout<<"---------------------------"<<std::endl;
  // std::cout<<"p_init_a    : "<< p_init_a.transpose()<<std::endl;
  // std::cout<<"p_target_a  : "<< p_target_a.transpose()<<std::endl;
  // std::cout<<"p_cmd_a     : "<< p_cmd_a.transpose()<<std::endl;
  // std::cout<<"p_cur_a     : "<< p_cur_a.transpose()<<std::endl;
  // std::cout<<"init_theta  : "<< init_theta*RAD2DEG<<std::endl;
  // std::cout<<"init_phi    : "<< init_phi*RAD2DEG<<std::endl;
  // std::cout<<"target_theta: "<< target_theta*RAD2DEG<<std::endl;
  // std::cout<<"target_phi  : "<< target_phi*RAD2DEG<<std::endl;
  // std::cout<<"f_a         : "<< f_a.transpose()<<std::endl;

  return f_a;
}

Eigen::Vector3d PegInHole::generateMoment(const Eigen::Isometry3d &origin,
                                            const Eigen::Isometry3d &current,
                                            const Eigen::Isometry3d &T_7a,
                                            const Eigen::Ref<const Eigen::Vector6d> &xd,
                                            const double angle,
                                            const double t,
                                            const double t_0,
                                            const double duration,
                                            const double kp,
                                            const double kv)
{
  Eigen::Vector3d asm_dir, r_a7, target_dir;
  Eigen::Vector3d m_star, delphi_delta;
  Eigen::Isometry3d T_wa;
  Eigen::Matrix3d rot_target, rot_cmd;

  T_wa = origin * T_7a;

  asm_dir = Eigen::Vector3d::UnitZ();
  r_a7 = (T_7a.inverse()).translation();
  r_a7 = r_a7/r_a7.norm();
  target_dir = asm_dir.cross(r_a7);
  target_dir = T_7a.linear()*target_dir;
  // Eigen::Matrix3d target_rot_a;
  // target_rot_a = dyros_math::angleaxis2rot(target_dir, angle);

  // rot_target = origin.linear()*T_7a.linear()*target_rot_a; // final rotation w.r.t {W}
  rot_target = origin.linear()*dyros_math::angleaxis2rot(target_dir, angle);
  rot_cmd = dyros_math::rotationCubic(t, t_0, t_0 + duration, origin.linear(), rot_target);

  m_star = PegInHole::rotateWithMat(origin, current, xd, rot_target, t, t_0, t_0 + duration, kp, kv);
  delphi_delta = -0.5 * dyros_math::getPhi(current.linear(), rot_cmd);
  m_star = kp * delphi_delta + kv * (-xd.tail<3>());

  Eigen::Vector3d init_euler, cur_euler, cmd_euler, target_euler;
  init_euler = dyros_math::rot2Euler(origin.linear());
  cur_euler = dyros_math::rot2Euler(current.linear());
  cmd_euler = dyros_math::rot2Euler(rot_cmd);
  target_euler = dyros_math::rot2Euler(rot_target);

  // std::cout << "--------------------" << std::endl;
  // std::cout << "m_star : " << m_star.transpose() << std::endl;
  // std::cout << "init_rot : \n" << origin.linear()<<std::endl;
  // std::cout << "current_ rot : \n" << current.linear()<<std::endl;
  // std::cout << "rot_target : \n" << rot_target<<std::endl;
  // std::cout << "init euler  : "<< init_euler.transpose()*180/M_PI<<std::endl;
  // std::cout << "cur_euler   : "<< cur_euler.transpose()*180/M_PI<<std::endl;
  // std::cout << "target_euler: "<< target_euler.transpose()*180/M_PI<<std::endl;
  // std::cout << "cmd_euler   : "<< cmd_euler.transpose()*180/M_PI<<std::endl;
  // std::cout << "init_rot : \n"<<origin.linear()<<std::endl;
  
  return m_star;
}

Eigen::Vector3d PegInHole::oneDofRotation(const Eigen::Isometry3d &origin,
                                          const Eigen::Isometry3d &current,
                                          const Eigen::Ref<const Eigen::Vector6d> &xd,
                                          const Eigen::Vector3d &m,
                                          const double speed, // + : positive, - : negative/ deg/s
                                          const double dir,    // 0 : x, 1 : y, 2 : z w.r.t global frame
                                          const double t,
                                          const double t_0,                                          
                                          const double kp,
                                          const double kv)
{
  Eigen::Vector3d m_star, delphi_delta;
  Eigen::Vector3d axis;
  Eigen::Matrix3d rot_target;
  double theta;

  theta = speed*(t - t_0)*DEG2RAD;
  axis.setZero();
  axis(dir) = 1.0;

  rot_target = origin.linear() * dyros_math::angleaxis2rot(axis, theta);
  delphi_delta = -0.5 * dyros_math::getPhi(current.linear(), rot_target);
  
  m_star = kp * delphi_delta + kv * (-xd.tail<3>());
  
  // std::cout<<"------------"<<std::endl;
  // std::cout<<"run time: "<<t - t_0 << std::endl;
  // std::cout<<"axis   : "<<axis.transpose()<<std::endl;
  // std::cout<<"theta  : "<<theta*RAD2DEG<<std::endl;
  // std::cout<<"m_star : "<<m_star.transpose()<<std::endl;
  
  return m_star;
}

Eigen::Vector3d PegInHole::generatePartialSpiralEE(const Eigen::Isometry3d &origin,
                                                   const Eigen::Isometry3d &current,
                                                   const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                   const double pitch,
                                                   const double lin_vel,
                                                   const double force,
                                                   const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                                   const double search_dir,       // 0 : x axis, 1 : y axis / z is fixed as an assembly dir.
                                                   const double t,
                                                   const double t_0,
                                                   const double duration,
                                                   const double kp,
                                                   const double kv)
{
  Eigen::Vector2d start_point, traj;
  Eigen::Vector3d f_motion, f_asm, f_a; //wrt {A}
  Eigen::Vector3d p_init_a, p_cmd_a, p_cur_a;
  Eigen::Vector3d v_cur_a;
  Eigen::Matrix3d K_p, K_v;
  Eigen::Isometry3d T_wa;
  
  K_p = Eigen::Matrix3d::Identity() * kp;
  K_v = Eigen::Matrix3d::Identity() * kv;
  T_wa = origin * T_7a; //T_WA

  start_point.setZero();
  traj.setZero();
  traj(search_dir) = dyros_math::spiral(t, t_0, t_0 + duration, start_point, lin_vel, pitch)(search_dir);

  p_init_a = T_7a.inverse().translation();
  
  p_cmd_a << traj(0), traj(1), 0.0; //w.r.t {A}
  p_cmd_a = p_init_a + p_cmd_a;
  p_cur_a = (T_wa.inverse() * current).translation(); // {E} w.r.t {A}
  v_cur_a = T_wa.linear().transpose() * xd.head<3>();

  f_motion = K_p * (p_cmd_a - p_cur_a) + K_v * (-v_cur_a); // <fx, fy, fz>
  f_asm = PegInHole::pressEE(force, xd, T_wa, 0.01, 5.0);
  f_a.head<2>() = f_motion.head<2>();
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

Eigen::Vector3d PegInHole::generateForceToDisassembleEE(const Eigen::Isometry3d &origin,
                                                        const Eigen::Isometry3d &current,
                                                        const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                        const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                                        const double force,                                                    
                                                        const double t,
                                                        const double t_0,
                                                        const double duration)
{
  Eigen::Vector3d f_motion, f_asm, f_a; //wrt {A}
  Eigen::Isometry3d T_wa;
  
  T_wa = origin * T_7a; //T_WA

  f_motion = PegInHole::vibrationForce(2.0, t, t_0, duration);
  f_asm = -PegInHole::pressCubicEE(origin, current, xd, T_wa, force, t, t_0, 2.0, 0.0,  100.0, 0.0);
  
  f_a.head<2>() = f_motion.head<2>();
  f_a(2) = f_asm(2);
  
  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"count: "<<count<<std::endl;
  // std::cout<<"p_init_a: "<<p_init_a.transpose()<<std::endl;
  // std::cout<<"p_cmd_a: "<<p_cmd_a.transpose()<<std::endl;
  // std::cout<<"p_cur_a: "<<p_cur_a.transpose()<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;

  return f_a;
}

Eigen::Vector3d PegInHole::vibrationForce(const double f_max,
                                          const double t,
                                          const double t_0,
                                          const double duration)
{
  double f, fx, fy;
  double run_time;
  Eigen::Vector3d f_vibration;

  run_time = t - t_0;

  fx = f_max*sin(2*M_PI/duration*run_time);
  fy = f_max*cos(2*M_PI/duration*run_time);

  f_vibration << fx, fy, 0.0;

  std::cout<<"========================"<<std::endl;
  std::cout<<"run time : "<<run_time<<std::endl;
  std::cout<<f_vibration.transpose()<<std::endl;
  return f_vibration;
}

Eigen::Vector3d PegInHole::generateInsertionForceWithLessFriction(const Eigen::Isometry3d &origin,
                                                                  const Eigen::Isometry3d &current,
                                                                  const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                                  const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                                                  const Eigen::Vector3d &f_ext,
                                                                  const Eigen::Vector3d &f_ext_error,
                                                                  const double force,
                                                                  const double t,
                                                                  const double t_0,
                                                                  const double duration,
                                                                  const double kp,
                                                                  const double ki,
                                                                  const double kv)
{
  Eigen::Vector3d f_a, f_asm, f_motion;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d f_ext_a, f_ext_error_a;

  T_wa = origin * T_7a; //T_WA
  f_ext_a = T_wa.linear().inverse()*f_ext;
  f_ext_error_a = T_wa.linear().inverse()*f_ext_error;

  f_asm = PegInHole::pressCubicEE(origin, current, xd, T_wa, force, t, t_0, duration,0.0, 100.0, 0.0);

  f_motion = -kp*f_ext_a - kv*xd.head<3>() + ki*f_ext_error_a; // f_ext is w.r.t {W}

  f_a.head<2>() = f_motion.head<2>();
  f_a.tail<2>() = f_asm.tail<2>();

  return f_a;
}

Eigen::Vector3d PegInHole::generateMomentToDisassembleEE(const Eigen::Isometry3d &origin,
                                                        const Eigen::Isometry3d &current,
                                                        const Eigen::Ref<const Eigen::Vector6d> &xd,
                                                        const Eigen::Isometry3d &T_7a, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
                                                        const double momnet,                                                    
                                                        const double t,
                                                        const double t_0,
                                                        const double duration)
{
  Eigen::Vector3d m_motion, m_a; //wrt {A}
  Eigen::Isometry3d T_wa;
  
  T_wa = origin * T_7a; //T_WA

  m_motion = PegInHole::vibrationForce(momnet, t, t_0, duration);
  
  m_a.head<2>() = m_motion.head<2>();
  m_a(2) = 0.0;
  return m_a;
}

Eigen::Vector6d PegInHole::generateRotationToDisassembleEE(const Eigen::Isometry3d &origin,
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
                                                           const double kp,
                                                           const double kv,
                                                           const double kwp,
                                                           const double kwv)
{
  Eigen::Vector3d f_a, f_asm, f_motion, m_a, m_motion, m_asm;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d p_init_a, p_cmd_a, p_cur_a, v_cur_a;
  double theta, l;
  Eigen::Vector3d asm_dir, delphi_delta;
  Eigen::Matrix3d r_init_a, r_target_a, r_cmd_a, r_cur_a;
  Eigen::Vector3d w_cur_a;
  Eigen::Vector6d f_star_zero_a;
  Eigen::Vector3d f_ext_a, m_ext_a, m_d;

  T_wa = origin * T_7a; //T_WA
  
  f_ext_a =  T_wa.linear().inverse()*f_ext.head<3>();
  m_ext_a =  T_wa.linear().inverse()*f_ext.tail<3>();

  asm_dir = Eigen::Vector3d::UnitZ();

  
  // theta = angle*sin(2*M_PI/duration*(t-t_0));
  theta = dyros_math::cubic(t, t_0, t_0 + duration, 0.0, angle, 0.0, 0.0); 
  
  p_init_a = T_7a.inverse().translation();
  l = sqrt(pow(p_init_a(0),2) + pow(p_init_a(1),2));
  p_cmd_a << l*cos(theta), l*sin(theta), p_init_a(2);
  p_cur_a = (T_wa.inverse() * current).translation();
  v_cur_a = T_wa.linear().transpose() * xd.head<3>();
  
  f_motion = kp*(p_cmd_a - p_cur_a) - kv*v_cur_a;
  f_asm = PegInHole::pressCubicEE(origin, current, xd, T_wa, force, t, t_0, 5.0, 0.0, 100.0, 0.0);  

  f_a.head<2>() = f_motion.head<2>();
  f_a.tail<2>() = f_asm.tail<2>();

  r_init_a = T_7a.inverse().linear();
  r_target_a = dyros_math::rotateWithZ(angle)*r_init_a;
  r_cmd_a = dyros_math::rotationCubic(t, t_0, t_0 + duration, r_init_a, r_target_a);
  r_cur_a = (T_wa.inverse()*current).linear();
  w_cur_a = T_wa.linear().inverse() * xd.tail<3>();
  delphi_delta = -0.5 * dyros_math::getPhi(r_cur_a, r_cmd_a);
  
  m_motion = kwp * delphi_delta + kwv * (-w_cur_a);

  m_d << 0.0, moment, 0.0;  
  m_d = m_d + T_7a.translation().cross(f_asm);
  m_asm = 0.8 * (m_d - m_ext_a) - 0.01 * xd.tail<3>(); // + 0.01*retreat_stage_force_error_.tail<3>();

  m_a.head<2>() = m_asm.head<2>();
  m_a.tail<1>() = m_motion.tail<1>();

  // std::cout<<"================================="<<std::endl;
  // std::cout<<"l        : "<< l<<std::endl;
  // std::cout<<"theta : "<< theta*RAD2DEG<<std::endl;
  // std::cout<<"cross : "<< (T_7a.translation().cross(f_asm)).transpose()<<std::endl;
  // std::cout<<"const : " << 0.0<< ","<< moment<<","<< 0.0<<std::endl;
  // std::cout<<"m_d   : "<<m_d.transpose()<<std::endl;
  // std::cout<<"m_a   : "<< m_a.transpose()<<std::endl;

  f_star_zero_a<< f_a, m_a;

  return f_star_zero_a;
}