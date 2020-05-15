#include <assembly_dual_controllers/utils/control/peg_in_hole_base.h>

Eigen::Vector3d PegInHole::straightMove(const Eigen::Vector3d origin,
                                        const Eigen::Vector3d current_position,
                                        const Eigen::Matrix<double, 6, 1> current_velocity,
                                        const int dir,
                                        const double speed,
                                        const double current_time,
                                        const double init_time)
{
  // double desent_speed = -0.02; //-0.005; // 5cm/s

  Eigen::Vector3d desired_position;
  Eigen::Vector3d desired_linear_velocity;
  Eigen::Vector3d f_star;
  Eigen::Matrix3d K_p;
  Eigen::Matrix3d K_v;

  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 100, 0, 0, 0, 100, 0, 0, 0, 100;

  desired_position = origin;
  desired_position(dir) = origin(dir) + speed * (current_time - init_time);

  desired_linear_velocity.setZero();
  desired_linear_velocity(dir) = speed;

  f_star = K_p * (desired_position - current_position) + K_v * (desired_linear_velocity - current_velocity.head<3>());

  return f_star;
}

Eigen::Vector3d PegInHole::oneDofMove(const Eigen::Vector3d origin,
                                      const Eigen::Vector3d current_position,
                                      const double target_position,
                                      const Eigen::Matrix<double, 6, 1> current_velocity,
                                      const double current_time,
                                      const double init_time,
                                      const double duration,
                                      const double desired_speed, //only positive value!!!
                                      const int direction)        // 0 -> x, 1 -> y, 2 -> z //DESIRED DIRECTION!!
{
  double speed;
  Eigen::Vector3d desired_position;
  Eigen::Vector3d desired_velocity;
  Eigen::Vector3d f_star;
  Eigen::Matrix3d K_p;
  Eigen::Matrix3d K_v;

  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

  if (origin(direction) < target_position)
    speed = desired_speed;
  else
    speed = -desired_speed;

  if (direction == 0)
  {
    desired_position(0) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(0), target_position, 0, 0);
    desired_position.tail<2>() = origin.tail<2>();
    desired_velocity << speed, 0, 0;
  }
  if (direction == 1.0)
  {
    desired_position(0) = origin(0);
    desired_position(1) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(1), target_position, 0, 0);
    desired_position(2) = origin(2);
    desired_velocity << 0, speed, 0;
  }
  if (direction == 2.0)
  {
    desired_position.head<2>() = origin.head<2>();
    desired_position(2) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(2), target_position, 0, 0);
    desired_velocity << 0, 0, speed;
  }

  f_star = K_p * (desired_position - current_position) + K_v * (desired_velocity - current_velocity.head<3>());

  return f_star;
}

Eigen::Vector3d PegInHole::twoDofMove(const Eigen::Vector3d origin,
                                      const Eigen::Vector3d current_position,
                                      const Eigen::Vector3d target_position,
                                      const Eigen::Matrix<double, 6, 1> current_velocity,
                                      const double current_time,
                                      const double init_time,
                                      const double duration,
                                      const double desired_speed,
                                      const int direction) // 0 -> x, 1 -> y, 2 -> z //NOT DESIRED DIRECTION!!
{
  Eigen::Vector3d speed; // x, y, z
  Eigen::Vector3d desired_position;
  Eigen::Vector3d desired_velocity;
  Eigen::Vector3d f_star;
  Eigen::Matrix3d K_p;
  Eigen::Matrix3d K_v;

  K_p << 5500, 0, 0, 0, 5500, 0, 0, 0, 5500;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

  for (size_t i = 0; i < 3; i++)
  {
    if (origin(i) < target_position(i))
      speed(i) = desired_speed;
    else
      speed(i) = -desired_speed;
  }

  if (direction == 0.0) //NOT WANT TO MOVE ALONG X - AXIS
  {
    desired_position(0) = target_position(0);
    desired_position(1) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(1), target_position(1), 0, 0);
    desired_position(2) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(2), target_position(2), 0, 0);
    desired_velocity << 0, speed(1), speed(2);
  }
  if (direction == 1.0)
  {
    desired_position(0) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(0), target_position(0), 0, 0);
    desired_position(1) = target_position(1);
    desired_position(2) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(2), target_position(2), 0, 0);
    desired_velocity << speed(0), 0, speed(2);
  }
  if (direction == 2.0) //NOT WANT TO MOVE ALONG Z - AXIS
  {
    desired_position(0) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(0), target_position(0), 0, 0);
    desired_position(1) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(1), target_position(1), 0, 0);
    desired_position(2) = target_position(2);
    desired_velocity << speed(0), speed(1), 0;
  }

  f_star = K_p * (desired_position - current_position) + K_v * (desired_velocity - current_velocity.head<3>());
  // std::cout<<"-------------------------------------------"<<std::endl;
  // std::cout<<"target_position : "<<target_position.transpose()<<std::endl;
  // std::cout<<"current_position : "<<current_position.transpose()<<std::endl;
  // std::cout<<"desired speed : "<<desired_velocity.transpose()<<std::endl;
  // std::cout<<"current speed : "<<current_velocity.head<3>().transpose()<<std::endl;
  // std::cout<<"f_star : "<<f_star.transpose()<<std::endl;
  return f_star;
}

Eigen::Vector3d PegInHole::oneDofMoveEE(const Eigen::Vector3d origin,
                                        const Eigen::Matrix3d init_rot,
                                        const Eigen::Vector3d current_position,
                                        const Eigen::Matrix<double, 6, 1> current_velocity,
                                        const double current_time,
                                        const double init_time,
                                        const double duration,
                                        const double target_distance, // + means go forward, - mean go backward
                                        const int direction)          // 0 -> x_ee, 1 -> y_ee, 2 -> z_ee //DESIRED DIRECTION W.R.T END EFFECTOR!!
{
  Eigen::Vector3d goal_position;
  Eigen::Vector3d cmd_position;
  Eigen::Vector3d f_star;
  Eigen::Matrix3d K_p;
  Eigen::Matrix3d K_v;
  double theta; //atfer projection the init_rot onto the global frame, the theta means yaw angle difference.

  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

  // start from EE
  for (int i = 0; i < 3; i++)
  {
    if (i == direction)
      goal_position(i) = target_distance;
    else
      goal_position(i) = 0.0;
  }

  goal_position = origin + init_rot * goal_position;

  for (int i = 0; i < 3; i++)
  {
    cmd_position(i) = dyros_math::cubic(current_time, init_time, init_time + duration, origin(i), goal_position(i), 0, 0);
  }

  f_star = K_p * (cmd_position - current_position) + K_v * (-current_velocity.head<3>());

  return f_star;
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

Eigen::Matrix<double, 3, 1> PegInHole::generateSpiralWithRotation(const Eigen::Matrix3d initial_rotation_M,
                                                                  const Eigen::Matrix3d rotation_M,
                                                                  const Eigen::Matrix<double, 3, 1> current_angular_velocity,
                                                                  const double current_time,
                                                                  const double init_time,
                                                                  const double duration,
                                                                  const double direction, //0 = the first motion, 1 = CCW, 2 == CW
                                                                  const double axis)      // 0 = x-axis, 1 = y-axis, 2 = z-axis
{
  double target_angle = 3.0 * M_PI / 180;
  double ori_change_theta;

  Eigen::Matrix3d rotation_matrix;
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;

  if (direction == 0.0)
  {
    ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration / 2, 0, target_angle, 0, 0);
  }

  if (direction == 1.0)
  {
    ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, target_angle, -target_angle, 0, 0);
  }

  if (direction == 2.0)
  {
    ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, -target_angle, target_angle, 0, 0);
  }

  if (axis == 0)
    rotation_matrix << 1, 0, 0, 0, cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta);
  if (axis == 1)
    rotation_matrix << cos(ori_change_theta), 0, sin(ori_change_theta), 0, 1, 0, -sin(ori_change_theta), 0, cos(ori_change_theta);
  if (axis == 2)
    rotation_matrix << cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta), 0, 0, 0, 1;

  target_rotation_M = rotation_matrix * initial_rotation_M;

  delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * 200.0 * delphi_delta + 5.0 * (-current_angular_velocity);

  return m_star;
}

Eigen::Matrix<double, 3, 1> PegInHole::generateSpiralWithRotationGainEe(const Eigen::Matrix3d initial_rotation_M,
                                                                        const Eigen::Matrix3d rotation_M,
                                                                        const Eigen::Matrix<double, 3, 1> current_angular_velocity,
                                                                        const double current_time,
                                                                        const double init_time,
                                                                        const double duration,
                                                                        const double direction, //0 = the first motion, 1 = CCW, 2 == CW
                                                                        const double axis,      // 0 = x-axis, 1 = y-axis, 2 = z-axis
                                                                        const double kp,
                                                                        const double kv)
{
  double target_angle = 3.0 * M_PI / 180;
  double ori_change_theta;

  Eigen::Matrix3d rotation_matrix;
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;
  Eigen::Matrix3d K_p;
  Eigen::Matrix3d K_v;

  K_p << kp, 0, 0, 0, kp, 0, 0, 0, kp;
  K_v << kv, 0, 0, 0, kv, 0, 0, 0, kv;

  if (direction == 0.0)
  {
    ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration / 2, 0, target_angle, 0, 0);
  }

  if (direction == 1.0)
  {
    ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, target_angle, -target_angle, 0, 0);
  }

  if (direction == 2.0)
  {
    ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, -target_angle, target_angle, 0, 0);
  }

  if (axis == 0)
    rotation_matrix << 1, 0, 0, 0, cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta);
  if (axis == 1)
    rotation_matrix << cos(ori_change_theta), 0, sin(ori_change_theta), 0, 1, 0, -sin(ori_change_theta), 0, cos(ori_change_theta);
  if (axis == 2)
    rotation_matrix << cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta), 0, 0, 0, 1;

  target_rotation_M = initial_rotation_M * rotation_matrix;

  delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * K_p * delphi_delta + K_v * (-current_angular_velocity);

  return m_star;
}

Eigen::Matrix<double, 3, 1> PegInHole::generateSpiralWithRotationEe(const Eigen::Matrix3d initial_rotation_M,
                                                                    const Eigen::Matrix3d rotation_M,
                                                                    const Eigen::Matrix<double, 3, 1> current_angular_velocity,
                                                                    const double current_time,
                                                                    const double init_time,
                                                                    const double duration,
                                                                    const double direction, //0 = the first motion, 1 = CCW, 2 == CW
                                                                    const double axis,
                                                                    const int rand) // 0 = x-axis, 1 = y-axis, 2 = z-axis
{
  double target_angle = 1.5 * M_PI / 180;
  double ori_change_theta;

  Eigen::Matrix3d rotation_matrix;
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;

  if (direction == 0.0)
  {
    ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration / 2, 0, target_angle, 0, 0);
  }

  if (direction == 1.0)
  {
    ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, target_angle, -target_angle, 0, 0);
  }

  if (direction == 2.0)
  {
    ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration, -target_angle, target_angle, 0, 0);
  }

  if (direction == 3.0)
  {
    ori_change_theta = dyros_math::cubic(current_time, init_time, init_time + duration / 2, 0, -target_angle, 0, 0);
  }

  if (axis == 0)
    rotation_matrix << 1, 0, 0, 0, cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta);
  if (axis == 1)
    rotation_matrix << cos(ori_change_theta), 0, sin(ori_change_theta), 0, 1, 0, -sin(ori_change_theta), 0, cos(ori_change_theta);
  if (axis == 2)
    rotation_matrix << cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta), 0, 0, 0, 1;

  target_rotation_M = initial_rotation_M * rotation_matrix;

  delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * 200.0 * delphi_delta + 5.0 * (-current_angular_velocity);

  return m_star;
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

Eigen::Vector3d PegInHole::keepCurrentOrientation(const Eigen::Matrix3d initial_rotation_M,
                                                  const Eigen::Matrix3d rotation_M,
                                                  const Eigen::Matrix<double, 6, 1> current_velocity,
                                                  const double kp,
                                                  const double kd)
{
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;

  delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, initial_rotation_M);

  m_star = (1.0) * kp * delphi_delta + kd * (-current_velocity.tail<3>());

  return m_star;
}

Eigen::Vector3d PegInHole::rotateWithGlobalAxis(const Eigen::Matrix3d initial_rotation_M,
                                                const Eigen::Matrix3d rotation_M,
                                                const Eigen::Matrix<double, 6, 1> current_velocity,
                                                const double goal,
                                                const double init_time,
                                                const double current_time,
                                                const double end_time,
                                                const int dir) // 0 = x-axis, 1 = y-axis, 2 = z-axis
{
  Eigen::Matrix3d rot;
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;

  double theta;
  double run_time;
  double duration = (end_time - init_time) / 4.0; //0.5s

  run_time = current_time - init_time;

  if (run_time < duration)
  {
    theta = dyros_math::cubic(current_time, init_time, init_time + duration, 0, goal, 0, 0);
  }

  else if (duration <= run_time && run_time < 3 * duration)
  {
    theta = dyros_math::cubic(current_time, init_time + duration, init_time + 3 * duration, goal, -goal, 0, 0);
  }

  else // if(3*duration/2 <= run_time && run_time < 2*duration)
  {
    theta = dyros_math::cubic(current_time, init_time + 3 * duration, init_time + 4 * duration, -goal, 0, 0, 0);
  }

  if (dir == 0)
    rot << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
  if (dir == 1)
    rot << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);
  if (dir == 2)
    rot << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;

  target_rotation_M = rot * initial_rotation_M;

  delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * 200.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"time: "<<run_time<<std::endl;
  // std::cout<<"duration : "<<duration<<std::endl;
  // std::cout<<"theta: "<<theta*180/M_PI<<std::endl;
  return m_star;
}

Eigen::Vector3d PegInHole::rotateWithEeAxis(const Eigen::Matrix3d initial_rotation_M,
                                            const Eigen::Matrix3d rotation_M,
                                            const Eigen::Matrix<double, 6, 1> current_velocity,
                                            const double goal,
                                            const double init_time,
                                            const double current_time,
                                            const double end_time,
                                            const int dir) // 0 = x-axis, 1 = y-axis, 2 = z-axis
{
  Eigen::Matrix3d rot;
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;

  double theta;
  double run_time;
  double duration = (end_time - init_time); //0.5s

  run_time = current_time - init_time;

  //if(run_time <= duration)
  //{
  theta = dyros_math::cubic(current_time, init_time, init_time + duration, 0, goal, 0, 0);
  //}

  if (dir == 0)
    rot << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
  if (dir == 1)
    rot << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);
  if (dir == 2)
    rot << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;

  target_rotation_M = initial_rotation_M * rot;

  delphi_delta = -0.5 * dyros_math::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * 200.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

  // std::cout<<"----------------------"<<std::endl;
  // std::cout<<"time: "<<run_time<<std::endl;
  // std::cout<<"duration : "<<duration<<std::endl;
  // std::cout<<"theta: "<<theta*180/M_PI<<std::endl;
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

  if (abs(o1(2)) >= abs(o2(2)))
    tilt_axis = tilt_axis;
  else
    tilt_axis = -tilt_axis;

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
                                      const Eigen::Ref<const Eigen::Vector6d>& xd, const Eigen::Isometry3d T_ea,
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

  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

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
  m_star = (1.0) * 250.0 * delphi_delta + 5.0 * (-xd.tail<3>());

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

  return f_star_zero;
}

// bool::judgeHeavyMass(const double t, const double )

Eigen::Vector3d PegInHole::straightMoveEE(const Eigen::Isometry3d &origin,
                                          const Eigen::Isometry3d &current,
                                          const Eigen::Ref<const Eigen::Vector6d>& xd,
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
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

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
Eigen::Vector3d PegInHole::threeDofMove(const Eigen::Isometry3d &origin,
                                        const Eigen::Isometry3d &current,
                                        const Eigen::Vector3d target,
                                        const Eigen::Ref<const Eigen::Vector6d>& xd,
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
                                           const Eigen::Ref<const Eigen::Vector6d>& xd,
                                           const double kp,
                                           const double kv,
                                           const double kp_o,
                                           const double kv_o,
                                           const Eigen::Ref<const Eigen::Matrix6d>& lambda)
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
                                                  const Eigen::Ref<const Eigen::Vector6d>& xd,
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
                                               const Eigen::Ref<const Eigen::Vector6d>& xd,
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

Eigen::Vector3d PegInHole::pressEE(const Eigen::Isometry3d &origin,
                                   const Eigen::Isometry3d &current,
                                   const Eigen::Ref<const Eigen::Vector6d>& xd,
                                   const double force,
                                   const Eigen::Isometry3d &T_ea) //the direction where a peg is inserted, wrt {E} .i.e., T_ga
{
  Eigen::Vector3d f_motion, f_asm, f_a; // wrt {A}

  Eigen::Vector3d p_cmd_a, p_cur_a, p_cur_w, p_cmd_w;
  Eigen::Vector3d v_cur_a;

  Eigen::Matrix3d K_p, K_v;
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d asm_dir;

  K_p = Eigen::Matrix3d::Identity() * 5000;
  K_v = Eigen::Matrix3d::Identity() * 200;
  asm_dir = T_ea.linear().col(2);

  T_wa = origin * T_ea; //T_WA

  p_cmd_a.setZero();

  p_cmd_a = T_ea.inverse().translation();

  // p_cur_w = (current * T_ea).translation() - T_wa.translation(); // T_we * T_ea - p_wa
  // p_cur_a = T_wa.linear().transpose() * p_cur_w;

  p_cur_a = (T_wa.inverse() * current).translation();

  v_cur_a = T_wa.linear().transpose() * xd.head<3>();

  f_motion = K_p * (p_cmd_a - p_cur_a) + K_v * (-v_cur_a); // <fx, fy, fz>
  f_asm << 0.0, 0.0, force;                                //<0, 0, fz>

  f_a(0) = f_motion(0);
  f_a(1) = f_motion(1);
  f_a(2) = f_asm(2);

  // std::cout<<"--------------------------"<<std::endl;
  // std::cout<<"f_a: "<<f_a.transpose()<<std::endl;
  // std::cout<<"p_cmd_a: "<<p_cmd_a.transpose()<<std::endl;
  // std::cout<<"p_cur_a: "<<p_cur_a.transpose()<<std::endl;

  return f_a;
}

Eigen::Vector3d PegInHole::generateSpiralEE(const Eigen::Isometry3d &origin,
                                            const Eigen::Isometry3d &current,
                                            const Eigen::Ref<const Eigen::Vector6d>& xd,
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

  K_p = Eigen::Matrix3d::Identity() * 5000;
  K_v = Eigen::Matrix3d::Identity() * 200;
  asm_dir = T_ea.linear().col(2);

  start_point.setZero();

  T_wa = origin * T_ea; //T_WA

  traj = dyros_math::spiral(t, t_0, t_0 + duration, start_point, lin_vel, pitch);

  p_init_a = T_ea.inverse().translation();

  p_cmd_a << traj(0), traj(1), 0.0;
  p_cmd_a = p_init_a + p_cmd_a;

  p_cur_w = (current * T_ea).translation() - T_wa.translation(); // T_we * T_ea - p_wa
  // p_cur_a = T_wa.linear().transpose() * p_cur_w;
  p_cur_a = (T_wa.inverse() * current).translation();

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

Eigen::Vector3d PegInHole::rotateWithAseemblyDir(const Eigen::Isometry3d &origin,
                                                 const Eigen::Isometry3d &current,
                                                 const Eigen::Ref<const Eigen::Vector6d>& xd,
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
                                         const Eigen::Ref<const Eigen::Vector6d>& xd,
                                         const Eigen::Matrix3d target_rot, //wrt {E}
                                         const double t,
                                         const double t_0,
                                         const double duration) //axis_vector wrt EE frame
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d m_star;
  Eigen::Vector3d delphi_delta;
  Eigen::Matrix3d rot_init, rot, desired_rot;

  rot_init = origin.linear();
  desired_rot = dyros_math::rotationCubic(t, t_0, t_0 + duration, rot_init, target_rot);
  rot = current.linear();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, desired_rot);

  m_star = (1.0) * 200.0 * delphi_delta + 5.0 * (-xd.tail<3>());

  return m_star;
}

Eigen::Vector3d PegInHole::generateWiggleMotionEE(const Eigen::Isometry3d &origin,
                                                  const Eigen::Isometry3d &current,
                                                  const Eigen::Isometry3d &T_ea,
                                                  const Eigen::Ref<const Eigen::Vector6d>& xd,
                                                  const double angle,
                                                  const double w, //rad/s
                                                  const double t,
                                                  const double t_0)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d x_axis, y_axis;
  Eigen::Vector3d m_a, delphi_delta;
  Eigen::Matrix3d rot_init, rot;
  Eigen::Matrix3d target_rot, target_rot_x, target_rot_y;
  Eigen::Vector3d v_cur_w, v_cur_a;

  T_wa = origin * T_ea;

  x_axis = Eigen::Vector3d::UnitX();
  y_axis = Eigen::Vector3d::UnitY();

  double t_e;
  double theta1,theta2;

  t_e = t - t_0;
  theta1 = angle * sin(w * t_e);
  theta2 = angle * cos(w*t_e);

  rot_init = T_wa.linear().transpose() * origin.linear(); //initial {E} wrt {A}

  target_rot_x = dyros_math::angleaxis2rot(x_axis, theta1) * rot_init; //wrt {A}
  target_rot_y = dyros_math::angleaxis2rot(y_axis, theta2) * rot_init; //wrt {A}
  target_rot = target_rot_x * target_rot_y;

  rot = T_wa.linear().transpose() * current.linear();

  v_cur_a = T_wa.linear().transpose() * xd.tail<3>();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, target_rot);

  m_a = (1.0) * 50.0 * delphi_delta + 0.25 * (-v_cur_a);

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

Eigen::Vector3d PegInHole::generateTwistSearchMotionEE(const Eigen::Isometry3d &origin,
                                                       const Eigen::Isometry3d &current,
                                                       const Eigen::Isometry3d &T_ea,
                                                       const Eigen::Ref<const Eigen::Vector6d>& xd,
                                                       const double angle,
                                                       const double t,
                                                       const double t_0,
                                                       const double duration)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d axis;
  Eigen::Vector3d m_a;
  Eigen::Vector3d delphi_delta;
  Eigen::Matrix3d rot_init, rot, target_rot;
  Eigen::Vector3d v_cur_w, v_cur_a;
  double t_e, w;
  double theta;

  T_wa = origin * T_ea;

  axis = Eigen::Vector3d::UnitZ();

  t_e = t - t_0;
  w = 2 * M_PI / duration;

  theta = angle * sin(w * t_e);

  rot_init = T_wa.linear().transpose() * origin.linear();         //initial {E} wrt {A}
  target_rot = dyros_math::angleaxis2rot(axis, theta) * rot_init; //wrt {A}
  rot = T_wa.linear().transpose() * current.linear();

  v_cur_a = T_wa.linear().transpose() * xd.tail<3>();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, target_rot);

  m_a = (1.0) * 200.0 * delphi_delta + 5.0 * (-v_cur_a);

  // Eigen::Vector3d euler_init, euler, euler_goal;
  // euler_init = dyros_math::rot2Euler(rot_init);
  // euler = dyros_math::rot2Euler(rot);
  // euler_goal = dyros_math::rot2Euler(target_rot);

  // std::cout<<"-------------------------------------"<<std::endl;
  // std::cout<<"rot_init: \n"<<rot_init<<std::endl;
  // std::cout<<"run time: "<<t - t_0<< std::endl;
  // std::cout<<"euler_init: "<<euler_init.transpose()*180/M_PI<<std::endl;
  // std::cout<<"euler_goal: "<<euler_goal.transpose()*180/M_PI<<std::endl;
  // std::cout<<"euler: "<<euler.transpose()*180/M_PI<<std::endl;
  // std::cout<<"m_a: "<<m_a.transpose()<<std::endl;

  return m_a;
}

Eigen::Vector3d PegInHole::generateRotationSearchMotionEE(const Eigen::Isometry3d &origin,
                                                          const Eigen::Isometry3d &current,
                                                          const Eigen::Isometry3d &T_ea,
                                                          const Eigen::Ref<const Eigen::Vector6d>& xd,
                                                          const double angle,
                                                          const double t,
                                                          const double t_0,
                                                          const double duration)
{
  Eigen::Isometry3d T_wa;
  Eigen::Vector3d axis;
  Eigen::Vector3d m_a;
  Eigen::Vector3d delphi_delta;
  Eigen::Matrix3d rot_init, rot, target_rot;
  Eigen::Vector3d v_cur_w, v_cur_a;
  double theta;

  T_wa = origin * T_ea;

  axis = Eigen::Vector3d::UnitZ();

  theta = dyros_math::cubic(t, t_0, t_0 + duration, 0.0, angle, 0.0, 0.0);

  rot_init = T_wa.linear().transpose() * origin.linear();         //initial {E} wrt {A}
  target_rot = dyros_math::angleaxis2rot(axis, theta) * rot_init; //wrt {A}
  rot = T_wa.linear().transpose() * current.linear();

  v_cur_a = T_wa.linear().transpose() * xd.tail<3>();

  delphi_delta = -0.5 * dyros_math::getPhi(rot, target_rot);

  m_a = (1.0) * 200.0 * delphi_delta + 5.0 * (-v_cur_a);

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
