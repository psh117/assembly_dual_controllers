#include <assembly_dual_controllers/utils/model/franka_model_updater.h>


const double FrankaModelUpdater::PRINT_RATE = 0.4;

FrankaModelUpdater::FrankaModelUpdater() {
  initialize();
}

FrankaModelUpdater::FrankaModelUpdater(
std::shared_ptr<franka_hw::FrankaModelHandle> model_handle, 
std::shared_ptr<franka_hw::FrankaStateHandle> state_handle) :
model_handle_(model_handle), state_handle_(state_handle)
{ 
  initialize();
  updateModel(); 
}

void FrankaModelUpdater::initialize()
{
  qd_.setZero(); 
  q_limit_center_ << 0.0, 0.0, 0.0, -M_PI_2, 0.0, 1.8675, 0.0;

  Eigen::Matrix4d::Map(F_T_EE_.data()) << 0.707106781, 0.707106781, 0, 0,
                                        -0.707106781, 0.707106781, 0, 0,
                                        0, 0, 1 ,0,
                                        0, 0, 0, 1;
  Eigen::Matrix4d::Map(EE_T_K_.data()) = Eigen::Matrix4d::Identity();
}
void FrankaModelUpdater::updateModel()
{
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kFlange);

  std::array<double, 49> massmatrix_array = model_handle_->getMass();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 7> gravity_array = model_handle_->getGravity();

  mass_matrix_ = Eigen::Map<const Eigen::Matrix<double, 7, 7>>(massmatrix_array.data());
  coriolis_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
  q_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
  Eigen::Matrix<double, 7, 1> qd_now = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
  
  for(int i=0; i<7; i++)
  {
    qd_(i) = franka::lowpassFilter(0.001, qd_now(i), qd_(i), 150.0);
  }

  jacobian_ = Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
  tau_measured_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J.data());
  tau_desired_read_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());
  gravity_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(gravity_array.data());

  transform_ = Eigen::Matrix4d::Map(model_handle_->getPose(franka::Frame::kEndEffector).data());
  // transform_.translation() += transform_.linear() * Eigen::Vector3d(0,0,-0.103);
  position_ = transform_.translation();
  rotation_ = transform_.rotation();
  
  xd_ = jacobian_ * qd_;

  lambda_matrix_ = (jacobian_ * mass_matrix_.inverse() * jacobian_.transpose()).inverse();
  jacobian_bar_ = mass_matrix_.inverse()*jacobian_.transpose()*lambda_matrix_;
  null_projector_ = Eigen::Matrix<double, 7,7>::Identity() - jacobian_.transpose() * jacobian_bar_.transpose();
  f_measured_ = jacobian_bar_.transpose()*(tau_measured_ - gravity_);

  printState();
}

void FrankaModelUpdater::setTorque(const Eigen::Matrix<double,7,1> &torque_command, bool idle_control)
{
  for(int i=0; i<7; ++i)
  {
    joint_handles_[i].setCommand(torque_command(i));
  }
  if (!idle_control)
  {
    target_updated_ = true;
  }
}

void FrankaModelUpdater::setInitialValues()
{
  initial_transform_ = transform_;
  initial_q_ = q_;
}


void FrankaModelUpdater::setInitialValues(const Eigen::Ref<const Eigen::VectorXd> &q)
{
  std::array<double, 7> q_array;
  std::array<double, 16> B_T_EE;

  initial_q_ = q;
  Eigen::Matrix<double, 7, 1>::Map(q_array.data()) = q;
  B_T_EE = model_handle_->getPose(franka::Frame::kEndEffector, q_array, F_T_EE_, EE_T_K_);
  initial_transform_ = Eigen::Matrix4d::Map(B_T_EE.data());
  std::cout << "initial_transform_: \n" << initial_transform_.matrix() << std::endl;
  std::cout << Eigen::Quaterniond(initial_transform_.linear()).coeffs() << std:: endl;
}

void FrankaModelUpdater::printState()
{
  // static ros::Time prev_time = ros::Time::now();
  // ros::Duration dt = ros::Time::now() - prev_time;
  // prev_time = ros::Time::now();
  // std::cout << dt.toSec() << std::endl;
  if (++print_count_ < 1000 * PRINT_RATE)
    return;
  Eigen::Quaterniond qt_init (initial_transform_.linear());
  Eigen::Quaterniond qt_cur (transform_.linear());
  std::cout << "-----------------------------\n"
    << "angle error: " << qt_init.angularDistance(qt_cur) << std::endl
    << "translation error: " << (initial_transform_.translation() - position_).norm() << std::endl
    << "initial_transform_: \n" << initial_transform_.matrix() << std::endl
    << "transform_: \n" << transform_.matrix() << std::endl
    << "qt_init: " << qt_init.coeffs().transpose() << std::endl
    << "qt_cur: " << qt_cur.coeffs().transpose() << std::endl;
  print_count_ = 0;
}
