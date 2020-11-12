#include <assembly_dual_controllers/utils/model/franka_model_updater.h>

const double FrankaModelUpdater::PRINT_RATE = 0.001;//0.5;//10.0;

FrankaModelUpdater::FrankaModelUpdater() {
  initialize();
}

#ifdef USE_REAL_ROBOT_INTERFACE
FrankaModelUpdater::FrankaModelUpdater(
std::shared_ptr<franka_hw::FrankaModelHandle> model_handle, 
std::shared_ptr<franka_hw::FrankaStateHandle> state_handle) :
model_handle_(model_handle), state_handle_(state_handle)
{ 
  initialize();
  updateModel(); 
}
#else
FrankaModelUpdater::FrankaModelUpdater(std::shared_ptr<MujocoHelper> mujoco_helper, int arm_num) :
mujoco_helper_(mujoco_helper), arm_num_(arm_num)
{
  initialize();
  updateModel(); 
}
#endif

void FrankaModelUpdater::initialize()
{
  qd_.setZero(); 
  q_limit_center_ << 0.0, 0.0, 0.0, -M_PI_2, 0.0, 1.8675, 0.0;

  Eigen::Matrix4d::Map(F_T_EE_.data()) << 0.707106781, 0.707106781, 0, 0,
                                        -0.707106781, 0.707106781, 0, 0,
                                        0, 0, 1 ,0,
                                        0, 0, 0, 1;
  Eigen::Matrix4d::Map(EE_T_K_.data()) = Eigen::Matrix4d::Identity();
  xd_lpf_.setZero();
  xd_lpf_.head<3>() = 0.1*xd_lpf_.head<3>().setOnes();
  position_lpf_.setZero();
}

void FrankaModelUpdater::updateModel()
{
  double t[30];
  int debug_index = 0;
  sb_.reset();
  Eigen::Matrix<double, 7, 1> qd_now;
#ifdef USE_REAL_ROBOT_INTERFACE
  franka::RobotState robot_state = state_handle_->getRobotState();
  t[debug_index++] = sb_.elapsedAndReset();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kFlange);
  t[debug_index++] = sb_.elapsedAndReset();

  std::array<double, 49> massmatrix_array = model_handle_->getMass();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 7> gravity_array = model_handle_->getGravity();

  mass_matrix_ = Eigen::Map<const Eigen::Matrix<double, 7, 7>>(massmatrix_array.data());
  coriolis_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis_array.data());
  q_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.q.data());
  qd_now = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());

  jacobian_ = Eigen::Map<Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
  tau_measured_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J.data());
  tau_desired_read_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_J_d.data());
  tau_ext_filtered_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.tau_ext_hat_filtered.data());
  gravity_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(gravity_array.data());

  t[debug_index++] = sb_.elapsedAndReset();
  // transform_ = Eigen::Matrix4d::Map(model_handle_->getPose(franka::Frame::kEndEffector).data());
  // transform_ = rbdl_model_.getTransform(q_ + q_offset_);
  // jacobian_ = rbdl_model_.getJacobianMatrix(q_ + q_offset_);
  transform_ = rbdl_model_.getTransform(q_);
  jacobian_ = rbdl_model_.getJacobianMatrix(q_);
  t[debug_index++] = sb_.elapsedAndReset();
#else
  // mass_matrix_ = 
  MujocoModelData md = mujoco_helper_->getModelData(arm_num_);
  mass_matrix_ = md.mass;
  coriolis_.setZero();
  q_ = md.q;
  qd_now = md.qdot;
  jacobian_ = md.jacobian;
  gravity_ = md.gravity;
  transform_ = md.transform;
  tau_measured_ = md.tau_measured;
#endif
  // transform_.translation() += transform_.linear() * Eigen::Vector3d(0,0,-0.103);

  for(int i=0; i<7; i++)
  {
    qd_(i) = franka::lowpassFilter(0.001, qd_now(i), qd_(i), 150.0);
  }

  t[debug_index++] = sb_.elapsedAndReset();
  position_ = transform_.translation();
  rotation_ = transform_.rotation();
  
  xd_ = jacobian_ * qd_;

  t[debug_index++] = sb_.elapsedAndReset();
  lambda_matrix_ = (jacobian_ * mass_matrix_.inverse() * jacobian_.transpose() + 0.001 * Eigen::Matrix<double,6,6>::Identity()).inverse();
  modified_mass_matrix_ = mass_matrix_;
  modified_mass_matrix_(4,4) += 0.1;
  modified_mass_matrix_(5,5) += 0.1;
  modified_mass_matrix_(6,6) += 0.1;
  modified_lambda_matrix_ = (jacobian_ * modified_mass_matrix_.inverse() * jacobian_.transpose() + 0.001 * Eigen::Matrix<double,6,6>::Identity()).inverse();
  jacobian_bar_ = mass_matrix_.inverse()*jacobian_.transpose()*lambda_matrix_;
  null_projector_ = Eigen::Matrix<double, 7,7>::Identity() - jacobian_.transpose() * jacobian_bar_.transpose();
  
  t[debug_index++] = sb_.elapsedAndReset();
  f_ext_ = jacobian_bar_.transpose()*(tau_ext_filtered_);  
  f_measured_ = jacobian_bar_.transpose()*(tau_measured_ - gravity_ - coriolis_);

  t[debug_index++] = sb_.elapsedAndReset();
  manipulability_ = jacobian_ * jacobian_.transpose();
  manipulability_measure_ = std::sqrt(manipulability_.determinant());

  for (int i = 0; i < 6; i++) xd_lpf_(i) = franka::lowpassFilter(0.001, xd_(i), xd_lpf_(i), 1.0);
  for (int i = 0; i < 3; i++) position_lpf_(i) = franka::lowpassFilter(0.001, position_(i), position_lpf_(i), 1.0);

  target_updated_ = false;

  q_out_file_ << q_.transpose() << std::endl;
  x_out_file_ << transform_.translation().transpose() << std::endl;
  
  printState();
  t[debug_index++] = sb_.elapsedAndReset();

  // for(int i=0; i<debug_index; ++i)
  // {
  //   debug_file_td_ << t[i] << '\t';
  // }
  // debug_file_td_ << std::endl;
}

void FrankaModelUpdater::setTorque(const Eigen::Matrix<double,7,1> &torque_command, bool idle_control)
{
#ifdef USE_REAL_ROBOT_INTERFACE
  for(int i=0; i<7; ++i)
  {
    joint_handles_[i].setCommand(torque_command(i));
  }
#else
  mujoco_helper_->control(arm_num_, torque_command);
#endif
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
#ifdef USE_REAL_ROBOT_INTERFACE
  // std::array<double, 7> q_array;
  // std::array<double, 16> B_T_EE;

  // Eigen::Matrix<double, 7, 1>::Map(q_array.data()) = q;
  // B_T_EE = model_handle_->getPose(franka::Frame::kEndEffector, q_array, F_T_EE_, EE_T_K_);
  // initial_transform_ = Eigen::Matrix4d::Map(B_T_EE.data());
  // initial_transform_ = rbdl_model_.getTransform(q + q_offset_);
  initial_transform_ = rbdl_model_.getTransform(q);
#else
  initial_transform_ = mujoco_helper_->getRBDLModel().getTransform(q);
#endif
  initial_q_ = q;

  std::cout << "initial_transform_: \n" << initial_transform_.matrix() << std::endl;
  std::cout << Eigen::Quaterniond(initial_transform_.linear()).coeffs() << std:: endl;
}

void FrankaModelUpdater::setInitialValues(const Eigen::Isometry3d & transform)
{
  initial_transform_ = transform;
  initial_q_ = q_;

  std::cout << "initial_transform_: \n" << initial_transform_.matrix() << std::endl;
  std::cout << Eigen::Quaterniond(initial_transform_.linear()).coeffs() << std:: endl;
}


void FrankaModelUpdater::printState()
{
  // static ros::Time prev_time = ros::Time::now();
  // ros::Duration dt = ros::Time::now() - prev_time;
  // prev_time = ros::Time::now();
  // std::cout << dt.toSec() << std::endl;
  if (++print_count_ < 1000 / PRINT_RATE)
    return;
  Eigen::Quaterniond qt_init (initial_transform_.linear());
  Eigen::Quaterniond qt_cur (transform_.linear());
  Eigen::IOFormat clean_fmt(5, 0, ", ", "\n", "[", "]");
  Eigen::IOFormat clean_fmt_space(5, 0, ", ", "\n                 ", "[", "]");
  std::cout 
  << "---------------------------------------------------------------------------------------\n"
  << "# name         : " << arm_name_ << std::endl
  << "  - q          : " << q_.transpose().format(clean_fmt) << std::endl
  // << "  - gravity    : " << gravity_.transpose().format(clean_fmt) << std::endl
  // << "  - torque     : " << tau_measured_.transpose().format(clean_fmt) << std::endl
  // << "  - manip_msr  : " << manipulability_measure_ << std::endl
  // << "  - manip_jjt  : " << manipulability_.format(clean_fmt_space) << std::endl
  // << "  - f_measured : " << f_measured_filtered_.transpose().format(clean_fmt) << std::endl
  // << "  - mass       : " << mass_matrix_.format(clean_fmt_space) << std::endl
  // << "  - lambda     : " << std::endl << lambda_matrix_.format(clean_fmt) << std::endl
  // << "  - mod_lambda : " << modified_lambda_matrix_.format(clean_fmt_space) << std::endl
  << "  - angle err  : " << qt_init.angularDistance(qt_cur) << std::endl
  << "  - trans err  : " << (initial_transform_.translation() - position_).norm() << std::endl
  << "  - transform 0: " << (initial_transform_).matrix().format(clean_fmt_space) << std::endl
  << "  - transform  : " << (transform_ ).matrix().format(clean_fmt_space) << std::endl
  << "  - quat_init  : " << qt_init.coeffs().transpose().format(clean_fmt) << std::endl
  << "  - quat_cur   : " << qt_cur.coeffs().transpose().format(clean_fmt) << std::endl
  << std::endl;
  print_count_ = 0;
}