
struct FrankaDataContainer {
  std::shared_ptr<franka_hw::FrankaStateHandle>
      state_handle_;  ///< To read to complete robot state.
  std::shared_ptr<franka_hw::FrankaModelHandle>
      model_handle_;  ///< To have access to e.g. jacobians.
  std::vector<hardware_interface::JointHandle> joint_handles_;  ///< To command joint torques.

  double filter_params_{0.005};       ///< [-] PT1-Filter constant to smooth target values set
                                      ///< by dynamic reconfigure servers (stiffness/damping)
                                      ///< or interactive markers for the target poses.
  double nullspace_stiffness_{20.0};  ///< [Nm/rad] To track the initial joint configuration in
                                      ///< the nullspace of the Cartesian motion.
  double nullspace_stiffness_target_{20.0};  ///< [Nm/rad] Unfiltered raw value.
  const double delta_tau_max_{1.0};          ///< [Nm/ms] Maximum difference in joint-torque per
                                             ///< timestep. Used to saturated torque rates to ensure
                                             ///< feasible commands.

  std::shared_ptr<FrankaModelUpdater> franka_model_updater_;
  // robot state
  Eigen::Matrix<double, 7, 7> mass_matrix_;
  Eigen::Matrix<double, 7, 1> coriolis_;
  Eigen::Matrix<double, 7, 1> q_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Matrix<double, 7, 1> qd_;

  Vector7d q_desired_;
  Vector7d qd_desired_;
  Vector7d qdd_desired_;

  Eigen::Affine3d transform_init_;


  Eigen::Vector3d position_d_;                              ///< Target position of the endeffector.
  Eigen::Quaterniond orientation_d_;         ///< Target orientation of the endeffector.
  Eigen::Vector3d position_d_target_;        ///< Unfiltered raw value.
  Eigen::Quaterniond orientation_d_target_;  ///< Unfiltered raw value.

};
