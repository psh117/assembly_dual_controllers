#pragma once

#include <memory>

#include <rbdl/rbdl.h>
#include <Eigen/Dense>

#include "assembly_dual_controllers/robot_model/robot_model.h"


class FrankaPandaModel : public RobotModel
{
public:
  static constexpr int kDof=7;
  static constexpr double kGravity=9.8;

  FrankaPandaModel();

  Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd &q) const override;
  Eigen::Vector3d getTranslation(const Eigen::VectorXd &q) const override;
  Eigen::Matrix3d getRotation(const Eigen::VectorXd &q) const override;
  Eigen::Isometry3d getTransform(const Eigen::VectorXd &q) const override;
  Eigen::MatrixXd getJointLimit() const override;
  Eigen::VectorXd getInitialConfiguration() const override;

  int getDof() override;

private:
  void initModel();

  RigidBodyDynamics::Math::Vector3d com_position_[kDof];
  RigidBodyDynamics::Math::Vector3d ee_position_;
  Eigen::Vector3d joint_posision_[kDof];

  std::shared_ptr<RigidBodyDynamics::Model> model_;
  unsigned int body_id_[kDof];
  RigidBodyDynamics::Body body_[kDof];
  RigidBodyDynamics::Joint joint_[kDof];
};