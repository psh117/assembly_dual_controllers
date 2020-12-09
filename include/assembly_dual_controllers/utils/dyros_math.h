#pragma once

#define DEG2RAD (0.01745329251994329576923690768489)
#define ZCE (1e-5)
// constexpr size_t MAX_DOF=50;

#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <fstream>

#define GRAVITY 9.80665
#define MAX_DOF 50U
#define RAD2DEG 1 / DEG2RAD

namespace Eigen
{

// Eigen default type definition
#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)    \
  typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix; \
  typedef Matrix<Type, Size, 1> Vector##SizeSuffix##TypeSuffix;    \
  typedef Matrix<Type, 1, Size> RowVector##SizeSuffix##TypeSuffix;

typedef double rScalar;

EIGEN_MAKE_TYPEDEFS(rScalar, d, 5, 5)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 6, 6)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 7, 7)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 8, 8)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 12, 12)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 18, 18)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 28, 28)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 30, 30)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 32, 32)

// typedef Transform<rScalar, 3, Eigen::Isometry> HTransform;  // typedef Transform< double, 3, Isometry > 	Eigen::Isometry3d

typedef Matrix<rScalar, 1, 3> Matrix1x3d;
typedef Matrix<rScalar, 1, 4> Matrix1x4d;
typedef Matrix<rScalar, 4, 3> Matrix4x3d;
typedef Matrix<rScalar, 6, 3> Matrix6x3d;
typedef Matrix<rScalar, 6, 7> Matrix6x7d;
typedef Matrix<rScalar, 8, 4> Matrix8x4d;
typedef Matrix<rScalar, 8, 2> Matrix8x2d;
typedef Matrix<rScalar, -1, 1, 0, MAX_DOF, 1> VectorJXd;
typedef Matrix<rScalar, -1, 1, 0, 12, 1> VectorLXd; //Leg IK
typedef Matrix<rScalar, -1, -1, 0, MAX_DOF, MAX_DOF> MatrixJXd;

//Complex
typedef Matrix<std::complex<double>, 8, 4> Matrix8x4cd;

} // namespace Eigen

namespace dyros_math
{
//constexpr double GRAVITY {9.80665};
//constexpr double DEG2RAD {};

struct ImpedanceControllerParameter
{
  Eigen::VectorXd active_command_idx{6};
  Eigen::VectorXd parameter_M{6};
  Eigen::VectorXd parameter_D{6};
  Eigen::VectorXd parameter_K{6};
  Eigen::VectorXd desired_value{6};

  void initialize()
  {
    active_command_idx.setZero();
    parameter_M.setZero();
    parameter_D.setZero();
    parameter_K.setZero();
    desired_value.setZero();
  }
};

struct ImpedanceController
{
  double Ts = 0.001;
  Eigen::VectorXd errorValuePPrev{6};
  Eigen::VectorXd errorValuePrev{6};
  Eigen::VectorXd errorValue{6};

  Eigen::VectorXd impedancePPrev{6};
  Eigen::VectorXd impedancePrev{6};
  Eigen::VectorXd impedance{6};

  ImpedanceControllerParameter param;

  Eigen::VectorXd getImpedance()
  {
    return impedance;
  }

  void initialize()
  {
    errorValuePPrev.setZero();
    errorValuePrev.setZero();
    errorValue.setZero();
    impedancePPrev.setZero();
    impedancePrev.setZero();
    impedance.setZero();
    param.initialize();
  }

  void update(Eigen::VectorXd FValue)
  {

    //update previous values
    errorValuePPrev = errorValuePrev;
    errorValuePrev = errorValue;
    errorValue = FValue - param.desired_value;

    impedancePPrev = impedancePrev;
    impedancePrev = impedance;

    // update impedance
    double d0, d1, d2, n0, n1, n2;
    for (int i = 0; i < 6; i++)
    {
      d0 = 4.0 * param.parameter_M(i) + 2.0 * Ts * param.parameter_D(i) + Ts * Ts * param.parameter_K(i);
      d1 = -8.0 * param.parameter_M(i) + 2.0 * Ts * Ts * param.parameter_K(i);
      d2 = 4.0 * param.parameter_M(i) - 2.0 * Ts * param.parameter_D(i) + Ts * Ts * param.parameter_K(i);

      n0 = Ts * Ts;
      n1 = 2.0 * Ts * Ts;
      n2 = Ts * Ts;

      if (d0 > 1e-8)
        impedance(i) = param.active_command_idx(i) * (n0 * errorValue(i) + n1 * errorValuePrev(i) + n2 * errorValuePPrev(i) - d1 * impedancePrev(i) - d2 * impedancePPrev(i)) / d0;
      else
        impedance(i) = 0.0;

      //  예외처리
      // if(isnan(impedance(i)) || isinf(impedance(i)))
      //   impedance(i) = 0.0;
    }
  }
};

struct LowPassFilter2
{
  // second order low-pass filter
  double CutOffFreq, Ts;
  double zeta, Wn, Den0, Den1, Den2, Num2, Num1, Num0;
  bool b_initialized, b_update;
  // Eigen::Matrix<double, 7, 1>
  Eigen::VectorXd RawDataPrev{7};
  Eigen::VectorXd RawDataPPrev{7};
  Eigen::VectorXd FilteredDataPrev{7};
  Eigen::VectorXd FilteredDataPPrev{7};

  LowPassFilter2(double dCutOffFreq, double dTs) // set cutoff-frequency and (Ts)control period
      : CutOffFreq(dCutOffFreq), Ts(dTs)
  {
    zeta = 0.9;
    Wn = dCutOffFreq * 2.0 * 3.141592;
    b_initialized = false;
    b_update = false;

    RawDataPrev.setZero();
    RawDataPPrev.setZero();
    FilteredDataPrev.setZero();
    FilteredDataPPrev.setZero();

    Den0 = 4.0 + 4.0 * zeta * Ts * Wn + Ts * Wn * Ts * Wn;
    Den1 = 2.0 * Ts * Wn * Ts * Wn - 8.0;
    Den2 = 4.0 - 4.0 * zeta * Ts * Wn + Ts * Wn * Ts * Wn;
    Num2 = Ts * Ts * Wn * Wn;
    Num1 = 2.0 * Ts * Ts * Wn * Wn;
    Num0 = Ts * Ts * Wn * Wn;
  }

  void initialize(double dCutOffFreq, double dTs)
  {
    b_initialized = true;
    CutOffFreq = dCutOffFreq;
    Ts = dTs;
    zeta = 0.9;
    Wn = dCutOffFreq * 2.0 * 3.141592;

    RawDataPrev.setZero();
    RawDataPPrev.setZero();
    FilteredDataPrev.setZero();
    FilteredDataPPrev.setZero();

    Den0 = 4.0 + 4.0 * zeta * Ts * Wn + Ts * Wn * Ts * Wn;
    Den1 = 2.0 * Ts * Wn * Ts * Wn - 8.0;
    Den2 = 4.0 - 4.0 * zeta * Ts * Wn + Ts * Wn * Ts * Wn;
    Num2 = Ts * Ts * Wn * Wn;
    Num1 = 2.0 * Ts * Ts * Wn * Wn;
    Num0 = Ts * Ts * Wn * Wn;
  }

  Eigen::VectorXd update(Eigen::VectorXd RawDataInput)
  {
    if (!b_update)
    {
      RawDataPrev = RawDataInput;
      RawDataPPrev = RawDataInput;
      FilteredDataPrev = RawDataInput;
      FilteredDataPPrev = RawDataInput;
      b_update = true;
    }

    if (b_initialized)
    {
      Eigen::VectorXd FilteredDataReturn(7);
      FilteredDataReturn.setZero();

      for (int i = 0; i < 7; i++)
      {
        FilteredDataReturn(i) = (Num0 * RawDataInput(i) + Num1 * RawDataPrev(i) + Num2 * RawDataPPrev(i) - Den1 * FilteredDataPrev(i) - Den2 * FilteredDataPPrev(i)) / Den0;
        RawDataPPrev(i) = RawDataPrev(i);
        RawDataPrev(i) = RawDataInput(i);
        FilteredDataPPrev(i) = FilteredDataPrev(i);
        FilteredDataPrev(i) = FilteredDataReturn(i);
      }
      return FilteredDataReturn;
    }
    else
      return RawDataInput;
  }
};

double minJerkTraj(double t, double &qinit, double &qtarget, double period);
double minJerkTrajVel(double t, double &qinit, double &qtarget, double period);
double minJerkTrajAcc(double t, double &qinit, double &qtarget, double period);

double cubic(double time,    ///< Current time
             double time_0,  ///< Start time
             double time_f,  ///< End time
             double x_0,     ///< Start state
             double x_f,     ///< End state
             double x_dot_0, ///< Start state dot
             double x_dot_f  ///< End state dot
);
double cubicDot(double time,    ///< Current time
                double time_0,  ///< Start time
                double time_f,  ///< End time
                double x_0,     ///< Start state
                double x_f,     ///< End state
                double x_dot_0, ///< Start state dot
                double x_dot_f  ///< End state dot
);

Eigen::Vector2d spiral(double time,                     ///< Current time
                       double time_0,                   ///< Start time
                       double time_f,                   ///< End time
                       Eigen::Matrix<double, 2, 1> x_0, ///< Start state
                       double line_v,
                       double pitch);

const Eigen::Matrix3d skew(const Eigen::Vector3d &src);


template <int N>
Eigen::Matrix<double, N, 1> cubicVector(double time,     ///< Current time
                                                double time_0,   ///< Start time
                                                double time_f,   ///< End time
                                                Eigen::Matrix<double, N, 1> x_0,      ///< Start state
                                                Eigen::Matrix<double, N, 1> x_f,      ///< End state
                                                Eigen::Matrix<double, N, 1> x_dot_0,  ///< Start state dot
                                                Eigen::Matrix<double, N, 1> x_dot_f   ///< End state dot
    )
{

  Eigen::Matrix<double, N, 1> res;
  for (unsigned int i=0; i<N; i++)
  {
    res(i) = cubic(time, time_0, time_f, x_0(i), x_f(i), x_dot_0(i), x_dot_f(i));
  }
  return res;
}


// Original Paper
// Kang, I. G., and F. C. Park.
// "Cubic spline algorithms for orientation interpolation."
// International journal for numerical methods in engineering 46.1 (1999): 45-64.
const Eigen::Matrix3d rotationCubic(
    double time, double time_0, double time_f,
    const Eigen::Vector3d &w_0, const Eigen::Vector3d &a_0,
    const Eigen::Matrix3d &rotation_0, const Eigen::Matrix3d &rotation_f);

const Eigen::Matrix3d rotationCubic(double time,
                                     double time_0,
                                     double time_f,
                                     const Eigen::Matrix3d &rotation_0,
                                     const Eigen::Matrix3d &rotation_f);

const void rotationQuinticZero(double time,
                               double time_0,
                               double time_f,
                               const Eigen::Matrix3d &rotation_0,
                               const Eigen::Matrix3d &rotation_f,
                               Eigen::Ref<Eigen::Matrix3d> r,
                               Eigen::Ref<Eigen::Vector3d> r_dot,
                               Eigen::Ref<Eigen::Vector3d> r_ddot);

Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation,
                       Eigen::Matrix3d desired_rotation);

Eigen::Matrix3d rotateWithZ(double yaw_angle);
Eigen::Matrix3d rotateWithY(double pitch_angle);
Eigen::Matrix3d rotateWithX(double roll_angle);

Eigen::Vector3d rot2Euler(Eigen::Matrix3d Rot);

template <typename _Matrix_Type_>
_Matrix_Type_ pinv(const _Matrix_Type_ &a, double epsilon =std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

Eigen::MatrixXd discreteRiccatiEquation(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd r, Eigen::MatrixXd q);
Eigen::Vector3d legGetPhi(Eigen::Isometry3d rotation_matrix1, Eigen::Isometry3d active_r1, Eigen::Vector6d ctrl_pos_ori);
Eigen::Vector3d qua2Euler(Eigen::Vector4d quaternion);

void toEulerAngle(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw);

Eigen::Vector3d quinticSpline(
    double time,      ///< Current time
    double time_0,    ///< Start time
    double time_f,    ///< End time
    double x_0,       ///< Start state
    double x_dot_0,   ///< Start state dot
    double x_ddot_0,  ///< Start state ddot
    double x_f,       ///< End state
    double x_dot_f,   ///< End state
    double x_ddot_f); ///< End state ddot

double lowPassFilter(double input, double prev, double ts, double tau);
template <int N>
Eigen::Matrix<double, N, 1> lowPassFilter(Eigen::Matrix<double, N, 1> input, Eigen::Matrix<double, N, 1> prev, double ts, double tau)
{
  Eigen::Matrix<double, N, 1> res;
  for(int i=0; i<N; i++)
  {
    res(i) = lowPassFilter(input(i), prev(i), ts, tau);
  }
  return res;
}

Eigen::Matrix3d angleaxis2rot(Eigen::Vector3d axis_angle_vector, double axis_angle);
Eigen::Matrix3d quat2Rot(const Eigen::Vector4d quat);

//There are some mistakes!
Eigen::Matrix<double, 2, 1> spiral_x(double time,                     ///< Current time
                                     double time_0,                   ///< Start time
                                     double time_f,                   ///< End time
                                     Eigen::Matrix<double, 2, 1> x_0, ///< Start state
                                     double line_v,
                                     double pitch);

Eigen::MatrixXd leastSquareLinear(const std::vector<double> vec, const int interval);

}
