// SPDX-License-Identifier: BSD-3-Clause

#ifndef POINT_LIO_NARROW_ESIKF_HPP
#define POINT_LIO_NARROW_ESIKF_HPP

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <manif/SO3.h>

#include <algorithm>
#include <cmath>

namespace point_lio::esikf
{

template <typename Scalar>
struct MeasurementData
{
  bool valid = true;
  bool converge = false;
  Scalar M_Noise = Scalar(1);
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> z;
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> h_x;
  Eigen::Matrix<Scalar, 6, 1> z_IMU = Eigen::Matrix<Scalar, 6, 1>::Zero();
  Eigen::Matrix<Scalar, 6, 1> R_IMU = Eigen::Matrix<Scalar, 6, 1>::Ones();
  bool satu_check[6] = {false, false, false, false, false, false};
};

template <typename State, int ProcessNoiseDof, typename Input = State>
class NarrowESIKF
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kDof = State::DOF;
  static_assert(ProcessNoiseDof == kDof, "The narrow filter expects full-state process noise");

  using scalar_type = typename State::scalar;
  using Vector = Eigen::Matrix<scalar_type, kDof, 1>;
  using Covariance = Eigen::Matrix<scalar_type, kDof, kDof>;
  using ProcessNoise = Eigen::Matrix<scalar_type, ProcessNoiseDof, ProcessNoiseDof>;
  using ProcessModel = Vector(State &, const Input &);
  using ProcessJacobian = Covariance(State &, const Input &);
  using PointMeasurementModel = void(
    State &, Eigen::Matrix3d, Eigen::Matrix3d, MeasurementData<scalar_type> &);
  using ImuMeasurementModel = void(State &, MeasurementData<scalar_type> &);

  NarrowESIKF(
    const State & state = State(), const Covariance & covariance = Covariance::Identity())
  : P_(covariance), x_(state)
  {
  }

  void init_dyn_share_modified_2h(
    ProcessModel process_model, ProcessJacobian process_jacobian,
    PointMeasurementModel point_measurement)
  {
    process_model_ = process_model;
    process_jacobian_ = process_jacobian;
    point_measurement_ = point_measurement;
    imu_measurement_ = nullptr;
  }

  void init_dyn_share_modified_3h(
    ProcessModel process_model, ProcessJacobian process_jacobian,
    PointMeasurementModel point_measurement, ImuMeasurementModel imu_measurement)
  {
    process_model_ = process_model;
    process_jacobian_ = process_jacobian;
    point_measurement_ = point_measurement;
    imu_measurement_ = imu_measurement;
  }

  void predict(
    double & dt, ProcessNoise & process_noise, const Input & input, bool predict_state,
    bool propagate_covariance)
  {
    if (process_model_ == nullptr || process_jacobian_ == nullptr) {
      return;
    }

    if (predict_state) {
      const Vector derivative = process_model_(x_, input);
      x_.retract(derivative * scalar_type(dt));
    }

    if (propagate_covariance) {
      const Vector derivative = process_model_(x_, input);
      const Covariance jacobian = process_jacobian_(x_, input);
      Covariance transition = Covariance::Identity() + jacobian * scalar_type(dt);
      updateRotationRows(transition, jacobian, derivative, dt, 3);
      updateRotationRows(transition, jacobian, derivative, dt, 6);
      P_ = transition * P_ * transition.transpose() +
           process_noise * scalar_type(dt * dt);
      symmetrize(P_);
    }
  }

  bool update_iterated_dyn_share_modified()
  {
    if (point_measurement_ == nullptr) {
      return false;
    }

    const State prior_state = x_;
    const Covariance prior_covariance = P_;
    State iterate = prior_state;
    Vector delta = Vector::Zero();
    Eigen::Matrix<scalar_type, kDof, Eigen::Dynamic> gain;
    Eigen::Matrix<scalar_type, Eigen::Dynamic, kDof> full_jacobian;
    scalar_type measurement_variance = scalar_type(0);
    bool accepted = false;

    for (int iteration = 0; iteration < maximum_iterations_; ++iteration) {
      MeasurementData<scalar_type> measurement;
      point_measurement_(
        iterate, prior_covariance.template block<3, 3>(0, 0),
        prior_covariance.template block<3, 3>(3, 3), measurement);
      if (!measurement.valid) {
        return false;
      }
      if (!validPointMeasurement(measurement)) {
        return false;
      }

      measurement_variance = measurement.M_Noise;
      full_jacobian = Eigen::Matrix<scalar_type, Eigen::Dynamic, kDof>::Zero(
        measurement.h_x.rows(), kDof);
      full_jacobian.leftCols(measurement.h_x.cols()) = measurement.h_x;

      if (!computeGain(prior_covariance, full_jacobian, measurement_variance, gain)) {
        return false;
      }

      const auto linearized_residual = measurement.z + full_jacobian * delta;
      const Vector next_delta = gain * linearized_residual;
      State next = prior_state;
      next.retract(next_delta);
      iterate = next;
      accepted = true;

      if ((next_delta - delta).template lpNorm<Eigen::Infinity>() < convergence_tolerance_) {
        delta = next_delta;
        break;
      }
      delta = next_delta;
    }

    if (!accepted) {
      return false;
    }

    x_ = iterate;
    const Covariance identity = Covariance::Identity();
    const Covariance residual_projection = identity - gain * full_jacobian;
    Covariance posterior =
      residual_projection * prior_covariance * residual_projection.transpose() +
      measurement_variance * gain * gain.transpose();
    transportCovariance(delta, posterior);
    P_ = posterior;
    return true;
  }

  void update_iterated_dyn_share_IMU()
  {
    if (imu_measurement_ == nullptr || kDof < 30) {
      return;
    }

    MeasurementData<scalar_type> measurement;
    imu_measurement_(x_, measurement);
    Eigen::Matrix<scalar_type, 6, kDof> jacobian =
      Eigen::Matrix<scalar_type, 6, kDof>::Zero();
    Eigen::Matrix<scalar_type, 6, 6> noise =
      Eigen::Matrix<scalar_type, 6, 6>::Zero();
    for (int row = 0; row < 6; ++row) {
      noise(row, row) = std::max(measurement.R_IMU(row), minimum_variance_);
      if (!measurement.satu_check[row]) {
        jacobian(row, 15 + row) = scalar_type(1);
        jacobian(row, 24 + row) = scalar_type(1);
      }
    }

    const Eigen::Matrix<scalar_type, 6, 6> innovation_covariance =
      jacobian * P_ * jacobian.transpose() + noise;
    Eigen::LDLT<Eigen::Matrix<scalar_type, 6, 6>> solver(innovation_covariance);
    if (solver.info() != Eigen::Success) {
      return;
    }
    const Eigen::Matrix<scalar_type, kDof, 6> gain =
      solver.solve(jacobian * P_).transpose();
    if (solver.info() != Eigen::Success || !gain.allFinite()) {
      return;
    }

    const Vector correction = gain * measurement.z_IMU;
    x_.retract(correction);
    const Covariance identity = Covariance::Identity();
    const Covariance residual_projection = identity - gain * jacobian;
    Covariance posterior = residual_projection * P_ * residual_projection.transpose() +
                           gain * noise * gain.transpose();
    transportCovariance(correction, posterior);
    P_ = posterior;
  }

  void change_x(State & state) { x_ = state; }
  void change_P(Covariance & covariance)
  {
    P_ = covariance;
    symmetrize(P_);
  }

  const State & get_x() const { return x_; }
  const Covariance & get_P() const { return P_; }

  Covariance P_;
  State x_;

private:
  static void symmetrize(Covariance & covariance)
  {
    covariance =
      (scalar_type(0.5) * (covariance + covariance.transpose())).eval();
  }

  static void updateRotationRows(
    Covariance & transition, const Covariance & process_jacobian, const Vector & derivative,
    double dt, int offset)
  {
    const Eigen::Vector3d increment = -derivative.template segment<3>(offset) * dt;
    const manif::SO3Tangentd tangent(increment);
    transition.template block<3, kDof>(offset, 0) =
      tangent.ljac() * process_jacobian.template block<3, kDof>(offset, 0) * dt;
    transition.template block<3, 3>(offset, offset) += tangent.exp().rotation();
  }

  static void transportCovariance(const Vector & correction, Covariance & covariance)
  {
    Covariance transport = Covariance::Identity();
    transport.template block<3, 3>(3, 3) =
      manif::SO3Tangentd(correction.template segment<3>(3)).rjac();
    transport.template block<3, 3>(6, 6) =
      manif::SO3Tangentd(correction.template segment<3>(6)).rjac();
    covariance = transport * covariance * transport.transpose();
    symmetrize(covariance);
  }

  bool validPointMeasurement(const MeasurementData<scalar_type> & measurement) const
  {
    return measurement.h_x.rows() > 0 && measurement.h_x.cols() > 0 &&
           measurement.h_x.cols() <= kDof && measurement.z.rows() == measurement.h_x.rows() &&
           std::isfinite(measurement.M_Noise) && measurement.M_Noise > minimum_variance_ &&
           measurement.h_x.allFinite() && measurement.z.allFinite();
  }

  static bool computeGain(
    const Covariance & prior_covariance,
    const Eigen::Matrix<scalar_type, Eigen::Dynamic, kDof> & jacobian,
    scalar_type measurement_variance,
    Eigen::Matrix<scalar_type, kDof, Eigen::Dynamic> & gain)
  {
    Eigen::LDLT<Covariance> prior_solver(prior_covariance);
    if (prior_solver.info() != Eigen::Success) {
      return false;
    }
    const Covariance prior_information = prior_solver.solve(Covariance::Identity());
    if (prior_solver.info() != Eigen::Success || !prior_information.allFinite()) {
      return false;
    }

    const scalar_type inverse_variance = scalar_type(1) / measurement_variance;
    Covariance information =
      prior_information + inverse_variance * jacobian.transpose() * jacobian;
    information =
      (scalar_type(0.5) * (information + information.transpose())).eval();
    Eigen::LDLT<Covariance> information_solver(information);
    if (information_solver.info() != Eigen::Success) {
      return false;
    }
    gain = information_solver.solve(inverse_variance * jacobian.transpose());
    return information_solver.info() == Eigen::Success && gain.allFinite();
  }

  ProcessModel * process_model_ = nullptr;
  ProcessJacobian * process_jacobian_ = nullptr;
  PointMeasurementModel * point_measurement_ = nullptr;
  ImuMeasurementModel * imu_measurement_ = nullptr;
  int maximum_iterations_ = 4;
  scalar_type convergence_tolerance_ = scalar_type(1e-5);
  static constexpr scalar_type minimum_variance_ = scalar_type(1e-12);
};

}  // namespace point_lio::esikf

#endif  // POINT_LIO_NARROW_ESIKF_HPP
