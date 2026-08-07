// SPDX-License-Identifier: BSD-3-Clause

#ifndef POINT_LIO_MANIFOLD_STATE_HPP
#define POINT_LIO_MANIFOLD_STATE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <manif/SO3.h>

namespace point_lio
{

class Rotation3d
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Rotation3d() : value_(manif::SO3d::Identity()) {}
  Rotation3d(const manif::SO3d & value) : value_(value) {}
  Rotation3d(const Eigen::Quaterniond & quaternion) : value_(quaternion.normalized()) {}
  Rotation3d(const Eigen::Matrix3d & rotation)
  : value_(Eigen::Quaterniond(rotation).normalized())
  {
  }

  Rotation3d & operator=(const Eigen::Matrix3d & rotation)
  {
    value_ = manif::SO3d(Eigen::Quaterniond(rotation).normalized());
    return *this;
  }

  Rotation3d & operator=(const Eigen::Quaterniond & quaternion)
  {
    value_ = manif::SO3d(quaternion.normalized());
    return *this;
  }

  Eigen::Matrix3d matrix() const { return value_.rotation(); }
  Eigen::Matrix3d transpose() const { return value_.rotation().transpose(); }
  Eigen::Quaterniond quaternion() const { return value_.quat(); }

  template <typename Derived>
  auto operator*(const Eigen::MatrixBase<Derived> & value) const
  {
    return (value_.rotation() * value).eval();
  }

  Eigen::Matrix3d operator-() const { return -value_.rotation(); }
  double operator()(Eigen::Index row, Eigen::Index column) const
  {
    return value_.rotation()(row, column);
  }

  void retract(const Eigen::Vector3d & delta)
  {
    value_ = value_.rplus(manif::SO3Tangentd(delta));
  }

  Eigen::Vector3d minus(const Rotation3d & reference) const
  {
    return value_.rminus(reference.value_).coeffs();
  }

  const manif::SO3d & manifold() const { return value_; }

private:
  manif::SO3d value_;
};

using Vector3d = Eigen::Vector3d;

struct Input
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector3d acc = Vector3d::Zero();
  Vector3d gyro = Vector3d::Zero();
};

struct InputState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using scalar = double;
  static constexpr int DOF = 24;

  Vector3d pos = Vector3d::Zero();
  Rotation3d rot;
  Rotation3d offset_R_L_I;
  Vector3d offset_T_L_I = Vector3d::Zero();
  Vector3d vel = Vector3d::Zero();
  Vector3d bg = Vector3d::Zero();
  Vector3d ba = Vector3d::Zero();
  Vector3d gravity = Vector3d::Zero();

  void retract(const Eigen::Matrix<double, DOF, 1> & delta)
  {
    pos += delta.segment<3>(0);
    rot.retract(delta.segment<3>(3));
    offset_R_L_I.retract(delta.segment<3>(6));
    offset_T_L_I += delta.segment<3>(9);
    vel += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
    gravity += delta.segment<3>(21);
  }

  Eigen::Matrix<double, DOF, 1> minus(const InputState & reference) const
  {
    Eigen::Matrix<double, DOF, 1> delta;
    delta.segment<3>(0) = pos - reference.pos;
    delta.segment<3>(3) = rot.minus(reference.rot);
    delta.segment<3>(6) = offset_R_L_I.minus(reference.offset_R_L_I);
    delta.segment<3>(9) = offset_T_L_I - reference.offset_T_L_I;
    delta.segment<3>(12) = vel - reference.vel;
    delta.segment<3>(15) = bg - reference.bg;
    delta.segment<3>(18) = ba - reference.ba;
    delta.segment<3>(21) = gravity - reference.gravity;
    return delta;
  }
};

struct OutputState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using scalar = double;
  static constexpr int DOF = 30;

  Vector3d pos = Vector3d::Zero();
  Rotation3d rot;
  Rotation3d offset_R_L_I;
  Vector3d offset_T_L_I = Vector3d::Zero();
  Vector3d vel = Vector3d::Zero();
  Vector3d omg = Vector3d::Zero();
  Vector3d acc = Vector3d::Zero();
  Vector3d gravity = Vector3d::Zero();
  Vector3d bg = Vector3d::Zero();
  Vector3d ba = Vector3d::Zero();

  void retract(const Eigen::Matrix<double, DOF, 1> & delta)
  {
    pos += delta.segment<3>(0);
    rot.retract(delta.segment<3>(3));
    offset_R_L_I.retract(delta.segment<3>(6));
    offset_T_L_I += delta.segment<3>(9);
    vel += delta.segment<3>(12);
    omg += delta.segment<3>(15);
    acc += delta.segment<3>(18);
    gravity += delta.segment<3>(21);
    bg += delta.segment<3>(24);
    ba += delta.segment<3>(27);
  }

  Eigen::Matrix<double, DOF, 1> minus(const OutputState & reference) const
  {
    Eigen::Matrix<double, DOF, 1> delta;
    delta.segment<3>(0) = pos - reference.pos;
    delta.segment<3>(3) = rot.minus(reference.rot);
    delta.segment<3>(6) = offset_R_L_I.minus(reference.offset_R_L_I);
    delta.segment<3>(9) = offset_T_L_I - reference.offset_T_L_I;
    delta.segment<3>(12) = vel - reference.vel;
    delta.segment<3>(15) = omg - reference.omg;
    delta.segment<3>(18) = acc - reference.acc;
    delta.segment<3>(21) = gravity - reference.gravity;
    delta.segment<3>(24) = bg - reference.bg;
    delta.segment<3>(27) = ba - reference.ba;
    return delta;
  }
};

template <typename State>
Eigen::Vector3d transformPointToWorld(
  const State & state, const Eigen::Vector3d & point_lidar, bool estimate_extrinsics,
  const Eigen::Matrix3d & fixed_lidar_rotation, const Eigen::Vector3d & fixed_lidar_translation)
{
  Eigen::Vector3d point_imu;
  if (estimate_extrinsics) {
    point_imu = state.offset_R_L_I * point_lidar + state.offset_T_L_I;
  } else {
    point_imu = fixed_lidar_rotation * point_lidar + fixed_lidar_translation;
  }
  return state.rot * point_imu + state.pos;
}

}  // namespace point_lio

using vect3 = Eigen::Vector3d;
using vect2 = Eigen::Vector2d;
using vect1 = Eigen::Matrix<double, 1, 1>;
using SO3 = point_lio::Rotation3d;
using input_ikfom = point_lio::Input;
using state_input = point_lio::InputState;
using state_output = point_lio::OutputState;

#endif  // POINT_LIO_MANIFOLD_STATE_HPP
