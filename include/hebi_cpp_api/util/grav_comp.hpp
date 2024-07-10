#pragma once

#include "Eigen/Dense"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/robot_model.hpp"

namespace hebi {
namespace util {

/**
 * Small helper function to return gravity vector from a hebi feedback quaternion.
 */
inline Eigen::Vector3d gravityFromQuaternion(const hebi::Quaternionf& q) {
  float xx = q.getX() * q.getX();
  float xz = q.getX() * q.getZ();
  float xw = q.getX() * q.getW();
  float yy = q.getY() * q.getY();
  float yz = q.getY() * q.getZ();
  float yw = q.getY() * q.getW();

  Eigen::Vector3d res;
  res[0] = -2.0f * (xz - yw);
  res[1] = -2.0f * (yz + xw);
  res[2] = -1.0f + 2.0f * (xx + yy);

  return res;
}

/**
 * A helper function to get the torques which approximately balance out the
 * effect of gravity on the arm.
 */
[[deprecated(
    "Use hebi::robot_model::RobotModel::getGravCompEfforts or hebi::Arm GravCompEfforts plug-in "
    "instead.")]] static Eigen::VectorXd
getGravCompEfforts(const hebi::robot_model::RobotModel& model, const Eigen::VectorXd& masses,
                   const hebi::GroupFeedback& feedback) {
  // Update gravity from base module:
  // NOTE: consider using pose-filtered "orientation" feedback instead
  auto base_accel = feedback[0].imu().accelerometer().get();
  Vector3d gravity(-base_accel.getX(), -base_accel.getY(), -base_accel.getZ());

  // Normalize gravity vector (to 1g, or 9.8 m/s^2)
  Eigen::Vector3d normed_gravity = gravity;
  normed_gravity /= normed_gravity.norm();
  normed_gravity *= 9.81;

  size_t num_dof = model.getDoFCount();
  Eigen::VectorXd comp_torque(num_dof);
  comp_torque.setZero();
  model.getGravCompEfforts(feedback.getPosition(), normed_gravity, comp_torque);
  return comp_torque;
}

} // namespace util
} // namespace hebi
