#include "hebi_cpp_api/trajectory.hpp"
#include <stdexcept>

using namespace Eigen;

namespace hebi {
namespace trajectory {

Trajectory::Trajectory(std::vector<HebiTrajectoryPtr> trajectories, size_t number_of_waypoints, double start_time,
                       double end_time)
  : trajectories_(trajectories),
    number_of_joints_(trajectories.size()),
    number_of_waypoints_(number_of_waypoints),
    start_time_(start_time),
    end_time_(end_time) {}

std::shared_ptr<Trajectory> Trajectory::createUnconstrainedQp(const VectorXd& time_vector, const MatrixXd& positions,
                                                              const MatrixXd* velocities,
                                                              const MatrixXd* accelerations) {
  std::shared_ptr<Trajectory> res;

  // Check argument validity
  size_t num_joints = positions.rows();
  size_t num_waypoints = positions.cols();
  if (static_cast<size_t>(time_vector.size()) != num_waypoints)
    return res;
  if (velocities != nullptr && (static_cast<size_t>(velocities->rows()) != num_joints && static_cast<size_t>(velocities->cols()) != num_waypoints))
    return res;
  if (accelerations != nullptr && (static_cast<size_t>(accelerations->rows()) != num_joints && static_cast<size_t>(accelerations->cols()) != num_waypoints))
    return res;
  if (num_waypoints < 2)
    return res;

  // Put data into C-style arrays (handles row/column major matrix representation via
  // Eigen::Map conversion, although this could be optimized if data already in column
  // major form):
  std::vector<double> positions_c(num_joints * num_waypoints);
  double* velocities_c = nullptr;
  double* accelerations_c = nullptr;
  {
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>> tmp(positions_c.data(), num_joints, num_waypoints);
    tmp = positions;
  }
  if (velocities != nullptr) {
    velocities_c = new double[num_joints * num_waypoints];
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>> tmp(velocities_c, num_joints, num_waypoints);
    tmp = *velocities;
  }
  if (accelerations != nullptr) {
    accelerations_c = new double[num_joints * num_waypoints];
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>> tmp(accelerations_c, num_joints, num_waypoints);
    tmp = *accelerations;
  }

  // Build C trajectory objects
  std::vector<HebiTrajectoryPtr> trajectories(num_joints, nullptr);
  for (size_t i = 0; i < num_joints; ++i) {
    HebiTrajectoryPtr trajectory = hebiTrajectoryCreateUnconstrainedQp(
        num_waypoints, (positions_c.data() + i * num_waypoints),
        velocities_c == nullptr ? nullptr : (velocities_c + i * num_waypoints),
        accelerations_c == nullptr ? nullptr : (accelerations_c + i * num_waypoints), time_vector.data());
    // Failure? cleanup previous trajectories
    if (trajectory == nullptr) {
      for (size_t j = 0; j < i; ++j) {
        hebiTrajectoryRelease(trajectories[j]);
      }
      return res;
    }
    trajectories[i] = trajectory;
  }

  if (velocities_c != nullptr)
    delete[] velocities_c;
  if (accelerations_c != nullptr)
    delete[] accelerations_c;

  // Create C++ wrapper
  return std::shared_ptr<Trajectory>(
      new Trajectory(trajectories, num_waypoints, time_vector[0], time_vector[time_vector.size() - 1]));
}

Trajectory::~Trajectory() noexcept {
  for (HebiTrajectoryPtr traj : trajectories_)
    hebiTrajectoryRelease(traj);
}

double Trajectory::getDuration() const {
  // Note -- could use any joint here, as they all have the same time vector
  return hebiTrajectoryGetDuration(trajectories_[0]);
}

bool Trajectory::getState(double time, VectorXd* position, VectorXd* velocity, VectorXd* acceleration) const {
  double tmp_p, tmp_v, tmp_a;
  bool success = true;
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    success = (hebiTrajectoryGetState(trajectories_[i], time, position == nullptr ? &tmp_p : &(*position)[i],
                                      velocity == nullptr ? &tmp_v : &(*velocity)[i],
                                      acceleration == nullptr ? &tmp_a : &(*acceleration)[i]) == 0) &&
              success;
  }
  return success;
}

bool Trajectory::getMinMaxPosition(VectorXd& min_position, VectorXd& max_position) {
  bool success = true;
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    success = (hebiTrajectoryGetMinMaxPosition(trajectories_[i], &min_position[i], &max_position[i]) == 0) &&
               success;
  }
  return success;
}

bool Trajectory::getMaxVelocity(VectorXd& max_velocity) {
  bool success = true;
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    success = (hebiTrajectoryGetMaxVelocity(trajectories_[i], &max_velocity[i]) == 0) &&
               success;
  }
  return success;
}

bool Trajectory::getMaxAcceleration(VectorXd& max_acceleration) {
  bool success = true;
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    success = (hebiTrajectoryGetMaxAcceleration(trajectories_[i], &max_acceleration[i]) == 0) &&
               success;
  }
  return success;
}

bool Trajectory::rescale(double scale) {
  if (scale <= 0.0) {
    return false;
  }
  bool success = true;
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    success = (hebiTrajectoryRescale(trajectories_[i], scale) == HebiStatusSuccess) && success;
  }
  return success;
}

VectorXd Trajectory::segmentTimesToWaypointTimes(const VectorXd& segment_times) {
  if (segment_times.size() < 1) {
    throw std::invalid_argument("At least one segment time is required.");
  }

  VectorXd time_vector(segment_times.size() + 1);
  time_vector[0] = 0.0;
  for (Eigen::Index i = 0; i < segment_times.size(); ++i) {
    if (segment_times[i] <= 0.0 || !std::isfinite(segment_times[i])) {
      throw std::invalid_argument("Segment times must be strictly positive and finite.");
    }
    time_vector[i + 1] = time_vector[i] + segment_times[i];
  }
  return time_vector;
}

VectorXd Trajectory::waypointTimesToSegmentTimes(const VectorXd& waypoint_times) {
  if (waypoint_times.size() < 2) {
    throw std::invalid_argument("At least two waypoint times are required.");
  }
  VectorXd segment_times(waypoint_times.size() - 1);
  for (Eigen::Index i = 0; i < segment_times.size(); ++i) {
    const double diff = waypoint_times[i + 1] - waypoint_times[i];
    if (diff <= 0.0 || !std::isfinite(diff)) {
      throw std::invalid_argument("Waypoint times must be strictly increasing and finite.");
    }
    segment_times[i] = diff;
  }
  return segment_times;
}

Eigen::VectorXd Trajectory::estimateSegmentTimes(const Eigen::MatrixXd& positions,
                                                 const Eigen::VectorXd& max_velocities,
                                                 const Eigen::VectorXd& max_accelerations,
                                                 const HebiTimeEstimationParams& params,
                                                 double min_segment_time) {
  if (positions.rows() == 0 || positions.cols() == 0) {
    throw std::invalid_argument("Position matrix cannot be empty.");
  }
  if (positions.cols() < 2) {
    throw std::invalid_argument("At least two waypoints are required.");
  }
  if (max_velocities.size() != positions.rows()) {
    throw std::invalid_argument("max_velocities size must match number of joints.");
  }
  if (max_accelerations.size() != positions.rows()) {
    throw std::invalid_argument("max_accelerations size must match number of joints.");
  }
  if (!positions.allFinite()) {
    throw std::invalid_argument("Position matrix contains non-finite values.");
  }
  if ((max_velocities.array() <= 0.0).any() || max_velocities.hasNaN()) {
    throw std::invalid_argument("Maximum velocities must be positive numbers.");
  }
  if ((max_accelerations.array() <= 0.0).any() || max_accelerations.hasNaN()) {
    throw std::invalid_argument("Maximum accelerations must be positive numbers.");
  }
  if (min_segment_time < 0.0 || !std::isfinite(min_segment_time)) {
    throw std::invalid_argument("Minimum segment time must be non-negative and finite.");
  }

  size_t num_joints = positions.rows();
  size_t num_waypoints = positions.cols();

  std::vector<double> positions_c(num_joints * num_waypoints);
  {
    Map<Matrix<double, Dynamic, Dynamic, RowMajor>> tmp(positions_c.data(), num_joints, num_waypoints);
    tmp = positions;
  }

  Eigen::VectorXd segment_times(positions.cols() - 1);
  int result = hebiEstimateSegmentTimes(positions_c.data(), max_velocities.data(), max_accelerations.data(),
                                        num_joints, num_waypoints, segment_times.data(), min_segment_time, &params);

  if (result == HebiStatusCode::HebiStatusSuccess) {
    return segment_times;
  } else if (result == HebiStatusCode::HebiStatusInvalidArgument) {
    throw std::invalid_argument("Invalid argument passed to estimateSegmentTimes.");
  } else if (result == HebiStatusCode::HebiStatusArgumentOutOfRange) {
    throw std::out_of_range("Argument out of range in estimateSegmentTimes.");
  } else {
    throw std::runtime_error("Unknown error in estimateSegmentTimes.");
  }
}

std::shared_ptr<Trajectory> Trajectory::createJerkazoidal(const Eigen::VectorXd& start_position,
                                                          const Eigen::VectorXd& end_position,
                                                          const Eigen::VectorXd& const_speed, double start_ramp_time,
                                                          double end_ramp_time, const Eigen::VectorXd* start_velocities,
                                                          const Eigen::VectorXd* end_velocities,
                                                          const Eigen::VectorXd* start_accelerations,
                                                          const Eigen::VectorXd* end_accelerations) {
  const auto num_joints = start_position.size();

  // Validate input sizes
  if (num_joints == 0) {
    throw std::invalid_argument("Input vectors must not be empty.");
  }
  if (end_position.size() != num_joints || const_speed.size() != num_joints ||
      (start_velocities != nullptr && start_velocities->size() != num_joints) ||
      (end_velocities != nullptr && end_velocities->size() != num_joints) ||
      (start_accelerations != nullptr && start_accelerations->size() != num_joints) ||
      (end_accelerations != nullptr && end_accelerations->size() != num_joints)) {
    throw std::invalid_argument("All input vectors must have the same size and must not be empty.");
  }

  // Create zero vectors for optional parameters if not provided
  Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(num_joints);
  const Eigen::VectorXd& start_vels = start_velocities ? *start_velocities : zero_vec;
  const Eigen::VectorXd& end_vels = end_velocities ? *end_velocities : zero_vec;
  const Eigen::VectorXd& start_accels = start_accelerations ? *start_accelerations : zero_vec;
  const Eigen::VectorXd& end_accels = end_accelerations ? *end_accelerations : zero_vec;

  const double MIN_TIME_DELTA = 1E-15;

  // Check if inputs are finite and valid
  if (!std::isfinite(start_ramp_time) || !std::isfinite(end_ramp_time)) {
    throw std::invalid_argument("Ramp times must be finite.");
  }
  if (end_ramp_time < MIN_TIME_DELTA) {
    throw std::invalid_argument("End ramp time must be positive.");
  }
  for (Eigen::Index i = 0; i < num_joints; ++i) {
    if (!std::isfinite(start_position[i]) || !std::isfinite(end_position[i]) || !std::isfinite(const_speed[i]) ||
        !std::isfinite(start_vels[i]) || !std::isfinite(end_vels[i]) ||
        !std::isfinite(start_accels[i]) || !std::isfinite(end_accels[i])) {
      throw std::invalid_argument("All position, velocity, acceleration, and speed values must be finite.");
    }
    // Validate constant speed
    if (const_speed[i] < 1E-6) {
      throw std::invalid_argument("Constant speeds must be positive.");
    }
  }

  // Determine maximum constant velocity segment time across all joints
  double max_const_vel_time = 0.0;
  for (Eigen::Index i = 0; i < num_joints; ++i) {
    double const_vel_time = hebiTrajectoryGetConstantVelocitySegmentTime(start_position[i], end_position[i], const_speed[i],
                                                                         start_ramp_time, end_ramp_time,
                                                                         start_vels[i], end_vels[i],
                                                                         start_accels[i], end_accels[i]);
    if (const_vel_time < 0.0) {
      // Should not happen due to prior checks, but just in case
      throw std::invalid_argument("Invalid parameters for constant velocity trajectory.");
    }
    if (const_vel_time > max_const_vel_time) {
      max_const_vel_time = const_vel_time;
    }
  }

  // Build C trajectory objects using the maximum constant velocity segment time
  // so that all joints finish at the same time and have the same number of waypoints
  std::vector<HebiTrajectoryPtr> trajectories(num_joints, nullptr);
  for (Eigen::Index i = 0; i < num_joints; ++i) {
    HebiTrajectoryPtr trajectory = hebiTrajectoryCreateConstantVelocity(start_position[i], end_position[i],
                                                                        start_ramp_time, max_const_vel_time,
                                                                        end_ramp_time, start_vels[i], end_vels[i],
                                                                        start_accels[i], end_accels[i]);
    // Failure? Cleanup previous trajectories
    if (trajectory == nullptr) {
      for (Eigen::Index j = 0; j < i; ++j) {
        hebiTrajectoryRelease(trajectories[j]);
      }
      throw std::runtime_error("Failed to create constant velocity trajectory.");
    }
    trajectories[i] = trajectory;
  }

  // Compute total duration and number of waypoints
  double max_duration = ((start_ramp_time > MIN_TIME_DELTA) ? start_ramp_time : 0.0) + max_const_vel_time + end_ramp_time;
  double num_waypoints = 4;
  if (start_ramp_time < MIN_TIME_DELTA) --num_waypoints;
  if (max_const_vel_time < MIN_TIME_DELTA) --num_waypoints;

  // Create C++ wrapper
  return std::shared_ptr<Trajectory>(
      new Trajectory(trajectories, num_waypoints, 0.0, max_duration));
}

std::shared_ptr<Trajectory> Trajectory::createJerkazoidal(double start_position, double end_position, double const_speed,
                                                          double start_ramp_time, double end_ramp_time,
                                                          double start_velocity, double end_velocity,
                                                          double start_acceleration, double end_acceleration) {
  const double MIN_TIME_DELTA = 1E-15;

  // Check if inputs are finite and valid
  if (!std::isfinite(start_ramp_time) || !std::isfinite(end_ramp_time)) {
    throw std::invalid_argument("Ramp times must be finite.");
  }
  if (end_ramp_time < MIN_TIME_DELTA) {
    throw std::invalid_argument("End ramp time must be positive.");
  }
  if (!std::isfinite(start_position) || !std::isfinite(end_position) || !std::isfinite(const_speed) ||
      !std::isfinite(start_velocity) || !std::isfinite(end_velocity) ||
      !std::isfinite(start_acceleration) || !std::isfinite(end_acceleration)) {
    throw std::invalid_argument("Position, velocity, acceleration, and speed values must be finite.");
  }
  // Validate constant speed
  if (const_speed < 1E-6) {
    throw std::invalid_argument("Constant speeds must be positive.");
  }

  // Determine maximum constant velocity segment time across all joints
  double const_vel_time = hebiTrajectoryGetConstantVelocitySegmentTime(start_position, end_position, const_speed,
                                                                        start_ramp_time, end_ramp_time,
                                                                        start_velocity, end_velocity,
                                                                        start_acceleration, end_acceleration);
  if (const_vel_time < 0.0) {
    // Should not happen due to prior checks, but just in case
    throw std::invalid_argument("Invalid parameters for constant velocity trajectory.");
  }
  
  // Build C trajectory objects using the maximum constant velocity segment time
  // so that all joints finish at the same time and have the same number of waypoints
  std::vector<HebiTrajectoryPtr> trajectories(1, nullptr);
  HebiTrajectoryPtr trajectory = hebiTrajectoryCreateConstantVelocity(start_position, end_position, 
                                                                      start_ramp_time, const_vel_time,
                                                                      end_ramp_time, start_velocity, end_velocity,
                                                                      start_acceleration, end_acceleration);
  // Failure? Cleanup previous trajectories
  if (trajectory == nullptr) {
    throw std::runtime_error("Failed to create constant velocity trajectory.");
  }
  trajectories[0] = trajectory;

  // Compute total duration and number of waypoints
  double max_duration = ((start_ramp_time > MIN_TIME_DELTA) ? start_ramp_time : 0.0) + const_vel_time + end_ramp_time;
  double num_waypoints = 4;
  if (start_ramp_time < MIN_TIME_DELTA) --num_waypoints;
  if (const_vel_time < MIN_TIME_DELTA) --num_waypoints;

  // Create C++ wrapper
  return std::shared_ptr<Trajectory>(
      new Trajectory(trajectories, num_waypoints, 0.0, max_duration));
}

} // namespace trajectory
} // namespace hebi
