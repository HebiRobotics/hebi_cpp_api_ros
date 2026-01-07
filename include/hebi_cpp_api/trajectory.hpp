#pragma once

#include "hebi.h"

#include <memory>
#include <vector>

#include "Eigen/Eigen"
#include "hebi_cpp_api/util.hpp"

namespace hebi {
namespace trajectory {

/**
 * \brief Represents a smooth trajectory through a set of waypoints.
 */
class Trajectory final {
private:
  /**
   * C-style trajectory objects (one for each module)
   */
  std::vector<HebiTrajectoryPtr> trajectories_;

  /**
   * The number of modules controlled by this trajectory.
   */
  const size_t number_of_joints_;

  /**
   * The number of waypoints in this trajectory.
   */
  const size_t number_of_waypoints_;

  /**
   * The time at which the trajectory starts (seconds).
   */
  const double start_time_;

  /**
   * The time at which the trajectory ends (seconds).
   */
  const double end_time_;

  /**
   * Creates a Trajectory from a list of the underlying C-style objects.
   */
  Trajectory(std::vector<HebiTrajectoryPtr> trajectories, size_t number_of_waypoints, double start_time,
             double end_time);

  /**
   * \brief Estimates the time required to move between waypoints based on
   * the given positions, maximum velocities, and maximum accelerations.
   *
   * \param positions A matrix of waypoint joint positions. The number of rows
   * should be equal to the number of joints, and the number of columns equal
   * to the number of waypoints. Values of +/-infinity are not allowed.
   * \param max_velocities A vector of maximum velocities for each joint.
   * The size of this vector should match the number of joints.
   * \param max_accelerations A vector of maximum accelerations for each joint.
   * The size of this vector should match the number of joints.
   * \param params A HebiTimeEstimationParams object specifying the method and
   * parameters for time estimation based on the method chosen.
   * \param min_segment_time Minimum allowed segment duration. Any estimated time
   * below this value will be clipped. Must be a finite non-negative value.
   *
   * \returns A vector of estimated segment times, or throws an exception if
   * the input matrix is empty or contains invalid values.
   */
  static Eigen::VectorXd estimateSegmentTimes(const Eigen::MatrixXd& positions, const Eigen::VectorXd& max_velocities,
                                              const Eigen::VectorXd& max_accelerations,
                                              const HebiTimeEstimationParams& params, double min_segment_time = 0.01);

public:
  /**
   * \brief Creates a smooth trajectory through a set of waypoints (position
   * velocity and accelerations defined at particular times). This trajectory
   * wrapper object can create multi-dimensional trajectories (i.e., multiple
   * joints moving together using the same time reference).
   *
   * \param time_vector A vector of desired times at which to reach each
   * waypoint; this must be defined (and not NAN for any element).
   * \param positions A matrix of waypoint joint positions (in SI units). The
   * number of rows should be equal to the number of joints, and the number of
   * columns equal to the number of waypoints.  Any elements that are NAN will
   * be considered free parameters when solving for a trajectory. Values of
   * +/-infinity are not allowed.
   * \param velocities An optional matrix of velocity constraints at the
   * corresponding waypoints; should either be nullptr or matching the size of
   * the positions matrix. Any elements that are NAN will be considered free
   * parameters when solving for a trajectory. Values of +/-infinity are not
   * allowed.
   * \param accelerations An optional matrix of acceleration constraints at
   * the corresponding waypoints; should either be nullptr or matching the
   * size of the positions matrix. Any elements that are NAN will be
   * considered free parameters when solving for a trajectory. Values of
   * +/-infinity are not allowed.
   *
   * \returns A HebiTrajectory object if there were no errors, and the
   * trajectory has been created. An empty shared_ptr indicates that there was
   * an error in creating the trajectory.
   */
  static std::shared_ptr<Trajectory> createUnconstrainedQp(const Eigen::VectorXd& time_vector,
                                                           const Eigen::MatrixXd& positions,
                                                           const Eigen::MatrixXd* velocities = nullptr,
                                                           const Eigen::MatrixXd* accelerations = nullptr);

  /**
   * \brief Creates a constant velocity trajectory with smooth ramp-up and ramp-down phases.
   *
   * This function generates a multi-joint trajectory that follows a "jerkazoidal" velocity
   * profile - similar to a trapezoidal velocity profile but with minimum-jerk ramp-up and
   * ramp-down phases instead of constant acceleration. This produces smoother motion by
   * maintaining a constant velocity between start and end positions, with smooth acceleration
   * and deceleration phases at the beginning and end.
   *
   * The trajectory consists of 2 to 4 waypoints depending on the ramp times:
   * 1. Start position (with specified initial velocity and acceleration)
   * 2. End of ramp-up phase (included only if start_ramp_time is sufficiently large)
   * 3. Start of ramp-down phase (included only if constant velocity segment exists)
   * 4. End position (with specified final velocity and acceleration)
   *
   * All joints are synchronized to finish at the same time. The function computes the
   * maximum constant velocity segment time required across all joints and uses this
   * value for all joints to ensure coordination.
   *
   * \param start_position Initial position for each joint (in SI units). Must have the
   * same size as all other vector parameters.
   * \param end_position Final position for each joint (in SI units). Must have the
   * same size as all other vector parameters.
   * \param const_speed Desired constant speed for each joint during the constant velocity
   * phase (in SI units per second). All values must be positive and finite.
   * \param start_ramp_time Duration of the initial acceleration phase (in seconds).
   * If very small or zero, the trajectory starts at constant velocity with no ramp-up.
   * \param end_ramp_time Duration of the final deceleration phase (in seconds). Must
   * be positive and finite.
   * \param start_velocities Optional initial velocity for each joint at the start of the trajectory
   * (in SI units per second). If nullptr, defaults to zero velocity. Must be finite if provided.
   * \param end_velocities Optional final velocity for each joint at the end of the trajectory
   * (in SI units per second). If nullptr, defaults to zero velocity. Must be finite if provided.
   * \param start_accelerations Optional initial acceleration for each joint at the start of the
   * trajectory (in SI units per second squared). If nullptr, defaults to zero acceleration. Must be finite if provided.
   * \param end_accelerations Optional final acceleration for each joint at the end of the
   * trajectory (in SI units per second squared). If nullptr, defaults to zero acceleration. Must be finite if provided.
   *
   * \returns A shared pointer to a Trajectory object representing the constant velocity
   * trajectory.
   *
   * \throws std::invalid_argument If any of the following conditions are met:
   * - Input vectors have mismatched sizes
   * - Any input value is non-finite (NaN or infinity)
   * - Any constant speed is not positive
   * - end_ramp_time is not positive
   * - The computed constant velocity segment time is negative (indicates invalid parameters)
   *
   * \throws std::runtime_error If the underlying C trajectory creation fails.
   */
  static std::shared_ptr<Trajectory> createJerkazoidal(const Eigen::VectorXd& start_position,
                                                       const Eigen::VectorXd& end_position,
                                                       const Eigen::VectorXd& const_speed,
                                                       double start_ramp_time, double end_ramp_time,
                                                       const Eigen::VectorXd* start_velocities = nullptr,
                                                       const Eigen::VectorXd* end_velocities = nullptr,
                                                       const Eigen::VectorXd* start_accelerations = nullptr,
                                                       const Eigen::VectorXd* end_accelerations = nullptr);

  /**
   * \brief Overload of createJerkazoidal for singular variable. See above for implementation details.
   */
  static std::shared_ptr<Trajectory> createJerkazoidal(double start_position, double end_position, double const_speed,
                                                       double start_ramp_time, double end_ramp_time,
                                                       double start_velocity = 0.0, double end_velocity = 0.0,
                                                       double start_acceleration = 0.0, double end_acceleration = 0.0);

  /**
   * \brief Destructor cleans up resources for trajectory.
   */
  ~Trajectory() noexcept;

  /**
   * \brief The number of independent position trajectories over the same time
   * domain that are managed by this object.
   */
  size_t getJointCount() const { return number_of_joints_; }

  /**
   * \brief The number of fixed waypoints that each joint is moving through.
   */
  size_t getWaypointCount() const { return number_of_waypoints_; }

  /**
   * \brief Get the time (in seconds) at which the defined trajectory begins.
   */
  double getStartTime() const { return start_time_; }

  /**
   * \brief Get the time (in seconds) at which the defined trajectory ends.
   */
  double getEndTime() const { return end_time_; }

  /**
   * \brief The time (in seconds) between the start and end of this
   * trajectory.
   */
  double getDuration() const;

  /**
   * \brief Returns the position, velocity, and acceleration for a given
   * point in time along the trajectory.
   *
   * \param time The time for which the trajectory state is being queried.
   * This should be between the start and end of the trajectory.
   * \param position If not nullptr, this vector is filled in with the
   * position along the trajectory for each joint at the given time.
   * \param velocity If not nullptr, this vector is filled in with the
   * velocity along the trajectory for each joint at the given time.
   * \param acceleration If not nullptr, this vector is filled in with the
   * acceleration along the trajectory for each joint at the given time.
   *
   * \returns True if the state was successfully retrieved for all joints,
   * false if there was an error.
   */
  bool getState(double time, Eigen::VectorXd* position, Eigen::VectorXd* velocity, Eigen::VectorXd* acceleration) const;

  /**
   * \brief Returns the minimum and maximum positions reached by each joint over
   * the entire trajectory. This can be useful for checking that a trajectory stays
   * within specified bounds.
   *
   * \param min_position This vector is filled in with the minimum position reached
   * by each joint over the entire trajectory.
   * \param max_position This vector is filled in with the maximum position reached
   * by each joint over the entire trajectory.
   *
   * \returns True if the min and max positions were successfully retrieved for
   * all joints, false if there was an error.
   */
  bool getMinMaxPosition(Eigen::VectorXd& min_position, Eigen::VectorXd& max_position);

  /**
   * \brief Returns the maximum absolute velocity reached by each joint over the
   * entire trajectory. This can be useful for checking that a trajectory stays
   * within specified velocity limits.
   *
   * \param max_velocity This vector is filled in with the maximum velocity reached
   * by each joint over the entire trajectory.
   *
   * \returns True if the max velocities were successfully retrieved for all joints,
   * false if there was an error.
   */
  bool getMaxVelocity(Eigen::VectorXd& max_velocity);

  /**
   * \brief Returns the maximum absolute acceleration reached by each joint over the
   * entire trajectory. This can be useful for checking that a trajectory stays
   * within specified acceleration limits.
   *
   * \param max_acceleration This vector is filled in with the maximum acceleration
   * reached by each joint over the entire trajectory.
   *
   * \returns True if the max accelerations were successfully retrieved for all joints,
   * false if there was an error.
   */
  bool getMaxAcceleration(Eigen::VectorXd& max_acceleration);

  /**
   * \brief Rescales a trajectory by a time factor.
   *
   * Values > 1 slow down the trajectory (increase duration), values < 1 speed up
   * the trajectory (decrease duration). The position waypoints remain unchanged,
   * but velocities and accelerations are scaled inversely.
   *
   * \param scale Time scale factor. Must be positive (> 0).
   *
   * \returns True if rescaling was successful, false if the scale factor is invalid (<= 0).
   */
  bool rescale(double scale);

  /**
   * \brief Converts a vector of segment times to a vector of time values at
   * each waypoint. The first element of the vector is the time at which the
   * trajectory starts is always 0.0.
   *
   * \param segment_times A vector of segment times, where each element
   * represents the duration of a segment between waypoints. Values must be
   * finite, i.e., not NaN or +/- infinity.
   *
   * \returns A vector of time values at each waypoint, or throws an exception
   * if the input vector is empty or contains invalid values.
   */
  static Eigen::VectorXd segmentTimesToWaypointTimes(const Eigen::VectorXd& segment_times);

  /**
   * \brief Converts a vector of time values at each waypoint to a vector of
   * segment times.
   *
   * \param time_vector A vector of time values at each waypoint.
   * Values must be finite, i.e., not NaN or +/- infinity.
   *
   * \returns A vector of segment times, or throws an exception if the input
   * vector is empty or contains invalid values.
   */
  static Eigen::VectorXd waypointTimesToSegmentTimes(const Eigen::VectorXd& waypoint_times);

  /**
   * \brief Estimates the time required to move between waypoints using the
   * N-Fabian method. It uses the both distance/v_max and v_max/a_max ratios
   * with a heuristic to determine the segment times.
   *
   * t_est = 2 * distance/v_max * (1 + magic_fabian_constant * v_max/a_max
   *         * exp(-distance/v_max * 2);
   *
   * The method assumes that the segment starts and ends with zero velocity
   * and acceleration.
   *
   * \param positions A matrix of waypoint joint positions. The number of rows
   * should be equal to the number of joints, and the number of columns equal
   * to the number of waypoints. Values of +/-infinity are not allowed.
   * \param max_velocities A vector of maximum velocities for each joint.
   * The size of this vector should match the number of joints.
   * \param max_accelerations A vector of maximum accelerations for each joint.
   * The size of this vector should match the number of joints.
   * \param fabian_constant A constant used in the N-Fabian heuristic.
   * Defaults to 6.5.
   * \param min_segment_time Minimum allowed segment duration. Any estimated time
   * below this value will be clipped. Must be a finite non-negative value.
   * Defaults to 0.01.
   *
   * \returns A vector of estimated segment times, or throws an exception if
   * the input matrix is empty or contains invalid values.
   */
  static Eigen::VectorXd estimateSegmentTimesNFabian(const Eigen::MatrixXd& positions,
                                                     const Eigen::VectorXd& max_velocities,
                                                     const Eigen::VectorXd& max_accelerations,
                                                     double fabian_constant = 6.5, double min_segment_time = 0.01) {
    HebiTimeEstimationParams params;
    params.method = HebiTimeEstimationMethod::HebiTimeEstimationNFabian;
    params.params.n_fabian_params.magic_fabian_constant = fabian_constant;
    return estimateSegmentTimes(positions, max_velocities, max_accelerations, params, min_segment_time);
  }

  /**
   * \brief Estimates the time required to move between waypoints based on the
   * trapezoidal velocity profiles and the maximum velocities and accelerations
   * provided. It calculates the velocity assuming an instantaneous constant
   * acceleration (max_acceleration) and then estimates the time to reach the
   * maximum velocity (v_max) and the time to decelerate to zero velocity.
   *
   * \param positions A matrix of waypoint joint positions. The number of rows
   * should be equal to the number of joints, and the number of columns equal
   * to the number of waypoints. Values of +/-infinity are not allowed.
   * \param max_velocities A vector of maximum velocities for each joint.
   * The size of this vector should match the number of joints.
   * \param max_accelerations A vector of maximum accelerations for each joint.
   * The size of this vector should match the number of joints.
   * \param min_segment_time Minimum allowed segment duration. Any estimated time
   * below this value will be clipped. Must be a finite non-negative value.
   * Defaults to 0.01.
   *
   * \returns A vector of estimated segment times, or throws an exception if
   * the input matrix is empty or contains invalid values.
   */
  static Eigen::VectorXd estimateSegmentTimesTrapezoidal(const Eigen::MatrixXd& positions,
                                                         const Eigen::VectorXd& max_velocities,
                                                         const Eigen::VectorXd& max_accelerations,
                                                         double min_segment_time = 0.01) {
    HebiTimeEstimationParams params;
    params.method = HebiTimeEstimationMethod::HebiTimeEstimationVelocityRamp;
    return estimateSegmentTimes(positions, max_velocities, max_accelerations, params, min_segment_time);
  }

private:
  /**
   * Disable copy and move constructors and assignment operators
   */
  HEBI_DISABLE_COPY_MOVE(Trajectory)
};

} // namespace trajectory
} // namespace hebi
