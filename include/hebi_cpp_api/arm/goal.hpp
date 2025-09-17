#pragma once

#include <Eigen/Dense>
#include "hebi_cpp_api/trajectory.hpp"

namespace hebi {
namespace arm {

// A class that specifies a goal position of one or more waypoint and/or
// auxilary states.
// Static create methods are provided for various cases; in the case that
// velocities or accelerations are omitted, the default behavior is to leave
// these unconstrained except for a final "0" state.  For aux states, this is
// left as unchanged. For times, this is left to the Arm object to fill in a
// heuristic.
class Goal {

public:
  //////////////////////////////////////////////////////////////////////////////
  // Single waypoint static create functions
  //////////////////////////////////////////////////////////////////////////////

  // Single waypoint, default vel/accel, no time
  static Goal createFromPosition(const Eigen::VectorXd& positions) {
    return Goal(Eigen::VectorXd(0),
                toMatrix(positions),
                nanWithZeroRight(positions.size(), 1),
                nanWithZeroRight(positions.size(), 1),
                Eigen::MatrixXd(0, 0));
  }

  // Single waypoint, default vel/accel
  static Goal createFromPosition(double time, const Eigen::VectorXd& positions) {
    return Goal(toVector(time),
                toMatrix(positions),
                nanWithZeroRight(positions.size(), 1),
                nanWithZeroRight(positions.size(), 1),
                Eigen::MatrixXd(0, 0));
  }

  // Single waypoint, no time
  static Goal createFromWaypoint(const Eigen::VectorXd& positions,
                                 const Eigen::VectorXd& velocities,
                                 const Eigen::VectorXd& accelerations) {
    return Goal(Eigen::VectorXd(0),
                toMatrix(positions),
                toMatrix(velocities),
                toMatrix(accelerations),
                Eigen::MatrixXd(0, 0));
  }

  // Single waypoint
  static Goal createFromWaypoint(double time,
                                 const Eigen::VectorXd& positions,
                                 const Eigen::VectorXd& velocities,
                                 const Eigen::VectorXd& accelerations) {
    return Goal(toVector(time),
                toMatrix(positions),
                toMatrix(velocities),
                toMatrix(accelerations),
                Eigen::MatrixXd(0, 0));
  }

  // Single waypoints + aux state, no time
  static Goal createFromWaypointWithAux(const Eigen::VectorXd& positions,
                                        const Eigen::VectorXd& velocities,
                                        const Eigen::VectorXd& accelerations,
                                        const Eigen::VectorXd& aux) {
    return Goal(Eigen::VectorXd(0),
                toMatrix(positions),
                toMatrix(velocities),
                toMatrix(accelerations),
                toMatrix(aux));
  }

  // Single waypoints + aux state
  static Goal createFromWaypointWithAux(double time,
                                        const Eigen::VectorXd& positions,
                                        const Eigen::VectorXd& velocities,
                                        const Eigen::VectorXd& accelerations,
                                        const Eigen::VectorXd& aux) {
    return Goal(toVector(time),
                toMatrix(positions),
                toMatrix(velocities),
                toMatrix(accelerations),
                toMatrix(aux));
  }

  //////////////////////////////////////////////////////////////////////////////
  // Multiple waypoint static create functions
  //////////////////////////////////////////////////////////////////////////////

  // Multiple waypoints, default vel/accel, no time
  static Goal createFromPositions(const Eigen::MatrixXd& positions) {
    return Goal(Eigen::VectorXd(0),
                positions,
                nanWithZeroRight(positions.rows(), positions.cols()),
                nanWithZeroRight(positions.rows(), positions.cols()),
                Eigen::MatrixXd(0, 0));
  }

  // Multiple waypoints, default vel/accel
  static Goal createFromPositions(const Eigen::VectorXd& times, 
                                  const Eigen::MatrixXd& positions) {
    return Goal(times,
                positions,
                nanWithZeroRight(positions.rows(), positions.cols()),
                nanWithZeroRight(positions.rows(), positions.cols()),
                Eigen::MatrixXd(0, 0));
  }

  // Multiple waypoints, no time
  static Goal createFromWaypoints(const Eigen::MatrixXd& positions,
                                  const Eigen::MatrixXd& velocities,
                                  const Eigen::MatrixXd& accelerations) {
    return Goal(Eigen::VectorXd(0),
                positions,
                velocities,
                accelerations,
                Eigen::MatrixXd(0, 0));
  }

  // Multiple waypoints
  static Goal createFromWaypoints(const Eigen::VectorXd& times,
                                  const Eigen::MatrixXd& positions,
                                  const Eigen::MatrixXd& velocities,
                                  const Eigen::MatrixXd& accelerations) {
    return Goal(times,
                positions,
                velocities,
                accelerations,
                Eigen::MatrixXd(0, 0));
  }

  // Multiple waypoints + aux state, no time
  static Goal createFromWaypointsWithAux(const Eigen::MatrixXd& positions,
                                         const Eigen::MatrixXd& velocities,
                                         const Eigen::MatrixXd& accelerations,
                                         const Eigen::MatrixXd& aux) {
    return Goal(Eigen::VectorXd(0),
                positions,
                velocities,
                accelerations,
                aux);
  }

  // Multiple waypoints + aux state
  static Goal createFromWaypointsWithAux(const Eigen::VectorXd& times,
                                         const Eigen::MatrixXd& positions,
                                         const Eigen::MatrixXd& velocities,
                                         const Eigen::MatrixXd& accelerations,
                                         const Eigen::MatrixXd& aux) {
    return Goal(times,
                positions,
                velocities,
                accelerations,
                aux);
  }

  const Eigen::VectorXd& times() const { return times_; }
  const Eigen::MatrixXd& positions() const { return positions_; }
  const Eigen::MatrixXd& velocities() const { return velocities_; }
  const Eigen::MatrixXd& accelerations() const { return accelerations_; }
  const Eigen::MatrixXd& aux() const { return aux_; }

  // Build a trajectory that is continuous from the current arm state
  // If "start_vel" is null, assumed to be zero
  // If "start_acc" is null, assumed to be zero
  std::tuple<std::shared_ptr<hebi::trajectory::Trajectory>, Eigen::MatrixXd, Eigen::VectorXd> buildTrajectoryFrom(const Eigen::VectorXd& start_pos, const Eigen::VectorXd* start_vel, const Eigen::VectorXd* start_accel) const {

    // Note -- in future, we can call heuristics from API here
    auto getWaypointTimes = [](const Eigen::MatrixXd& positions, const Eigen::MatrixXd& /*velocities*/,
                                    const Eigen::MatrixXd& /*accelerations*/) {
      double rampTime = 1.2;

      size_t num_waypoints = positions.cols();

      Eigen::VectorXd times(num_waypoints);
      for (size_t i = 0; i < num_waypoints; ++i)
        times[i] = rampTime * (double)i;

      return times;
    };

    const auto num_joints = positions_.rows();
    auto num_waypoints = positions_.cols() + 1;
    if (start_pos.size() != num_joints) {
      throw std::invalid_argument("start positions size must match number of joints in Goal object");
    } else if (start_vel != nullptr && start_vel->size() != num_joints) {
      throw std::invalid_argument("start velocities size must match number of joints in Goal object");
    } else if (start_accel != nullptr && start_accel->size() != num_joints) {
      throw std::invalid_argument("start accelerations size must match number of joints in Goal object");
    }

    // Start with provided velocity and acceleration, or default to zero
    Eigen::VectorXd curr_vel = Eigen::VectorXd::Zero(num_joints);
    Eigen::VectorXd curr_accel = Eigen::VectorXd::Zero(num_joints);
    if (start_vel)
      curr_vel = *start_vel;
    if (start_accel)
      curr_accel = *start_accel;

    Eigen::MatrixXd positions(num_joints, num_waypoints);
    Eigen::MatrixXd velocities(num_joints, num_waypoints);
    Eigen::MatrixXd accelerations(num_joints, num_waypoints);

    // Initial state
    positions.col(0) = start_pos;
    velocities.col(0) = curr_vel;
    accelerations.col(0) = curr_accel;

    // Copy new waypoints
    positions.rightCols(num_waypoints - 1) = positions_;
    velocities.rightCols(num_waypoints - 1) = velocities_;
    accelerations.rightCols(num_waypoints - 1) = accelerations_;

    // Get waypoint times
    Eigen::VectorXd waypoint_times(num_waypoints);
    // If time vector is empty, automatically determine times
    if (times_.size() == 0) {
      waypoint_times = getWaypointTimes(positions, velocities, accelerations); // TODO: reference??!?
    } else {
      waypoint_times(0) = 0;
      waypoint_times.tail(num_waypoints - 1) = times_;
    }

    // Create new trajectory
    auto trajectory =
        hebi::trajectory::Trajectory::createUnconstrainedQp(waypoint_times, positions, &velocities, &accelerations);

    Eigen::MatrixXd aux(0,0);
    Eigen::VectorXd aux_times(0);

    // Update aux state:
    if (aux_.rows() > 0 && (aux_.cols() + 1) == num_waypoints) {
      aux.resize(aux_.rows(), aux_.cols() + 1);
      aux.col(0).setConstant(std::numeric_limits<double>::quiet_NaN());
      aux.rightCols(num_waypoints - 1) = aux_;
      aux_times = waypoint_times;
    }

    return {trajectory, aux, aux_times};
  }

private:

  Goal(const Eigen::VectorXd& times,
       const Eigen::MatrixXd& positions,
       const Eigen::MatrixXd& velocities,
       const Eigen::MatrixXd& accelerations,
       const Eigen::MatrixXd& aux)
    : times_(times),
      positions_(positions),
      velocities_(velocities),
      accelerations_(accelerations),
      aux_(aux) {
    if (positions_.rows() != velocities_.rows() || positions_.rows() != accelerations_.rows() ||
        positions_.cols() != velocities_.cols() || positions_.cols() != accelerations_.cols())
      throw std::invalid_argument("Goal must have consistent position/velocity/acceleration sizes");
    if (aux_.cols() != 0 && positions_.cols() != aux_.cols())
      throw std::invalid_argument("Goal must have empty aux or aux consistent with number of waypoints");
  }

  // Helper function to create unconstrained points along a motion, with nan at the right side.
  static Eigen::MatrixXd nanWithZeroRight(size_t num_joints, size_t num_waypoints) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    Eigen::MatrixXd matrix(num_joints, num_waypoints);
    matrix.setConstant(nan);
    matrix.rightCols<1>().setZero();
    return matrix;
  }

  static Eigen::VectorXd toVector(double scalar) {
    Eigen::VectorXd vector(1);
    vector[0] = scalar;
    return vector;
  }

  static Eigen::MatrixXd toMatrix(const Eigen::VectorXd& vector) {
    Eigen::MatrixXd matrix(vector.size(), 1);
    matrix.col(0) = vector;
    return matrix;
  }

  const Eigen::VectorXd times_{0};
  const Eigen::MatrixXd positions_{0, 0};
  const Eigen::MatrixXd velocities_{0, 0};
  const Eigen::MatrixXd accelerations_{0, 0};
  const Eigen::MatrixXd aux_{0, 0};
};

} // namespace arm
} // namespace hebi
