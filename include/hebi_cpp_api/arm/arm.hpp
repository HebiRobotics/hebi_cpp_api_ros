#pragma once

#include <set>

// HEBI C++ API components
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/robot_config.hpp"
#include "hebi_cpp_api/robot_model.hpp"
#include "hebi_cpp_api/trajectory.hpp"

// Arm API components
#include "end_effector.hpp"
#include "goal.hpp"
#include "kinematics_helper.hpp"

namespace hebi {
namespace experimental {
namespace arm {

// Forward declare for usage in plugin.
class Arm;

namespace plugin {

/// Abstract base class representing a plugin to be used for an Arm object.
/// Each implementation should also implement two static functions, and be included in the
/// ArmPluginMap list below:
///  - static std::unique_ptr<T> create(const PluginConfig&);
///  - static std::string pluginTypeName(); // For matching robot config file entries
///
/// Plugins allow extending an arm object with custom functionality not supported in the base class
///
/// The PluginConfig structure can have the following optional parameters used by this base
/// Plugin class, in addition to those defined for classes that inherit from Plugin.
/// optional:
///   - enabled: (bool) Represents the starting state of the plugin, either enabled (true)
///     or disabled.
///   - ramp_time: (float) A value in seconds that indicates how long it takes to transition
///     between enabled and disabled states.  Different plugins may scale their effects based on
///     this ramped enable/disable effect.  Must be positive.
class Plugin {
public:
  Plugin(const std::string& name) : name_(name) {}
  virtual ~Plugin() = default;

  // The name of this plugin
  std::string name() const { return name_; }

  // Determines if the plugin's effects are enabled.  Note that every
  // plugin is always invoked by Arm::update, and the plugin itself is
  // responsible for using this state (and the current ramped "enabled ratio")
  // to moderate its effect.
  bool enabled() const { return enabled_; }
  // Sets enabled state.  If "ramp time" > 0, the "enabled" state is immediately
  // set to the new value, but "enabled_ratio" will change gradually.
  void setEnabled(bool enabled) { enabled_ = enabled; }
  // Float value between 0 and 1 of "how enabled" we are as we are
  // ramping between disabled/enabled state.
  float enabledRatio() { return enabled_ratio_; }
  // Set how fast we ramp between enabled and disabled states; returns "false"
  // if value given is invalid.
  bool setRampTime(float ramp_time);
  // How fast we ramp between enabled and disabled states
  float rampTime() { return ramp_time_; }

  // Callback which updates state on the arm. Invoked by Arm::update.
  // Returns `true` on success and `false` otherwise.
  bool update(Arm&, double dt);

  // Can be overridden by plugin implementations if necessary; called by `Arm::send`
  virtual bool send() { return true; }

  // Override to update any state based on the associated arm.
  // Invoked when the instance is added to an arm via `Arm::addPlugin`.  Returns
  // "false" if there was an error when attempting to add.
  virtual bool onAssociated(const Arm&) { return true; }

protected:
  // When the object is created through the "create" factory method,
  // parameters should be applied in turn.  Each parameter should be
  // validated, and an error returned if they cannot be applied.
  //
  // Implementations should override the "applyParameterImpl" functions for parameters of types
  // they have. The main "applyParameters" function is called to iterate through the config
  // structure, and should be called by each implementing class' "create" method
  bool applyParameters(const PluginConfig& config, std::set<std::string> required_parameters);
  virtual bool applyParameterImpl(const std::string& name, bool value) { return false; }
  virtual bool applyParameterImpl(const std::string& name, const std::vector<bool>& value) { return false; }
  virtual bool applyParameterImpl(const std::string& name, float value) { return false; }
  virtual bool applyParameterImpl(const std::string& name, const std::vector<float>& value) { return false; }
  virtual bool applyParameterImpl(const std::string& name, const std::string& value) { return false; }
  virtual bool applyParameterImpl(const std::string& name, const std::vector<std::string>& value) { return false; }

  // Overridden by plugin implementations, and called by `Plugin::update`
  virtual bool updateImpl(Arm&, double dt) = 0;

private:
  virtual bool applyParameter(const std::string& name, bool value); // we implement for "enabled"
  virtual bool applyParameter(const std::string& name, const std::vector<bool>& value) {
    return applyParameterImpl(name, value);
  }
  virtual bool applyParameter(const std::string& name, float value); // we implement for "ramp_time"
  virtual bool applyParameter(const std::string& name, const std::vector<float>& value) {
    return applyParameterImpl(name, value);
  }
  virtual bool applyParameter(const std::string& name, const std::string& value) {
    return applyParameterImpl(name, value);
  }
  virtual bool applyParameter(const std::string& name, const std::vector<std::string>& value) {
    return applyParameterImpl(name, value);
  }

  // Name of the plugin
  const std::string name_{};
  // How long it takes to fully transition enabled state.
  float ramp_time_{};
  // Whether the plugin is enabled
  bool enabled_{true};
  // Current linear level between off (0) and on (1); updated during `update`
  float enabled_ratio_{1.f};
};

using Factory = std::function<std::unique_ptr<Plugin>(const PluginConfig&)>;

/// @brief Plugin for providing gravity compensating torques for the arm
///
/// The PluginConfig structure should have the following parameters, in addition
/// to the `enabled` and `ramp_time` parameters defined for the base Plugin class.
/// required:
///   (none)
/// optional:
///   imu_feedback_index: float (should be integer but float is basic yaml number; defaults to "0")
///   imu_frame_index: float (should be integer; which frame index this should be transformed by; defaults to "0")
///   imu_rotation_offset: vector<float> (row-major 3x3 rotation matrix to transform feedback by; defaults to "eye(3)”)
class GravityCompensationEffort : public Plugin {
public:
  static std::unique_ptr<GravityCompensationEffort> create(const PluginConfig&);
  static std::string pluginTypeName() { return "GravityCompensationEffort"; };
  bool onAssociated(const Arm& arm) override;

protected:
  // For "imu_feedback_index" and "imu_frame_index"
  bool applyParameterImpl(const std::string& name, float value) override;
  // For "imu_rotation_offset"
  bool applyParameterImpl(const std::string& name, const std::vector<float>& value) override;
  bool updateImpl(Arm& arm, double dt) override;

private:
  GravityCompensationEffort(const std::string& name) : Plugin(name) {}
  // Cached helper var
  Eigen::VectorXd grav_efforts_;
  // Parameters
  size_t imu_feedback_index_{};
  size_t imu_frame_index_{};
  Eigen::Matrix3d imu_local_transform_{Eigen::Matrix3d::Identity()};
};

class ImpedanceController : public Plugin {
public:
  static std::unique_ptr<ImpedanceController> create(const PluginConfig&);
  static std::string pluginTypeName() { return "ImpedanceController"; };
  bool onAssociated(const Arm& arm) override;

protected:
  // For "gains_in_end_effector_frame"
  bool applyParameterImpl(const std::string& name, bool value) override;
  // For "kp", "kd", "ki", and "i_clamp"
  bool applyParameterImpl(const std::string& name, const std::vector<float>& value) override;
  bool updateImpl(Arm& arm, double dt) override;

private:
  ImpedanceController(const std::string& name) : Plugin(name) {}

  // Cached helper vars

  // Current integral error term
  Eigen::VectorXd i_error_{Eigen::VectorXd::Zero(6)};

  // Parameters

  // Translations and Rotations can be specified in the
  // base frame or in the end effector frame.
  bool gains_in_end_effector_frame_{};
  // Impendance Control Gains
  // NOTE: The gains correspond to:
  // [ trans_x trans_y trans_z rot_x rot_y rot_z ]
  Eigen::VectorXd kp_{Eigen::VectorXd::Zero(6)}; // (N/m) or (Nm/rad)
  Eigen::VectorXd kd_{Eigen::VectorXd::Zero(6)}; // (N/(m/sec)) or (Nm/(rad/sec))
  Eigen::VectorXd ki_{Eigen::VectorXd::Zero(6)}; // (N/(m*sec)) or (Nm/(rad*sec))
  Eigen::VectorXd i_clamp_;
};

class EffortOffset : public Plugin {
public:
  static std::unique_ptr<EffortOffset> create(const PluginConfig&);
  static std::string pluginTypeName() { return "EffortOffset"; };
  bool onAssociated(const Arm& arm) override;

protected:
  // For "offset"
  bool applyParameterImpl(const std::string& name, const std::vector<float>& value) override;
  bool updateImpl(Arm& arm, double dt) override;

private:
  EffortOffset(const std::string& name) : Plugin(name) {}
  // Parameters
  Eigen::VectorXd effort_offsets_{};
};

} // namespace plugin

static std::map<std::string, plugin::Factory> ArmPluginMap = {
    {plugin::GravityCompensationEffort::pluginTypeName(), plugin::GravityCompensationEffort::create},
    {plugin::ImpedanceController::pluginTypeName(), plugin::ImpedanceController::create},
    {plugin::EffortOffset::pluginTypeName(), plugin::EffortOffset::create}
};

// A high-level abstraction of a robot arm, coordinating kinematics, control,
// and basic motion planning.
//
// Typical usage is as follows; the robot.cfg file includes information such as
// module family and names, HRDFs, gains, etc.
//
// std::vector<std::string> errors;
// auto cfg = RobotConfig::loadConfig("robot.cfg", errors);
// if (!cfg)
//   return; // see contents of "errors"
// auto arm = Arm::create(*cfg);
// if (!arm)
//   return; // are modules on network?
//
// arm->loadGains(cfg->getGains("default"));
//
// while(true) {
//   arm->update();
//   arm->send();
//   if (some_condition)
//     arm->setGoal(target_goal);
// }
//
// (Note -- in an actual application, you would want to verify the return
// values of many of the functions above to ensure proper operation!)
class Arm {
public:
  //////////////////////////////////////////////////////////////////////////////
  // Setup functions
  //////////////////////////////////////////////////////////////////////////////

  // Parameters for creating an arm
  struct Params {
    // The family and names passed to the "lookup" function to find modules
    // Both are required.
    std::vector<std::string> families_;
    std::vector<std::string> names_;
    // How long a command takes effect for on the robot before expiring.
    int command_lifetime_ = 100;
    // Loop rate, in Hz.  This is how fast the arm update loop will nominally
    // run.
    double control_frequency_ = 200.f;

    // The robot description.  Either supply the hrdf_file _or_ the robot_model.
    std::string hrdf_file_;
    std::shared_ptr<robot_model::RobotModel> robot_model_;

    // Optionally, supply an end effector to be controlled by the "aux" state of
    // provided goals.
    std::shared_ptr<EndEffectorBase> end_effector_;

    // A function pointer that returns a double representing the current time in
    // seconds. (Can be overloaded to use, e.g., simulator time)
    //
    // The default value uses the steady clock wall time.
    std::function<double()> get_current_time_s_ = []() {
      using clock = std::chrono::steady_clock;
      static const clock::time_point start_time = clock::now();
      return (std::chrono::duration<double>(clock::now() - start_time)).count();
    };
  };

  // Creates an "Arm" object; uses the RobotConfig file for information about the robot.
  static std::unique_ptr<Arm> create(const RobotConfig& config);

  // Creates an "Arm" object, and puts it into a "weightless" no-goal control
  // mode.
  static std::unique_ptr<Arm> create(const Params& params);

  // Adds the plugin to the arm object, taking ownership of the plugin.
  bool addPlugin(std::unique_ptr<plugin::Plugin> plugin);

  // Returns a weak pointer to the first plugin found of the given type, or nullopt
  // if nothing is found.
  template<class T>
  std::weak_ptr<plugin::Plugin> getPluginByType() {
    for (auto& p : plugins_) {
      if (dynamic_cast<T*>(p.get()) != nullptr)
        return p;
    }
    return {};
  }

  std::weak_ptr<plugin::Plugin> getPluginByName(const std::string& name);

  // Loads gains from the given .xml file, and sends them to the module. Returns
  // false if the gains file could not be found, if these is a mismatch in
  // number of modules, or the modules do not acknowledge receipt of the gains.
  bool loadGains(const std::string& gains_file);

  //////////////////////////////////////////////////////////////////////////////
  // Accessors
  //////////////////////////////////////////////////////////////////////////////

  // Returns the number of modules / DoF in the arm
  size_t size() const { return group_->size(); }

  // Returns the internal group. Not necessary for most use cases.
  const Group& group() const { return *group_; }

  // Returns the internal robot model. Not necessary for most use cases.
  const robot_model::RobotModel& robotModel() const { return *robot_model_; }
  // Returns the (non-const) internal robot model. Not necessary for most use cases.
  // Use with care, as modifying the properties of the underlying robot model while
  // using the arm may result in undefined behavior.
  robot_model::RobotModel& robotModel() { return *robot_model_; }

  // Returns the currently active internal trajectory. Not necessary for most
  // use cases.
  // Returns 'nullptr' if there is no active trajectory.
  const trajectory::Trajectory* trajectory() const { return trajectory_.get(); }

  // Returns the end effector object, if given. Not necessary for most use
  // cases.
  // Returns 'nullptr' if there is no end effector.
  const EndEffectorBase* endEffector() const { return end_effector_.get(); }

  // Returns the command last computed by update, or an empty command object
  // if "update" has never successfully run. The returned command can be
  // modified as desired before it is sent to the robot with the send function.
  GroupCommand& pendingCommand() { return command_; }
  const GroupCommand& pendingCommand() const { return command_; }

  // Returns the last feedback obtained by update, or an empty feedback object
  // if "update" has never successfully run.
  const GroupFeedback& lastFeedback() const { return feedback_; }

  //////////////////////////////////////////////////////////////////////////////
  // Main loop functions
  //
  // Typical usage:
  //
  // while(true) {
  //   arm->update();
  //   arm->send();
  // }
  //////////////////////////////////////////////////////////////////////////////

  // Updates feedback and generates the basic command for this timestep.
  // To retrieve the feedback, call `getLastFeedback()` after this call.
  // You can modify the command object after calling this.
  //
  // Returns 'false' on a connection problem; true on success.
  bool update();

  // Sends the command last computed by "update" to the robot arm.  Any user
  // modifications to the command are included.
  bool send();

  //////////////////////////////////////////////////////////////////////////////
  // Goals
  //
  // A goal is a desired (joint angle) position that the arm should reach, and
  // optionally information about the time it should reach that goal at and the
  // path (position, velocity, and acceleration waypoints) it should take to
  // get there.
  //
  // The default behavior when a goal is set is for the arm to plan and begin
  // executing a smooth motion from its current state to this goal, with an
  // internal heuristic that defines the time at which it will reach the goal.
  // This immediately overrides any previous goal that was set.
  //
  // If there is no "active" goal the arm is set into a mode where it is
  // actively controlled to be approximately weightless, and can be moved around
  // by hand easily.  This is the default state when the arm is created.
  //
  // After reaching the goal, the arm continues to be commanded with the final
  // joint state of the set goal, and is _not_ implicitly returned to a
  // "weightless" mode.
  //
  // A goal may also define "aux" states to be sent to an end effector
  // associated with the arm.  In this case, the end effector states are
  // treated as "step functions", immediately being commanded at the timestamp
  // of the waypoint they are associated with.  An empty "aux" goal or "NaN"
  // defines a "no transition" at the given waypoint.
  //////////////////////////////////////////////////////////////////////////////

  // Set the current goal waypoint(s), immediately replanning to these
  // location(s) and optionally end effector states.
  // Goal is a commanded position / velocity.
  void setGoal(const Goal& goal);

  // Set the state of aux, if added (e.g., end effector).  Overrides any
  // future aux waypoints.
  template<typename T>
  void setAuxState(const T& aux_state) {
    auto aux_size = aux_state.size();
    if (aux_state.size() > 0) {
      aux_times_.resize(1);
      aux_times_[0] = get_current_time_s_();
      aux_.resize(aux_size, 1);
      // Replaces 'aux_.leftCols(1) = aux_state;'
      for (size_t i = 0; i < aux_size; i++) {
        aux_(i, 0) = aux_state[i];
      }
    } else {
      // Reset aux states
      aux_.resize(0, 0);
      aux_times_.resize(0);
    }
  }

  // Returns the progress (from 0 to 1) of the current goal, per the last
  // update call.
  //
  // If we have reached the goal, progress is "1".  If there is no active goal,
  // or we have just begun, progress is "0".
  double goalProgress() const;

  // Have we reached the goal?  If there is no goal, returns 'false'
  bool atGoal() const { return goalProgress() >= 1.0; }

  // Cancels any active goal, returning to a "weightless" state which does not
  // actively command position or velocity.
  void cancelGoal();

  //////////////////////////////////////////////////////////////////////////////
  // Helper functions for forward and inverse kinematics.
  //////////////////////////////////////////////////////////////////////////////

  // Use the following joint limits when computing IK
  // Affects future calls to solveIK**.
  void setJointLimits(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions) {
    kinematics_helper_.setJointLimits(*robot_model_, min_positions, max_positions);
  }

  // Do not use joint limits when computing IK
  // Affects future calls to solveIK**.
  void clearJointLimits(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions) {
    kinematics_helper_.clearJointLimits();
  }

  // Get the end effector (x,y,z) location
  Eigen::Vector3d FK(const Eigen::VectorXd& positions) const {
    return kinematics_helper_.FK3Dof(*robot_model_, positions);
  }

  // Get the end effector (x,y,z) location and direction, represented by a unit-length vector
  void FK(const Eigen::VectorXd& positions, Eigen::Vector3d& xyz_out, Eigen::Vector3d& tip_axis) const {
    kinematics_helper_.FK5Dof(*robot_model_, positions, xyz_out, tip_axis);
  }

  // Get the end effector (x,y,z) location and orientation (represented by a rotation matrix)
  void FK(const Eigen::VectorXd& positions, Eigen::Vector3d& xyz_out, Eigen::Matrix3d& orientation) const {
    kinematics_helper_.FK6Dof(*robot_model_, positions, xyz_out, orientation);
  }

  // Return the joint angles to move to a given xyz location
  Eigen::VectorXd solveIK(const Eigen::VectorXd& initial_positions, const Eigen::Vector3d& target_xyz) const {
    return kinematics_helper_.solveIK3Dof(*robot_model_, initial_positions, target_xyz);
  }

  // Return the joint angles to move to a given xyz location while
  // pointing a certain direction
  Eigen::VectorXd solveIK(const Eigen::VectorXd& initial_positions, const Eigen::Vector3d& target_xyz,
                          const Eigen::Vector3d& end_tip) const {
    return kinematics_helper_.solveIK5Dof(*robot_model_, initial_positions, target_xyz, end_tip);
  }

  // Return the joint angles to move to a given xyz location while
  // pointing a certain direction
  Eigen::VectorXd solveIK(const Eigen::VectorXd& initial_positions, const Eigen::Vector3d& target_xyz,
                          const Eigen::Matrix3d& orientation) const {
    return kinematics_helper_.solveIK6Dof(*robot_model_, initial_positions, target_xyz, orientation);
  }

private:
  // Private arm constructor
  Arm(std::function<double()> get_current_time_s, std::shared_ptr<Group> group,
      std::shared_ptr<robot_model::RobotModel> robot_model, std::shared_ptr<EndEffectorBase> end_effector = nullptr)
    : get_current_time_s_(get_current_time_s),
      last_time_(get_current_time_s()),
      group_(std::move(group)),
      robot_model_(std::move(robot_model)),
      end_effector_(std::move(end_effector)),
      pos_(Eigen::VectorXd::Zero(group_->size())),
      vel_(Eigen::VectorXd::Zero(group_->size())),
      accel_(Eigen::VectorXd::Zero(group_->size())),
      feedback_(group_->size()),
      command_(group_->size()) {}

  // Returns the aux state at this point in the trajectory
  Eigen::VectorXd getAux(double t) const;

  std::function<double()> get_current_time_s_;
  double last_time_;
  std::shared_ptr<Group> group_;
  std::shared_ptr<robot_model::RobotModel> robot_model_;
  std::shared_ptr<EndEffectorBase> end_effector_;

  // The joint angle trajectory for reaching the current goal.
  std::shared_ptr<trajectory::Trajectory> trajectory_;
  double trajectory_start_time_{std::numeric_limits<double>::quiet_NaN()};
  // These are just temporary variables to cache output from
  // Trajectory::getState.
  Eigen::VectorXd pos_;
  Eigen::VectorXd vel_;
  Eigen::VectorXd accel_;

  // Along with a trajectory, aux states may be set.  These are the last aux
  // state for each timestep in the trajectory:
  Eigen::VectorXd aux_times_;
  Eigen::MatrixXd aux_;

  // Robot model helpers for FK + IK
  internal::KinematicsHelper kinematics_helper_;

  hebi::GroupFeedback feedback_;
  hebi::GroupCommand command_;

  // Current arm plugins
  std::vector<std::shared_ptr<plugin::Plugin>> plugins_;
};

} // namespace arm
} // namespace experimental
} // namespace hebi
