#include "hebi_cpp_api/arm/arm.hpp"

#include <Eigen/Geometry>

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/util/grav_comp.hpp"

namespace hebi {
namespace experimental {
namespace arm {

namespace plugin {

bool Plugin::setRampTime(float ramp_time) {
  if (ramp_time < 0)
    return false;
  ramp_time_ = ramp_time;
  return true;
}

bool Plugin::applyParameters(const PluginConfig& config, std::set<std::string> required_parameters) {
  for (const auto& p : config.bools_) {
    if (!applyParameter(p.first, p.second))
      return false;
    if (required_parameters.count(p.first))
      required_parameters.erase(p.first);
  }
  for (const auto& p : config.bool_lists_) {
    if (!applyParameter(p.first, p.second))
      return false;
    if (required_parameters.count(p.first))
      required_parameters.erase(p.first);
  }
  for (const auto& p : config.floats_) {
    if (!applyParameter(p.first, p.second))
      return false;
    if (required_parameters.count(p.first))
      required_parameters.erase(p.first);
  }
  for (const auto& p : config.float_lists_) {
    if (!applyParameter(p.first, p.second))
      return false;
    if (required_parameters.count(p.first))
      required_parameters.erase(p.first);
  }
  for (const auto& p : config.strings_) {
    if (!applyParameter(p.first, p.second))
      return false;
    if (required_parameters.count(p.first))
      required_parameters.erase(p.first);
  }
  for (const auto& p : config.string_lists_) {
    if (!applyParameter(p.first, p.second))
      return false;
    if (required_parameters.count(p.first))
      required_parameters.erase(p.first);
  }
  if (!required_parameters.empty()) {
    std::cout << "Required plugin parameter not present!\n";
    return false;
  }
  return true;
}

bool Plugin::applyParameter(const std::string& name, bool value) {
  // we implement for "enabled"
  if (name == "enabled") {
    enabled_ = value;
    enabled_ratio_ = value ? 1.f : 0.f;
    return true;
  }
  return applyParameterImpl(name, value);
}

bool Plugin::applyParameter(const std::string& name, float value) {
  // we implement for "ramp_time"
  if (name == "ramp_time") {
    if (value < 0) // Cannot be negative!
      return false;
    ramp_time_ = value;
    return true;
  }
  return applyParameterImpl(name, value);
}

bool Plugin::update(Arm& arm, double dt) {
  if (enabled_ && enabled_ratio_ < 1.f) {
    if (ramp_time_ == 0.f)
      enabled_ratio_ = 1.f;
    else
      enabled_ratio_ = std::min(1.f, enabled_ratio_ + static_cast<float>(dt) / ramp_time_);
  } else if (!enabled_ && enabled_ratio_ > 0.f) {
    if (ramp_time_ == 0.f)
      enabled_ratio_ = 0.f;
    else
      enabled_ratio_ = std::max(0.f, enabled_ratio_ - static_cast<float>(dt) / ramp_time_);
  }
  return updateImpl(arm, dt);
}

std::unique_ptr<GravityCompensationEffort> GravityCompensationEffort::create(const PluginConfig& config) {
  auto plugin = std::unique_ptr<GravityCompensationEffort>(new GravityCompensationEffort(config.name_));
  if (!plugin->applyParameters(config, {}))
    return nullptr;
  return plugin;
}

bool GravityCompensationEffort::onAssociated(const Arm& arm) {
  // Initialize helper/cache variable
  grav_efforts_ = Eigen::VectorXd(arm.size());
  if (imu_feedback_index_ >= arm.size())
    return false;
  if (imu_frame_index_ >= arm.robotModel().getFrameCount(hebi::robot_model::FrameType::Input))
    return false;
  return true;
}

bool GravityCompensationEffort::applyParameterImpl(const std::string& name, float value) {
  // Note -- ideally, we would check these are both valid indices, but we don't have info
  // on the HRDF or group yet...
  if (name == "imu_feedback_index") {
    if (std::round(value) != value || value < 0)
      return false;
    imu_feedback_index_ = value;
    return true;
  }
  if (name == "imu_frame_index") {
    if (std::round(value) != value || value < 0)
      return false;
    imu_frame_index_ = value;
    return true;
  }
  return false;
}

bool GravityCompensationEffort::applyParameterImpl(const std::string& name, const std::vector<float>& value) {
  if (name == "imu_rotation_offset") {
    if (value.size() != 9)
      return false;
    imu_local_transform_(0, 0) = value.at(0);
    imu_local_transform_(0, 1) = value.at(1);
    imu_local_transform_(0, 2) = value.at(2);
    imu_local_transform_(1, 0) = value.at(3);
    imu_local_transform_(1, 1) = value.at(4);
    imu_local_transform_(1, 2) = value.at(5);
    imu_local_transform_(2, 0) = value.at(6);
    imu_local_transform_(2, 1) = value.at(7);
    imu_local_transform_(2, 2) = value.at(8);
    return true;
  }
  // Note -- could check for valid transform matrix here, too...
  return false;
}

bool GravityCompensationEffort::updateImpl(Arm& arm, double dt) {
  // Get orientation of specified module in the arm:
  auto gravity = util::gravityFromQuaternion(arm.lastFeedback()[imu_feedback_index_].imu().orientation().get());
  auto g_norm = gravity.norm();
  if (g_norm <= 0.0)
    return false;

  Eigen::Vector3d normed_gravity = gravity / g_norm * 9.81;

  // transform this gravity vector by local transform if necessary
  normed_gravity = imu_local_transform_ * normed_gravity;

  hebi::robot_model::Matrix4dVector frames;
  arm.robotModel().getFK(hebi::robot_model::FrameType::Input, arm.lastFeedback().getPosition(), frames);

  // transform by output frame of module where IMU is:
  Eigen::Matrix3d my_frame = frames[imu_frame_index_].topLeftCorner<3, 3>();
  normed_gravity = my_frame * normed_gravity;

  // Compute and set grav comp efforts
  arm.robotModel().getGravCompEfforts(arm.lastFeedback().getPosition(), normed_gravity, grav_efforts_);
  arm.pendingCommand().setEffort(arm.pendingCommand().getEffort() + grav_efforts_ * enabledRatio());

  return true;
}

std::unique_ptr<ImpedanceController> ImpedanceController::create(const PluginConfig& config) {
  auto plugin = std::unique_ptr<ImpedanceController>(new ImpedanceController(config.name_));
  if (!plugin->applyParameters(config, {"gains_in_end_effector_frame", "kp", "kd"}))
    return nullptr;
  return plugin;
}

bool ImpedanceController::onAssociated(const Arm& arm) {
  return true;
}

bool ImpedanceController::applyParameterImpl(const std::string& name, bool value) {
  if (name == "gains_in_end_effector_frame") {
    gains_in_end_effector_frame_ = value;
    return true;
  }
  return false;
}

bool ImpedanceController::applyParameterImpl(const std::string& name, const std::vector<float>& value) {
  // For "kp", "kd", "ki", and "i_clamp"
  if (name == "kp" || name == "kd" || name == "ki" || name == "i_clamp") {
    Eigen:VectorXd* dest = &kp_;
    if (name == "kd") {
      dest = &kd_;
    } else if (name == "ki") {
      dest = &ki_;
    } else if (name == "i_clamp") {
      i_clamp_ = Eigen::VectorXd::Zero(6);
      dest = &i_clamp_;
    }
    if (value.size() != 6)
      return false;
    for (int i = 0; i < 6; ++i)
      (*dest)[i] = value[i];
    return true;
  }
  return false;
}

bool ImpedanceController::updateImpl(Arm& arm, double dt) {
  auto& cmd = arm.pendingCommand();
  auto cmd_pos = cmd.getPosition();
  auto cmd_vel = cmd.getVelocity();

  // If either pos or vel is unset:
  if (cmd_pos.end() != std::find_if(cmd_pos.begin(), cmd_pos.end(), [](double v){ return std::isnan(v); }) ||
      cmd_vel.end() != std::find_if(cmd_vel.begin(), cmd_vel.end(), [](double v){ return std::isnan(v); })) {
    i_error_ = Eigen::VectorXd::Zero(6);
    return true;
  }

  auto& fbk = arm.lastFeedback();
  auto fbk_pos = fbk.getPosition();
  auto fbk_vel = fbk.getVelocity();

  // Get Updated Forward Kinematics and Jacobians
  Eigen::Vector3d desired_tip_xyz_fk;
  Eigen::Matrix3d desired_tip_rot_fk;
  arm.FK(cmd_pos, desired_tip_xyz_fk, desired_tip_rot_fk);
  Eigen::Vector3d actual_tip_xyz_fk;
  Eigen::Matrix3d actual_tip_rot_fk;
  arm.FK(fbk_pos, actual_tip_xyz_fk, actual_tip_rot_fk);
  Eigen::MatrixXd j_arm_tip;
  arm.robotModel().getJacobianEndEffector(fbk_pos, j_arm_tip);
  // end effector or world frame?
  Eigen::Matrix3d frame_rot =
    gains_in_end_effector_frame_ ? actual_tip_rot_fk : Eigen::Matrix3d::Identity();

  // Linear error is easy
  auto xyz_error = desired_tip_xyz_fk - actual_tip_xyz_fk;

  // Rotational error involves calculating axis-angle from the
  // resulting error in S03 and providing a torque around that axis.
  auto error_rot_mat = desired_tip_rot_fk * actual_tip_rot_fk.transpose();
  Eigen::AngleAxisd axis_angle(error_rot_mat);
  auto rot_error_vec = axis_angle.angle() * axis_angle.axis();

  Eigen::VectorXd pos_error(6);
  pos_error.head<3>() = xyz_error;
  pos_error.tail<3>() = rot_error_vec;
  Eigen::VectorXd vel_error = j_arm_tip * (cmd_vel - fbk_vel);

  // Calculate Impedance Control Wrenches and Appropriate Joint
  // Torques
  pos_error.head<3>() = frame_rot.transpose() * pos_error.head<3>(); // linear component
  pos_error.tail<3>() = frame_rot.transpose() * pos_error.tail<3>(); // rotational component
  vel_error.head<3>() = frame_rot.transpose() * vel_error.head<3>(); // linear component
  vel_error.tail<3>() = frame_rot.transpose() * vel_error.tail<3>(); // rotational component
  i_error_ += pos_error * dt;

  // Combine everything
  Eigen::VectorXd wrench = kp_.cwiseProduct(pos_error);

  Eigen::VectorXd i_wrench = ki_.cwiseProduct(i_error_);
  if (i_clamp_.size() > 0) {
    i_wrench = i_wrench.cwiseMax(-i_clamp_);
    i_wrench = i_wrench.cwiseMin(i_clamp_);
  }
  wrench += i_wrench;

  wrench += kd_.cwiseProduct(vel_error);

  // Rotate to appropriate frame
  wrench.head<3>() = frame_rot * wrench.head<3>(); // linear component
  wrench.tail<3>() = frame_rot * wrench.tail<3>(); // rotational component

  // Add impedance efforts to effort output
  auto impedance_efforts = j_arm_tip.transpose() * wrench;
  arm.pendingCommand().setEffort(arm.pendingCommand().getEffort() + impedance_efforts * enabledRatio());
  return true;
}

std::unique_ptr<EffortOffset> EffortOffset::create(const PluginConfig& config) {
  auto plugin = std::unique_ptr<EffortOffset>(new EffortOffset(config.name_));
  if (!plugin->applyParameters(config, {"offset"}))
    return nullptr;
  return plugin;
}

bool EffortOffset::onAssociated(const Arm& arm) {
  // Initialize helper/cache variable
  if (effort_offsets_.size() != arm.size())
    return false;
  return true;
}

bool EffortOffset::applyParameterImpl(const std::string& name, const std::vector<float>& value) {
  if (name == "offset") {
    effort_offsets_.resize(value.size());
    for (size_t i = 0; i < value.size(); ++i)
      effort_offsets_(i) = value[i];
    return true;
  }
  // Note -- could check for valid transform matrix here, too...
  return false;
}

bool EffortOffset::updateImpl(Arm& arm, double dt) {
  arm.pendingCommand().setEffort(arm.pendingCommand().getEffort() + effort_offsets_ * enabledRatio());
  return true;
}

} // namespace plugin

std::unique_ptr<Arm> Arm::create(const RobotConfig& config) {
  // Load the HRDF:
  std::shared_ptr<robot_model::RobotModel> robot_model;
  if (!config.getHrdf().empty())
    robot_model = robot_model::RobotModel::loadHRDF(config.getHrdf());

  if (!robot_model)
    return nullptr;

  // Get the group (scope the lookup object so it is destroyed
  // immediately after the lookup operation)
  std::shared_ptr<Group> group;
  {
    Lookup lookup;
    group = lookup.getGroupFromNames(config.getFamilies(), config.getNames());
  }
  if (!group) {
    std::cout << "Could not create arm! Check that family and names match actuators on the network.\n";
    return nullptr;
  }

  // Check sizes
  if (group->size() != robot_model->getDoFCount()) {
    std::cout << "HRDF does not have the same number of actuators as group!\n";
    return nullptr;
  }

  // Try to get feedback -- if we don't get a packet in the first N times,
  // something is wrong
  int num_attempts = 0;

  // We need feedback, so we can plan trajectories if that gets called before the first "update"
  GroupFeedback feedback(group->size());
  while (!group->getNextFeedback(feedback)) {
    if (num_attempts++ > 10) {
      std::cout << "Could not communicate with robot; check network connection.\n";
      return nullptr;
    }
  }

  std::function<double()> get_current_time_s = []() {
    using clock = std::chrono::steady_clock;
    static const clock::time_point start_time = clock::now();
    return (std::chrono::duration<double>(clock::now() - start_time)).count();
  };

  // Note: once ROS moves up to C++14, we can change this to "make_unique".
  auto arm = std::unique_ptr<Arm>(new Arm(get_current_time_s, std::move(group), std::move(robot_model), {}));

  for (const auto& plugin_config : config.getPluginConfigs()) {
    try {
      auto creator = ArmPluginMap.at(plugin_config.type_);
      auto plugin = creator(plugin_config);
      if (!plugin) {
        std::cout << "Could not create plugin.\n";
      } else if (!arm->addPlugin(std::move(plugin))) {
        std::cout << "Could not add plugin.\n";
      }
    } catch (const std::out_of_range& ex) {
      std::cout << "Could not create and add " << plugin_config.type_ << "; unknown type.\n";
    }
  }

  return std::move(arm);
}

std::unique_ptr<Arm> Arm::create(const Arm::Params& params) {
  // Load the HRDF:
  std::shared_ptr<robot_model::RobotModel> robot_model;
  if (params.hrdf_file_.empty())
    robot_model = params.robot_model_;
  else
    robot_model = robot_model::RobotModel::loadHRDF(params.hrdf_file_);

  if (!robot_model)
    return nullptr;

  // Get the group (scope the lookup object so it is destroyed
  // immediately after the lookup operation)
  std::shared_ptr<Group> group;
  {
    Lookup lookup;
    group = lookup.getGroupFromNames(params.families_, params.names_);
  }
  if (!group) {
    std::cout << "Could not create arm! Check that family and names match actuators on the network.\n";
    return nullptr;
  }

  // Check sizes
  if (group->size() != robot_model->getDoFCount()) {
    std::cout << "HRDF does not have the same number of actuators as group!\n";
    return nullptr;
  }

  // Set parameters
  if (!group->setCommandLifetimeMs(params.command_lifetime_)) {
    std::cout << "Could not set command lifetime on group; check that it is valid.\n";
    return nullptr;
  }
  if (!group->setFeedbackFrequencyHz(params.control_frequency_)) {
    std::cout << "Could not set feedback frequency on group; check that it is valid.\n";
    return nullptr;
  }

  // Try to get feedback -- if we don't get a packet in the first N times,
  // something is wrong
  int num_attempts = 0;

  // We need feedback, so we can plan trajectories if that gets called before the first "update"
  GroupFeedback feedback(group->size());
  while (!group->getNextFeedback(feedback)) {
    if (num_attempts++ > 10) {
      std::cout << "Could not communicate with robot; check network connection.\n";
      return nullptr;
    }
  }

  // Note: once ROS moves up to C++14, we can change this to "make_unique".
  auto arm =
      std::unique_ptr<Arm>(new Arm(params.get_current_time_s_, std::move(group), std::move(robot_model), params.end_effector_));

  // Implicitly defined to keep previous behavior
  PluginConfig config("GravityCompensationEffort", "GravityCompensationEffort");
  auto gcp = arm::plugin::GravityCompensationEffort::create(config);
  arm->addPlugin(std::move(gcp));

  return arm;
}

bool Arm::addPlugin(std::unique_ptr<plugin::Plugin> plugin) {
  if (!plugin) {
    std::cout << "Null plugin passed to addPlugin!\n";
    return false;
  }
  if (!plugin->onAssociated(*this)) {
    std::cout << "Could not associate plugin with arm: " << plugin->name() << "\n";
    return false;
  }
  plugins_.push_back(std::move(plugin));
  return true;
}

std::weak_ptr<plugin::Plugin> Arm::getPluginByName(const std::string& name) {
  for (auto& p : plugins_) {
    if (p->name() == name)
      return p;
  }
  return {};
}

bool Arm::loadGains(const std::string& gains_file) {
  hebi::GroupCommand gains_cmd(group_->size());
  if (!gains_cmd.readGains(gains_file))
    return false;

  return group_->sendCommandWithAcknowledgement(gains_cmd);
}

bool Arm::update() {
  double t = get_current_time_s_();

  // Time must be monotonically increasing!
  if (t < last_time_)
    return false;

  double dt = t - last_time_;
  last_time_ = t;

  if (!group_->getNextFeedback(feedback_))
    return false;

  // Define aux state so end effector can be updated
  Eigen::VectorXd aux(0);

  // Update command from trajectory
  if (trajectory_) {
    // If we have an active trajectory to our goal, use this.
    // Note -- this applies even if we are past the end of it;
    // we just stay with last state.
    // (trajectory_start_time_ should not be nan here!)
    double t_traj = t - trajectory_start_time_;
    t_traj = std::min(t_traj, trajectory_->getDuration());
    trajectory_->getState(t_traj, &pos_, &vel_, &accel_);

    aux = getAux(t_traj);
  } else {
    pos_.setConstant(std::numeric_limits<double>::quiet_NaN());
    vel_.setConstant(std::numeric_limits<double>::quiet_NaN());
    accel_.setConstant(0.0);
  }
  command_.setPosition(pos_);
  command_.setVelocity(vel_);
  // Set to zero by default
  command_.setEffort(Eigen::VectorXd::Constant(pos_.size(), 0.0));

  bool res = true;

  // Update end effector if one exists
  if (end_effector_)
    res = end_effector_->update(aux) && res;

  // Update all plugins
  for (const auto& plugin : plugins_)
    res = plugin->update(*this, dt) && res;

  return res;
}

bool Arm::send() { return group_->sendCommand(command_) && (end_effector_ ? end_effector_->send() : true); }

// TODO: think about adding customizability, or at least more intelligence for
// the default heuristic.
Eigen::VectorXd getWaypointTimes(const Eigen::MatrixXd& positions, const Eigen::MatrixXd& velocities,
                                 const Eigen::MatrixXd& accelerations) {
  double rampTime = 1.2;

  size_t num_waypoints = positions.cols();

  Eigen::VectorXd times(num_waypoints);
  for (size_t i = 0; i < num_waypoints; ++i)
    times[i] = rampTime * (double)i;

  return times;
}

void Arm::setGoal(const Goal& goal) {
  int num_joints = goal.positions().rows();

  // If there is a current trajectory, use the commands as a starting point;
  // if not, replan from current feedback.
  Eigen::VectorXd curr_pos = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_vel = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_accel = Eigen::VectorXd::Zero(num_joints);

  // Replan if these is a current trajectory:
  if (trajectory_) {
    double t_traj = last_time_ - trajectory_start_time_;
    t_traj = std::min(t_traj, trajectory_->getDuration());
    trajectory_->getState(t_traj, &curr_pos, &curr_vel, &curr_accel);
  } else {
    curr_pos = feedback_.getPosition();
    curr_vel = feedback_.getVelocity();
    // (accelerations remain zero)
  }

  int num_waypoints = goal.positions().cols() + 1;

  Eigen::MatrixXd positions(num_joints, num_waypoints);
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);

  // Initial state
  positions.col(0) = curr_pos;
  velocities.col(0) = curr_vel;
  accelerations.col(0) = curr_accel;

  // Copy new waypoints
  positions.rightCols(num_waypoints - 1) = goal.positions();
  velocities.rightCols(num_waypoints - 1) = goal.velocities();
  accelerations.rightCols(num_waypoints - 1) = goal.accelerations();

  // Get waypoint times
  Eigen::VectorXd waypoint_times(num_waypoints);
  // If time vector is empty, automatically determine times
  if (goal.times().size() == 0) {
    waypoint_times = getWaypointTimes(positions, velocities, accelerations);
  } else {
    waypoint_times(0) = 0;
    waypoint_times.tail(num_waypoints - 1) = goal.times();
  }

  // Create new trajectory
  trajectory_ =
      hebi::trajectory::Trajectory::createUnconstrainedQp(waypoint_times, positions, &velocities, &accelerations);
  trajectory_start_time_ = last_time_;

  // Update aux state:
  if (goal.aux().rows() > 0 && (goal.aux().cols() + 1) == num_waypoints) {
    aux_.resize(goal.aux().rows(), goal.aux().cols() + 1);
    aux_.col(0).setConstant(std::numeric_limits<double>::quiet_NaN());
    aux_.rightCols(num_waypoints - 1) = goal.aux();
    aux_times_ = waypoint_times;
  } else {
    // Reset aux states!
    aux_.resize(0, 0);
    aux_times_.resize(0);
  }
}

double Arm::goalProgress() const {
  if (trajectory_) {
    double t_traj = last_time_ - trajectory_start_time_;
    t_traj = std::min(t_traj, trajectory_->getDuration());
    return t_traj / trajectory_->getDuration();
  }
  // No current goal!
  return 0.0;
}

void Arm::cancelGoal() {
  trajectory_ = nullptr;
  trajectory_start_time_ = std::numeric_limits<double>::quiet_NaN();
}

Eigen::VectorXd Arm::getAux(double t) const {
  Eigen::VectorXd res;
  if (aux_times_.size() == 0 || aux_.cols() != aux_times_.size() || aux_.rows() == 0)
    return Eigen::VectorXd();

  // Find the first time
  // TODO: use a tracer for performance here...or at least a std::upper_bound/etc.
  for (int i = aux_times_.size() - 1; i >= 0; --i) {
    if (t >= aux_times_[i]) {
      return aux_.col(i);
    }
  }

  // Note -- should never get here...should always be at least as big as the initial t == 0...
  return aux_.col(0);
}

} // namespace arm
} // namespace experimental
} // namespace hebi
