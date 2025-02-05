#pragma once

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group.hpp"

namespace hebi {
namespace arm {

// This is a general end effector that can be added to the end of the arm.
// Override this class to create end effectors for particular purposes.
class EndEffectorBase {
public:
  EndEffectorBase(size_t aux_size) : command_(aux_size), feedback_(aux_size) {};
  virtual ~EndEffectorBase() = default;
  // Updates feedback and sets aux state.
  virtual bool update(Eigen::VectorXd& /*aux_state*/) = 0;
  // Sends command
  virtual bool send() = 0;

  hebi::GroupCommand& pendingCommand() { return command_; }

  const hebi::GroupCommand& pendingCommand() const { return command_; }

protected:

  hebi::GroupCommand command_;
  hebi::GroupFeedback feedback_;
};

// An effort-controlled gripper, sending commands to HEBI modules.
// 
// Example usage for effort-controller gripper:
// 
// auto gripper = Gripper::create("Arm", "gripperSpool");
// Eigen::VectorXd aux_state(1);
// aux_state.setConstant(0);
// while(true) {
//   aux_state = updateAuxState(); // Fill in this state from somewhere...
//   end_effector->update(aux_state);
//   end_effector->send();
// }
//
// Note -- you will probably want to check the return values of update and send to
// verify module connection is stable.
// 
// Note that this is designed to be used with the arm API, but can also be used independently
// if desired.
class Gripper : public EndEffectorBase {

public:
  ~Gripper() override = default;

  enum class State {
    Open, Close
  };

  // Conversion of State enum to internal double representation here
  static double StateToDouble(State state) {
    return state == State::Open ? 0.0 : 1.0;
  }

  // Create a gripper group for 1 module, using the module's family and name.
  static std::unique_ptr<Gripper> create(const std::string& family, const std::string& name, double close_effort, double open_effort) {
    hebi::Lookup lookup;
    if (auto group = getGroup(lookup, family, name))
      return std::unique_ptr<Gripper>(new Gripper(group, close_effort, open_effort));
    return nullptr;
  }

  // Use an existing group
  static std::unique_ptr<Gripper> create(std::shared_ptr<hebi::Group>& group, double close_effort, double open_effort) {
    if (!group || group->size() != 1)
      return nullptr;
    return std::unique_ptr<Gripper>(new Gripper(group, close_effort, open_effort));
  }

  // Use an existing lookup object
  static std::unique_ptr<Gripper> create(hebi::Lookup& lookup, const std::string& family, const std::string& name, double close_effort, double open_effort) {
    if (auto group = getGroup(lookup, family, name))
      return std::unique_ptr<Gripper>(new Gripper(group, close_effort, open_effort));
    return nullptr;
  }

  // Implementation of EndEffectorBase
  //
  // Updates feedback and sets aux state.
  // State of size "0" indicates no change. Values of "nan" also indicate
  // no change.
  // Invalid inputs result in a "false" return value, with no
  // command set.
  bool update(Eigen::VectorXd& aux_state) override {
    // Check for valid aux state:
    auto n = aux_state.size();
    if (n > 1)
      return false;

    // Set aux state when given:
    if (n == 1 && std::isfinite(aux_state[0])) {
      setCommand(aux_state[0]);
    }

    return group_->getNextFeedback(feedback_);
  }

  // Implementation of EndEffectorBase
  //
  // Sends command to gripper.
  bool send() override {
    return group_->sendCommand(command_);
  }

  // Gets the commanded state of the gripper. As this can be
  // set to intermediate values, returns a double instead of
  // a State enum.  Use StateToDouble to convert from State
  // to this double value.
  double getState() {
    return state_;
  }

  // Sets the gripper to the given state
  void setState(double state) {
    setCommand(state);
  }

  // Sets the gripper to the given state
  void setState(State state) {
    setCommand(StateToDouble(state));
  }

  // Sets the gripper to be fully open
  void open() {
    setState(State::Open);
  }

  // Sets the gripper to be fully closed.
  void close() {
    setState(State::Close);
  }

  // Toggle the state of the gripper.
  //
  // If the gripper was commanded more than half closed, it will become fully open.
  // Otherwise it will become fully closed
  void toggle() {
    // Note -- still works it value has been manually set to non-0/1 values
    if (state_ <= 0.5)
      setCommand(1.0);
    else
      setCommand(0.0);
  }

  // Loads gains from the given .xml file, and sends them to the gripper group.
  // Returns false if the gains file could not be found, if these is a mismatch
  // in number of modules, or the modules do not acknowledge receipt of the
  // gains.
  bool loadGains(const std::string& gains_file) {
    hebi::GroupCommand gains_cmd(group_->size());
    if (!gains_cmd.readGains(gains_file))
      return false;

    return group_->sendCommandWithAcknowledgement(gains_cmd);
  }

protected:
  // Typed setters depending on class type
  void setCommand(double value) {
    command_[0].actuator().effort().set(value * close_effort_ + (1.0 - value) * open_effort_);
    state_ = value;
  }

private:
  Gripper(std::shared_ptr<hebi::Group> group, double close_effort, double open_effort)
    : EndEffectorBase(1), group_(group), close_effort_(close_effort), open_effort_(open_effort) {
      setCommand(state_);
  }

  static std::shared_ptr<hebi::Group> getGroup(Lookup& lookup, const std::string& family, const std::string& name) {
    auto group = lookup.getGroupFromNames(std::vector<std::string>{ family}, std::vector<std::string>{ name });
    if (!group || group->size() != 1)
      return nullptr;
    return group;
  }

  std::shared_ptr<hebi::Group> group_;

  double close_effort_{};
  double open_effort_{};
  // The current state of the gripper. Range of the value is [0.0, 1.0]
  double state_{};
};

} // namespace arm
} // namespace hebi
