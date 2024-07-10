#include "group_command.hpp"

namespace hebi {

GroupCommand::GroupCommand(size_t number_of_modules)
  : internal_(std::make_shared<GroupCommandWrapper>(number_of_modules)), number_of_modules_(number_of_modules) {
  for (size_t i = 0; i < number_of_modules_; i++)
    commands_.emplace_back(hebiGroupCommandGetModuleCommand(internal_->internal_, i));
}

GroupCommand::GroupCommand(std::shared_ptr<GroupCommandWrapper> internal, std::vector<int> indices)
  : internal_(std::move(internal)), number_of_modules_(indices.size()), is_subview_(true) {
  for (auto i : indices)
    commands_.emplace_back(hebiGroupCommandGetModuleCommand(internal_->internal_, i));
}

GroupCommand GroupCommand::subview(std::vector<int> indices) const {
  for (auto i : indices) {
    if (i < 0 || i >= number_of_modules_)
      throw std::out_of_range("Invalid index when creating subview.");
  }
  return GroupCommand(internal_, indices);
}

size_t GroupCommand::size() const { return number_of_modules_; }

Command& GroupCommand::operator[](size_t index) { return commands_[index]; }

const Command& GroupCommand::operator[](size_t index) const { return commands_[index]; }

void GroupCommand::clear() {
  if (is_subview_)
    return;
  hebiGroupCommandClear(internal_->internal_);
}

bool GroupCommand::readGains(const std::string& file) {
  if (is_subview_)
    return false;
  return hebiGroupCommandReadGains(internal_->internal_, file.c_str()) == HebiStatusSuccess;
}

bool GroupCommand::writeGains(const std::string& file) const {
  if (is_subview_)
    return false;
  return hebiGroupCommandWriteGains(internal_->internal_, file.c_str()) == HebiStatusSuccess;
}

void GroupCommand::setPosition(const Eigen::VectorXd& position) {
  if (position.size() != number_of_modules_)
    return;
  for (size_t i = 0; i < number_of_modules_; ++i)
    commands_[i].actuator().position().set(position[i]);
}
void GroupCommand::setVelocity(const Eigen::VectorXd& velocity) {
  if (velocity.size() != number_of_modules_)
    return;
  for (size_t i = 0; i < number_of_modules_; ++i)
    commands_[i].actuator().velocity().set(static_cast<float>(velocity[i]));
}
void GroupCommand::setEffort(const Eigen::VectorXd& effort) {
  if (effort.size() != number_of_modules_)
    return;
  for (size_t i = 0; i < number_of_modules_; ++i)
    commands_[i].actuator().effort().set(static_cast<float>(effort[i]));
}

void GroupCommand::setSpringConstant(const Eigen::VectorXd& springConstant) {
  if (springConstant.size() != number_of_modules_)
    return;
  for (size_t i = 0; i < number_of_modules_; ++i)
    commands_[i].settings().actuator().springConstant().set(static_cast<float>(springConstant[i]));
}

Eigen::VectorXd GroupCommand::getPosition() const {
  Eigen::VectorXd res(number_of_modules_);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    const auto& cmd = commands_[i].actuator().position();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupCommand::getVelocity() const {
  Eigen::VectorXd res(number_of_modules_);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    const auto& cmd = commands_[i].actuator().velocity();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupCommand::getEffort() const {
  Eigen::VectorXd res(number_of_modules_);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    const auto& cmd = commands_[i].actuator().effort();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}
Eigen::VectorXd GroupCommand::getSpringConstant() const {
  Eigen::VectorXd res(number_of_modules_);
  for (size_t i = 0; i < number_of_modules_; ++i) {
    const auto& cmd = commands_[i].settings().actuator().springConstant();
    res[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
  return res;
}

void GroupCommand::getPosition(Eigen::VectorXd& out) const {
  if (out.size() != number_of_modules_) {
    out.resize(number_of_modules_);
  }

  for (size_t i = 0; i < number_of_modules_; ++i) {
    const auto& cmd = commands_[i].actuator().position();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupCommand::getVelocity(Eigen::VectorXd& out) const {
  if (out.size() != number_of_modules_) {
    out.resize(number_of_modules_);
  }

  for (size_t i = 0; i < number_of_modules_; ++i) {
    const auto& cmd = commands_[i].actuator().velocity();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupCommand::getEffort(Eigen::VectorXd& out) const {
  if (out.size() != number_of_modules_) {
    out.resize(number_of_modules_);
  }

  for (size_t i = 0; i < number_of_modules_; ++i) {
    const auto& cmd = commands_[i].actuator().effort();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}
void GroupCommand::getSpringConstant(Eigen::VectorXd& out) const {
  if (out.size() != number_of_modules_) {
    out.resize(number_of_modules_);
  }

  for (size_t i = 0; i < number_of_modules_; ++i) {
    const auto& cmd = commands_[i].settings().actuator().springConstant();
    out[i] = (cmd) ? cmd.get() : std::numeric_limits<float>::quiet_NaN();
  }
}

} // namespace hebi
