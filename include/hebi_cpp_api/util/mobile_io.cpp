#include "mobile_io.hpp"

#include <stdexcept>

#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/lookup.hpp"

#include <chrono>
#include <thread>

namespace hebi {
namespace util {

std::unique_ptr<MobileIO> MobileIO::create(const std::string& family, const std::string& name) {
  hebi::Lookup lookup;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::shared_ptr<hebi::Group> group = lookup.getGroupFromNames({family}, {name});
  if (!group)
    return nullptr;
  return std::unique_ptr<MobileIO>(new MobileIO(std::move(group)));
}

bool MobileIO::update(int32_t timeout_ms) {
  // Update and return true if we get another packet from the Mobile IO device
  if (group_->getNextFeedback(fbk_, timeout_ms)) {
    prev_buttons_ = buttons_;
    prev_axes_ = axes_;
    // We assume the Mobile IO controller is only ever talking to one
    // device at a time...
    auto& f0 = fbk_[0];
    // Update all the buttons in the current state:
    for (int i = 1; i <= NumButtons; ++i) {
      if (f0.io().b().hasInt(i)) {
        buttons_[i - 1] = f0.io().b().getInt(i) == 1;
      }
    }
    // And the axes
    for (int i = 1; i <= NumButtons; ++i) {
      if (f0.io().a().hasFloat(i)) {
        axes_[i - 1] = f0.io().a().getFloat(i);
      } else if (f0.io().a().hasInt(i)) {
        // IO devices may send exact integers as ints instead of floats
        // to save space on wire, so we check this, too
        axes_[i - 1] = f0.io().a().getInt(i);
      }
    }
    return true;
  }
  return false;
}

bool MobileIO::resetUI(bool acknowledge_send) {
  hebi::GroupCommand cmd(group_->size());
  auto is_joy = [](int i) { return i <= 2 || i >= 7; };
  for (int i = 1; i <= NumButtons; ++i) {
    cmd[0].io().a().setFloat(i, is_joy(i) ? 0 : std::numeric_limits<float>::quiet_NaN());
    cmd[0].io().f().setFloat(i, 0);
    cmd[0].io().b().setInt(i, 0);
    cmd[0].io().e().setInt(i, 0);
    cmd[0].io().a().setLabel(i, "A" + std::to_string(i));
    cmd[0].io().b().setLabel(i, "B" + std::to_string(i));
  }
  cmd[0].clearLog().set();
  cmd[0].led().set(hebi::Color(255, 255, 255, 0));
  if (acknowledge_send)
    return group_->sendCommandWithAcknowledgement(cmd);
  return group_->sendCommand(cmd);
}

bool MobileIO::setAxisSnap(int axis_number, float snap_to, bool acknowledge_send) {
  if (axis_number < 1 || axis_number > NumButtons)
    throw std::out_of_range("Invalid axis number");
  hebi::GroupCommand cmd(group_->size());
  cmd[0].io().a().setFloat(axis_number, snap_to);
  if (acknowledge_send)
    return group_->sendCommandWithAcknowledgement(cmd);
  return group_->sendCommand(cmd);
}

bool MobileIO::setAxisValue(int axis_number, float value, bool acknowledge_send) {
  if (axis_number < 1 || axis_number > NumButtons)
    throw std::out_of_range("Invalid axis number");
  hebi::GroupCommand cmd(group_->size());
  cmd[0].io().f().setFloat(axis_number, value);
  if (acknowledge_send)
    return group_->sendCommandWithAcknowledgement(cmd);
  return group_->sendCommand(cmd);
}

bool MobileIO::setAxisLabel(int axis_number, const std::string& label, bool acknowledge_send) {
  if (axis_number < 1 || axis_number > NumButtons)
    throw std::out_of_range("Invalid axis number");
  hebi::GroupCommand cmd(group_->size());
  cmd[0].io().a().setLabel(axis_number, label);
  if (acknowledge_send)
    return group_->sendCommandWithAcknowledgement(cmd);
  return group_->sendCommand(cmd);
}

bool MobileIO::setButtonMode(int button_number, ButtonMode mode, bool acknowledge_send) {
  if (button_number < 1 || button_number > NumButtons)
    throw std::out_of_range("Invalid button number");
  hebi::GroupCommand cmd(group_->size());
  cmd[0].io().b().setInt(button_number, mode == ButtonMode::Toggle ? 1 : 0);
  if (acknowledge_send)
    return group_->sendCommandWithAcknowledgement(cmd);
  return group_->sendCommand(cmd);
}

bool MobileIO::setButtonLed(int button_number, bool on, bool acknowledge_send) {
  if (button_number < 1 || button_number > NumButtons)
    throw std::out_of_range("Invalid button number");
  hebi::GroupCommand cmd(group_->size());
  cmd[0].io().e().setInt(button_number, on ? 1 : 0);
  if (acknowledge_send)
    return group_->sendCommandWithAcknowledgement(cmd);
  return group_->sendCommand(cmd);
}

// NB: needs support on mobile IO app side...
//bool MobileIO::setButtonLed(int button_number, hebi::Color color) {
//  if (button_number < 1 || button_number > NumButtons)
//    throw std::out_of_range("Invalid button number");
//  hebi::GroupCommand cmd(group_->size());
//  cmd[0].io().e().setInt(button_number, color.toInt());
//  return group_->sendCommand(cmd);
//}

bool MobileIO::setButtonLabel(int button_number, const std::string& message, bool acknowledge_send) {
  if (button_number < 1 || button_number > NumButtons)
    throw std::out_of_range("Invalid button number");
  hebi::GroupCommand cmd(group_->size());
  cmd[0].io().b().setLabel(button_number, message);
  if (acknowledge_send)
    return group_->sendCommandWithAcknowledgement(cmd);
  return group_->sendCommand(cmd);
}

bool MobileIO::setLedColor(uint8_t r, uint8_t g, uint8_t b, bool acknowledge_send) {
  hebi::GroupCommand cmd(group_->size());
  cmd[0].led().set({r, g, b, 255});
  if (acknowledge_send)
    return group_->sendCommandWithAcknowledgement(cmd);
  return group_->sendCommand(cmd);
}

bool MobileIO::appendText(const std::string& message, bool acknowledge_send) {
  hebi::GroupCommand cmd(group_->size());
  cmd[0].appendLog().set(message);
  if (acknowledge_send)
    return group_->sendCommandWithAcknowledgement(cmd);
  return group_->sendCommand(cmd);
}

bool MobileIO::clearText(bool acknowledge_send) {
  hebi::GroupCommand cmd(group_->size());
  cmd[0].clearLog().set();
  if (acknowledge_send)
    return group_->sendCommandWithAcknowledgement(cmd);
  return group_->sendCommand(cmd);
}

float MobileIO::getAxis(int axis) const {
  if (axis < 1 || axis > NumButtons)
    throw std::out_of_range("Invalid axis number");
  return axes_[axis - 1];
}

bool MobileIO::getButton(int button) const {
  if (button < 1 || button > NumButtons)
    throw std::out_of_range("Invalid button number");
  return buttons_[button - 1];
}

MobileIO::ButtonState MobileIO::getButtonDiff(int button) const {
  if (button < 1 || button > NumButtons)
    throw std::out_of_range("Invalid button number");
  if (prev_buttons_[button - 1] == buttons_[button - 1])
    return ButtonState::Unchanged;
  return buttons_[button - 1] ? ButtonState::ToOn : ButtonState::ToOff;
}

} // namespace util
} // namespace hebi
