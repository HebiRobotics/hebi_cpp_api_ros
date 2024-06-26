#pragma once

/**
 * \file mobile_io.hpp
 *
 * Contains utility class and data structures to assist with use of a mobile IO
 * application as a controller for a robotic system.
 */

#include <array>
#include <bitset>
#include <memory>

#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_feedback.hpp"

namespace hebi {
namespace util {

// Wrapper around a mobile IO controller
class MobileIO {
public:
  static constexpr size_t NumButtons = 8;

  enum class ButtonMode { Momentary = 0, Toggle = 1 };

  enum class ButtonState {
    ToOff = -1,    // Edge triggers; these occur if last + current state are different
    Unchanged = 0, // Last + current state are the same
    ToOn = 1       // Edge triggers; these occur if last + current state are different
  };

  static std::unique_ptr<MobileIO> create(const std::string& family, const std::string& name);

  // Call to update the current state.  Returns "true" if feedback was received within the timeout
  // or not.
  bool update(int32_t timeout_ms = Group::DEFAULT_TIMEOUT_MS);

  bool resetUI(bool acknowledge_send = true);

  // Outputs
  // Note: one-indexed to match axes/buttons on the screen

  bool setAxisSnap(int axis_number, float snap_to, bool acknowledge_send = true);
  bool disableAxisSnap(int axis_number, bool acknowledge_send = true) { return setAxisSnap(axis_number, std::numeric_limits<float>::quiet_NaN()); }
  bool setAxisValue(int axis_number, float value, bool acknowledge_send = true);
  bool setAxisLabel(int axis_number, const std::string& message, bool acknowledge_send = true);

  bool setButtonMode(int button_number, ButtonMode mode, bool acknowledge_send = true);
  bool setButtonLed(int button_number, bool on, bool acknowledge_send = true);
  // NB: needs support on mobile IO app side...
  //bool setButtonLed(int button_number, hebi::Color color);
  bool setButtonLabel(int button_number, const std::string& message, bool acknowledge_send = true);

  bool setLedColor(uint8_t r, uint8_t g, uint8_t b, bool acknowledge_send = true);

  bool appendText(const std::string& message, bool acknowledge_send = true);
  bool clearText(bool acknowledge_send = true);

  // Return Feedback object specific to the mobile device (not GroupFeedback)
  const hebi::Feedback& getLastFeedback() const { return fbk_[0]; };

  // Get AR Position
  const Vector3f getArPosition() const { return fbk_[0].mobile().arPosition().get(); }

  // Get AR Orientation
  const Quaternionf getArOrientation() const { return fbk_[0].mobile().arOrientation().get(); }

  // Get current state of axis; one-indexed to match axes on the screen
  float getAxis(int axis) const;
  // Get current state of button; one-indexed to match buttons on the screen
  bool getButton(int button) const;

  // Difference between two IO states, useful for checking to see if a button
  // has been pressed.
  // Note: one-indexed to match buttons on the screen
  ButtonState getButtonDiff(int button) const;

private:
  MobileIO(std::shared_ptr<hebi::Group> group) : group_(std::move(group)), fbk_(group_->size()) {}

  std::shared_ptr<hebi::Group> group_;
  hebi::GroupFeedback fbk_;

  std::bitset<NumButtons> buttons_{};
  std::array<float, NumButtons> axes_{};

  std::bitset<NumButtons> prev_buttons_{};
  std::array<float, NumButtons> prev_axes_{};
};

} // namespace util
} // namespace hebi
