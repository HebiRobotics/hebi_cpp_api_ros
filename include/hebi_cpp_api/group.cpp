#include "hebi_cpp_api/group.hpp"

#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/group_info.hpp"
#include "hebi_cpp_api/log_file.hpp"

namespace hebi {

#ifndef DOXYGEN_OMIT_INTERNAL
void callbackWrapper(HebiGroupFeedbackPtr group_feedback, void* user_data) {
  reinterpret_cast<Group*>(user_data)->callAttachedHandlers(group_feedback);
}
#endif // DOXYGEN_OMIT_INTERNAL

void Group::callAttachedHandlers(HebiGroupFeedbackPtr group_feedback) {
  // Wrap this:
  GroupFeedback wrapped_fbk(group_feedback);
  // Call handlers:
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  for (unsigned int i = 0; i < handlers_.size(); i++) {
    GroupFeedbackHandler handler = handlers_[i];
    try {
      handler(wrapped_fbk);
    } catch (...) {
    }
  }
}

Group::Group(HebiGroupPtr group, float initial_feedback_frequency, int32_t initial_command_lifetime)
  : internal_(group), number_of_modules_(hebiGroupGetSize(internal_)) {
  if (initial_feedback_frequency != 0)
    setFeedbackFrequencyHz(initial_feedback_frequency);
  if (initial_command_lifetime != 0)
    setCommandLifetimeMs(initial_command_lifetime);
}

std::shared_ptr<Group> Group::createImitation(size_t size) {
  return std::make_shared<Group>(hebiGroupCreateImitation(size));
}

Group::~Group() noexcept {
  // Cleanup group object allocated by the C library
  if (internal_ != nullptr)
    hebiGroupRelease(internal_);
}

int Group::size() const { return number_of_modules_; }

bool Group::setCommandLifetimeMs(int32_t ms) {
  return (hebiGroupSetCommandLifetime(internal_, ms) == HebiStatusSuccess);
}

int32_t Group::getCommandLifetimeMs() const {
  return hebiGroupGetCommandLifetime(internal_);
}

bool Group::sendCommand(const GroupCommand& group_command) {
  // Note -- should not use this with a subview!
  if (group_command.isSubview())
    return false;
  return (hebiGroupSendCommand(internal_, group_command.internal_->internal_) == HebiStatusSuccess);
}

bool Group::sendCommandWithAcknowledgement(const GroupCommand& group_command, int32_t timeout_ms) {
  // Note -- should not use this with a subview!
  if (group_command.isSubview())
    return false;
  return (hebiGroupSendCommandWithAcknowledgement(internal_, group_command.internal_->internal_, timeout_ms) == HebiStatusSuccess);
}

bool Group::sendFeedbackRequest() { return (hebiGroupSendFeedbackRequest(internal_) == HebiStatusSuccess); }

bool Group::getNextFeedback(GroupFeedback& feedback, int32_t timeout_ms) {
  // Note -- should not use this with a subview!
  if (feedback.isSubview())
    return false;
  return (hebiGroupGetNextFeedback(internal_, feedback.internal_->internal_, timeout_ms) == HebiStatusSuccess);
}

bool Group::requestInfo(GroupInfo& info, int32_t timeout_ms) {
  // Note -- should not use this with a subview!
  if (info.isSubview())
    return false;
  return (hebiGroupRequestInfo(internal_, info.internal_->internal_, timeout_ms) == HebiStatusSuccess);
}

bool Group::requestInfoExtra(GroupInfo& info, InfoExtraFields extra_fields, int32_t timeout_ms) {
  // Note -- should not use this with a subview!
  if (info.isSubview())
    return false;
  return (hebiGroupRequestInfoExtra(internal_, info.internal_->internal_, static_cast<uint64_t>(extra_fields), timeout_ms) == HebiStatusSuccess);
}

std::string Group::startLog(const std::string& dir) const {
  HebiStringPtr str;
  if (hebiGroupStartLog(internal_, dir.c_str(), nullptr, &str) == HebiStatusSuccess) {
    assert(str);
    size_t len;

    hebiStringGetString(str, nullptr, &len);
    char* buffer = new char[len];
    hebiStringGetString(str, buffer, &len);
    std::string ret(buffer, --len);

    delete[] buffer;
    hebiStringRelease(str);

    return ret;
  }
  return "";
}

std::string Group::startLog(const std::string& dir, const std::string& file) const {
  HebiStringPtr str;
  if (hebiGroupStartLog(internal_, dir.c_str(), file.c_str(), &str) == HebiStatusSuccess) {
    assert(str);
    size_t len;

    hebiStringGetString(str, nullptr, &len);
    char* buffer = new char[len];
    hebiStringGetString(str, buffer, &len);
    std::string ret(buffer, --len);

    delete[] buffer;
    hebiStringRelease(str);

    return ret;
  }
  return "";
}

std::shared_ptr<LogFile> Group::stopLog() const {
  auto internal = hebiGroupStopLog(internal_);
  if (internal == nullptr) {
    return std::shared_ptr<LogFile>();
  }

  return std::shared_ptr<LogFile>(new LogFile(internal, hebiLogFileGetNumberOfModules(internal)));
}

bool Group::logUserState(const UserState& state) const {
  HebiUserState c_state;
  // Copy data to avoid const double* values in the source message. Could also const_cast when
  // setting or have immutable data structure in future C API version. 
  std::array<double, 9> tmp_values = state.state_values_;
  c_state.state1 = state.has_state_bits_[0] ? &tmp_values[0] : nullptr;
  c_state.state2 = state.has_state_bits_[1] ? &tmp_values[1] : nullptr;
  c_state.state3 = state.has_state_bits_[2] ? &tmp_values[2] : nullptr;
  c_state.state4 = state.has_state_bits_[3] ? &tmp_values[3] : nullptr;
  c_state.state5 = state.has_state_bits_[4] ? &tmp_values[4] : nullptr;
  c_state.state6 = state.has_state_bits_[5] ? &tmp_values[5] : nullptr;
  c_state.state7 = state.has_state_bits_[6] ? &tmp_values[6] : nullptr;
  c_state.state8 = state.has_state_bits_[7] ? &tmp_values[7] : nullptr;
  c_state.state9 = state.has_state_bits_[8] ? &tmp_values[8] : nullptr;
  return hebiGroupLogUserState(internal_, c_state) == HebiStatusSuccess;
}

bool Group::setFeedbackFrequencyHz(float frequency) {
  return (hebiGroupSetFeedbackFrequencyHz(internal_, frequency) == HebiStatusSuccess);
}

float Group::getFeedbackFrequencyHz() const { return hebiGroupGetFeedbackFrequencyHz(internal_); }

void Group::addFeedbackHandler(GroupFeedbackHandler handler) {
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  handlers_.push_back(handler);
  if (handlers_.size() == 1) // (i.e., this was the first one)
    hebiGroupRegisterFeedbackHandler(internal_, callbackWrapper, reinterpret_cast<void*>(this));
}

void Group::clearFeedbackHandlers() {
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  hebiGroupClearFeedbackHandlers(internal_);
  handlers_.clear();
}

} // namespace hebi
