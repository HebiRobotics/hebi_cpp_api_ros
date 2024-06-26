#pragma once

#include "hebi.h"

#include <memory>
#include <vector>

#include "Eigen/Eigen"
#include "feedback.hpp"
#include "group_message_wrapper.hpp"

namespace hebi {

/**
 * \brief A list of Feedback objects that can be received from a Group of
 * modules; the size() must match the number of modules in the group.
 */
class GroupFeedback final {
public:
#ifndef DOXYGEN_OMIT_INTERNAL
  /**
   * Light wrapper around C-style group feedback object.
   * NOTE: this should not be used except by library functions!
   */
  std::shared_ptr<GroupFeedbackWrapper> internal_;
#endif // DOXYGEN_OMIT_INTERNAL

private:
  /**
   * The number of modules in this group feedback.
   */
  const size_t number_of_modules_;
  /**
   * The list of Feedback subobjects
   */
  std::vector<Feedback> feedbacks_;
  /**
   * Is this GroupFeedback a subview?
   */
  const bool is_subview_{};

  /**
   * \brief Create a group feedback subview with the specified number of modules.
   */
  GroupFeedback(std::shared_ptr<GroupFeedbackWrapper>, std::vector<int> indices);

public:
  /**
   * \brief Create a group feedback with the specified number of modules.
   */
  GroupFeedback(size_t number_of_modules);
#ifndef DOXYGEN_OMIT_INTERNAL
  /**
   * Wraps an existing C-style feedback object; object lifetime is assumed to
   * be managed by the caller.
   * NOTE: this should not be used except by internal library functions!
   */
  GroupFeedback(HebiGroupFeedbackPtr group_feedback);
#endif // DOXYGEN_OMIT_INTERNAL

  /**
   * \brief Destructor cleans up group feedback object as necessary.
   */
  ~GroupFeedback() noexcept = default;

  /**
   * \brief Allows moving result from "subview"
   */
  GroupFeedback(GroupFeedback&&) = default;

  /**
   * \brief Creates a "subview" of this group feedback object, with shared
   * access to a subset of the Feedback elements.
   *
   * The indices do not need to remain in order, and do not need to be
   * unique; however, they must each be >= 0 and < size(); otherwise an
   * out_of_range exception is thrown.
   */
  GroupFeedback subview(std::vector<int> indices) const;

  /**
   * \brief Was this created as a subview of another GroupFeedback?
   */
  bool isSubview() const { return is_subview_; }

  /**
   * \brief Returns the number of module feedbacks in this group feedback.
   */
  size_t size() const;

  /**
   * \brief Access the feedback for an individual module.
   */
  const Feedback& operator[](size_t index) const;

  /**
   * \brief Gets a single wall-clock timestamp (in seconds since the epoch)
   * associated with the entire group feedback (this is time the last feedback
   * packet arrived).
   *
   * \returns the maximum value of each feedback's PC receive time, or NaN if
   * not all feedbacks are populated.
   */
  double getTime() const;

  /**
   * \brief Gets a single wall-clock timestamp (in microseconds since the epoch)
   * associated with the entire group feedback (this is time the last feedback
   * packet arrived).
   *
   * \returns the maximum value of each feedback's PC receive time, or 0 if
   * not all feedbacks are populated.
   */
  uint64_t getTimeUs() const;

  /**
   * \brief Convenience function for returning feedback board temperature values.
   */
  Eigen::VectorXd getBoardTemperature() const;
  /**
   * \brief Convenience function for returning feedback processor temperature values.
   */
  Eigen::VectorXd getProcessorTemperature() const;
  /**
   * \brief Convenience function for returning feedback voltage values.
   */
  Eigen::VectorXd getVoltage() const;
  /**
   * \brief Convenience function for returning feedback deflection values.
   */
  Eigen::VectorXd getDeflection() const;
  /**
   * \brief Convenience function for returning feedback deflection velocity values.
   */
  Eigen::VectorXd getDeflectionVelocity() const;
  /**
   * \brief Convenience function for returning feedback motor velocity values.
   */
  Eigen::VectorXd getMotorVelocity() const;
  /**
   * \brief Convenience function for returning feedback motor current values.
   */
  Eigen::VectorXd getMotorCurrent() const;
  /**
   * \brief Convenience function for returning feedback motor sensor temperature values.
   */
  Eigen::VectorXd getMotorSensorTemperature() const;
  /**
   * \brief Convenience function for returning feedback motor winding current values.
   */
  Eigen::VectorXd getMotorWindingCurrent() const;
  /**
   * \brief Convenience function for returning feedback motor winding temperature values.
   */
  Eigen::VectorXd getMotorWindingTemperature() const;
  /**
   * \brief Convenience function for returning feedback motor housing temperature values.
   */
  Eigen::VectorXd getMotorHousingTemperature() const;

  /**
   * \brief Convenience function for returning feedback position values.
   */
  Eigen::VectorXd getPosition() const;
  /**
   * \brief Convenience function for returning feedback velocity values.
   */
  Eigen::VectorXd getVelocity() const;
  /**
   * \brief Convenience function for returning feedback effort values.
   */
  Eigen::VectorXd getEffort() const;

  /**
   * \brief Convenience function for returning commanded position values.
   */
  Eigen::VectorXd getPositionCommand() const;
  /**
   * \brief Convenience function for returning commanded velocity values.
   */
  Eigen::VectorXd getVelocityCommand() const;
  /**
   * \brief Convenience function for returning commanded effort values.
   */
  Eigen::VectorXd getEffortCommand() const;

  /**
   * \brief Convenience function for returning feedback accelerometer values.
   */
  Eigen::MatrixX3d getAccelerometer() const;
  /**
   * \brief Convenience function for returning feedback gyroscope values.
   */
  Eigen::MatrixX3d getGyro() const;

  /**
   *\brief Convenience function for returning feedback board temperature values.
   */
  void getBoardTemperature(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback processor temperature values.
   */
  void getProcessorTemperature(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback voltage values.
   */
  void getVoltage(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback deflection values.
   */
  void getDeflection(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback deflection velocity values.
   */
  void getDeflectionVelocity(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback motor velocity values.
   */
  void getMotorVelocity(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback motor current values.
   */
  void getMotorCurrent(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback motor sensor temperature values.
   */
  void getMotorSensorTemperature(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback motor winding current values.
   */
  void getMotorWindingCurrent(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback motor winding temperature values.
   */
  void getMotorWindingTemperature(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback motor housing temperature values.
   */
  void getMotorHousingTemperature(Eigen::VectorXd& out) const;

  /**
   * \brief Convenience function for returning feedback position values.
   */
  void getPosition(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback velocity values.
   */
  void getVelocity(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning feedback effort values.
   */
  void getEffort(Eigen::VectorXd& out) const;

  /**
   * \brief Convenience function for returning commanded position values.
   */
  void getPositionCommand(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning commanded velocity values.
   */
  void getVelocityCommand(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning commanded effort values.
   */
  void getEffortCommand(Eigen::VectorXd& out) const;

  /**
   * \brief Convenience function for returning feedback accelerometer values.
   */
  void getAccelerometer(Eigen::MatrixX3d& out) const;
  /**
   * \brief Convenience function for returning feedback gyroscope values.
   */
  void getGyro(Eigen::MatrixX3d& out) const;
};

} // namespace hebi
