#pragma once

#include "hebi.h"

#include <memory>
#include <vector>

#include "Eigen/Eigen"
#include "command.hpp"
#include "group_message_wrapper.hpp"
#include "util.hpp"

namespace hebi {

/**
 * \brief A list of Command objects appropriate for sending to a Group of
 * modules; the size() must match the number of modules in the group.
 */
class GroupCommand final {
public:
#ifndef DOXYGEN_OMIT_INTERNAL
  /**
   * Light wrapper around C-style group command object.
   * NOTE: this should not be used except by library functions!
   */
  std::shared_ptr<GroupCommandWrapper> internal_;
#endif // DOXYGEN_OMIT_INTERNAL

private:
  /**
   * The number of modules in this group command.
   */
  const size_t number_of_modules_;
  /**
   * The list of Command subobjects
   */
  std::vector<Command> commands_;
  /**
   * Is this GroupCommand a subview?
   */
  const bool is_subview_{};

  /**
   * \brief Create a group command subview with the specified number of modules.
   */
  GroupCommand(std::shared_ptr<GroupCommandWrapper>, std::vector<int> indices);

public:
  /**
   * \brief Create a group command with the specified number of modules.
   */
  GroupCommand(size_t number_of_modules);

  /**
   * \brief Destructor cleans up group command object as necessary.
   */
  ~GroupCommand() noexcept = default;

  /**
   * \brief Allows moving result from "subview"
   */
  GroupCommand(GroupCommand&&) = default;

  /**
   * \brief Creates a "subview" of this group command object, with shared
   * access to a subset of the Command elements.
   * Note that certain functions (clear, read/write gains, and read/write
   * safety parameters and not currently supported on subviews)
   *
   * The indices do not need to remain in order, and do not need to be
   * unique; however, they must each be >= 0 and < size(); otherwise an
   * out_of_range exception is thrown.
   */
  GroupCommand subview(std::vector<int> indices) const;

  /**
   * \brief Was this created as a subview of another GroupCommand?
   */
  bool isSubview() const { return is_subview_; }

  /**
   * \brief Returns the number of module commands in this group command.
   */
  size_t size() const;

  /**
   * \brief Access the command for an individual module.
   */
  Command& operator[](size_t index);

  /**
   * \brief Access the command for an individual module.
   */
  const Command& operator[](size_t index) const;

  /**
   * \brief Clears all data in this GroupCommand object; this returns to the
   * state the GroupCommand was at time of creation.
   */
  void clear();

  /**
   * \brief Import the gains from a file into this GroupCommand object.
   * \param file The filename (or path + filename) to the file to read from.
   */
  bool readGains(const std::string& file);

  /**
   * \brief Export the gains from this GroupCommand object into a file, creating it as necessary.
   * \param file The filename (or path + filename) to the file to write to.
   */
  bool writeGains(const std::string& file) const;

  /**
   * \brief Import the safety parameters from a file into this GroupCommand object.
   * \param file The filename (or path + filename) to the file to read from.
   */
  FunctionCallResult readSafetyParameters(const std::string& file) {
    if (is_subview_)
      return FunctionCallResult{false, "Cannot call this method on a subview!"};
    auto res = hebiGroupCommandReadSafetyParameters(internal_->internal_, file.c_str()) == HebiStatusSuccess;
    if (res) {
      return FunctionCallResult{true};
    }
    return FunctionCallResult{false, std::string{hebiSafetyParametersGetLastError()}};
  }

  /**
   * \brief Export the safety parameters from this GroupCommand object into a file, creating it as necessary.
   * \param file The filename (or path + filename) to the file to write to.
   */
  FunctionCallResult writeSafetyParameters(const std::string& file) const {
    if (is_subview_)
      return FunctionCallResult{false, "Cannot call this method on a subview!"};
    auto res = hebiGroupCommandWriteSafetyParameters(internal_->internal_, file.c_str()) == HebiStatusSuccess;
    if (res) {
      return FunctionCallResult{true};
    }
    return FunctionCallResult{false, std::string{hebiSafetyParametersGetLastError()}};
  }

  /**
   * \brief Convenience function for setting position commands from Eigen
   * vectors.
   *
   * Note that if the vector is not the correct link, no action is taken.
   */
  void setPosition(const Eigen::VectorXd& position);
  /**
   * \brief Convenience function for setting velocity commands from Eigen
   * vectors.
   *
   * Note that if the vector is not the correct link, no action is taken.
   */
  void setVelocity(const Eigen::VectorXd& velocity);
  /**
   * \brief Convenience function for setting effort commands from Eigen
   * vectors.
   *
   * Note that if the vector is not the correct link, no action is taken.
   */
  void setEffort(const Eigen::VectorXd& effort);
  /**
   * \brief Convenience function for setting spring constant commands from Eigen
   * vectors.
   *
   * Note that if the vector is not the correct link, no action is taken.
   */
  void setSpringConstant(const Eigen::VectorXd& springConstant);

  /**
   * \brief Convenience function for returning commanded position values.
   */
  Eigen::VectorXd getPosition() const;
  /**
   * \brief Convenience function for returning commanded velocity values.
   */
  Eigen::VectorXd getVelocity() const;
  /**
   * \brief Convenience function for returning commanded effort values.
   */
  Eigen::VectorXd getEffort() const;
  /**
   * \brief Convenience function for returning commanded spring constant values.
   */
  Eigen::VectorXd getSpringConstant() const;

  /**
   * \brief Convenience function for returning commanded position values.
   */
  void getPosition(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning commanded velocity values.
   */
  void getVelocity(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning commanded effort values.
   */
  void getEffort(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning commanded spring constant values.
   */
  void getSpringConstant(Eigen::VectorXd& out) const;
};

} // namespace hebi
