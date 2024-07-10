#pragma once

#include "hebi.h"

#include <memory>
#include <vector>

#include "Eigen/Eigen"
#include "group_message_wrapper.hpp"
#include "info.hpp"

namespace hebi {

/**
 * \brief A list of Info objects that can be received from a Group of modules;
 * the size() must match the number of modules in the group.
 */
class GroupInfo final {
public:
#ifndef DOXYGEN_OMIT_INTERNAL
  /**
   * Light wrapper around C-style group info object.
   * NOTE: this should not be used except by library functions!
   */
  std::shared_ptr<GroupInfoWrapper> internal_;
#endif // DOXYGEN_OMIT_INTERNAL

private:
  /**
   * The number of modules in this group info.
   */
  const size_t number_of_modules_;
  /**
   * The list of Info subobjects
   */
  std::vector<Info> infos_;

  /**
   * Is this GroupInfo a subview?
   */
  const bool is_subview_{};

  /**
   * \brief Create a group info subview with the specified number of modules.
   */
  GroupInfo(std::shared_ptr<GroupInfoWrapper>, std::vector<int> indices);

public:
  /**
   * \brief Create a group info with the specified number of modules.
   */
  GroupInfo(size_t number_of_modules);

  /**
   * \brief Destructor cleans up group info object as necessary.
   */
  ~GroupInfo() noexcept = default;

  /**
   * \brief Allows moving result from "subview"
   */
  GroupInfo(GroupInfo&&) = default;

  /**
   * \brief Creates a "subview" of this group info object, with shared
   * access to a subset of the Info elements.
   * Note that certain functions (write gains and write safety
   * parameters and not currently supported on subviews)
   *
   * The indices do not need to remain in order, and do not need to be
   * unique; however, they must each be >= 0 and < size(); otherwise an
   * out_of_range exception is thrown.
   */
  GroupInfo subview(std::vector<int> indices) const;

  /**
   * \brief Was this created as a subview of another GroupInfo?
   */
  bool isSubview() const { return is_subview_; }

  /**
   * \brief Returns the number of module infos in this group info.
   */
  size_t size() const;

  /**
   * \brief Access the info for an individual module.
   */
  const Info& operator[](size_t index) const;

  /**
   * \brief Export the gains from this GroupInfo object into a file, creating it as necessary.
   * \param file The filename (or path + filename) to the file to write to.
   */
  bool writeGains(const std::string& file) const;

  /**
   * \brief Export the safety parameters from this GroupInfo object into a file, creating it as necessary.
   * \param file The filename (or path + filename) to the file to write to.
   */
  FunctionCallResult writeSafetyParameters(const std::string& file) const {
    if (is_subview_)
      return FunctionCallResult{false, "Cannot call this method on a subview!"};
    auto res = hebiGroupInfoWriteSafetyParameters(internal_->internal_, file.c_str()) == HebiStatusSuccess;
    if (res) {
      return FunctionCallResult{true};
    }
    return FunctionCallResult{false, std::string{hebiSafetyParametersGetLastError()}};
  }

  /**
   * \brief Convenience function for returning spring constant values.
   */
  Eigen::VectorXd getSpringConstant() const;

  /**
   * \brief Convenience function for returning spring constant values.
   */
  void getSpringConstant(Eigen::VectorXd& out) const;
};

} // namespace hebi
