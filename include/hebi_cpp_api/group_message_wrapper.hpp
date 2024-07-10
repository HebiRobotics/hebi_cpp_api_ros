#pragma once

#include "hebi.h"

namespace hebi {

class GroupFeedbackWrapper final {
public:
#ifndef DOXYGEN_OMIT_INTERNAL
  /**
   * C-style group feedback object.
   * NOTE: this should not be used except by library functions!
   */
  HebiGroupFeedbackPtr internal_;
#endif // DOXYGEN_OMIT_INTERNAL
private:
  /**
   * True if this object is responsible for creating and destroying the
   * underlying C pointer; false otherwise.
   */
  const bool manage_pointer_lifetime_;

public:
  /**
   * \brief Create a group feedback with the specified number of modules.
   */
  GroupFeedbackWrapper(size_t number_of_modules)
    : internal_(hebiGroupFeedbackCreate(number_of_modules)), manage_pointer_lifetime_(true) {}
#ifndef DOXYGEN_OMIT_INTERNAL
  /**
   * Wraps an existing C-style feedback object; object lifetime is assumed to
   * be managed by the caller.
   * NOTE: this should not be used except by internal library functions!
   */
  GroupFeedbackWrapper(HebiGroupFeedbackPtr group_feedback)
    : internal_(group_feedback), manage_pointer_lifetime_(false) {}
#endif // DOXYGEN_OMIT_INTERNAL
  /**
   * \brief Destructor cleans up group feedback object as necessary.
   */
  ~GroupFeedbackWrapper() noexcept {
    if (manage_pointer_lifetime_ && internal_ != nullptr)
      hebiGroupFeedbackRelease(internal_);
  }

  // Delete copy (and implied delete of move) constructors/operators
  GroupFeedbackWrapper(const GroupFeedbackWrapper&) = delete;
  GroupFeedbackWrapper& operator=(const GroupFeedbackWrapper&) = delete;
};

class GroupCommandWrapper final {
public:
#ifndef DOXYGEN_OMIT_INTERNAL
  /**
   * C-style group command object.
   * NOTE: this should not be used except by library functions!
   */
  HebiGroupCommandPtr internal_;
#endif // DOXYGEN_OMIT_INTERNAL
  /**
   * \brief Create a group command with the specified number of modules.
   */
  GroupCommandWrapper(size_t number_of_modules) : internal_(hebiGroupCommandCreate(number_of_modules)) {}
  /**
   * \brief Destructor cleans up group command object as necessary.
   */
  ~GroupCommandWrapper() noexcept {
    if (internal_ != nullptr)
      hebiGroupCommandRelease(internal_);
  }

  // Delete copy (and implied delete of move) constructors/operators
  GroupCommandWrapper(const GroupCommandWrapper&) = delete;
  GroupCommandWrapper& operator=(const GroupCommandWrapper&) = delete;
};

class GroupInfoWrapper final {
public:
#ifndef DOXYGEN_OMIT_INTERNAL
  /**
   * C-style group command object.
   * NOTE: this should not be used except by library functions!
   */
  HebiGroupInfoPtr internal_;
#endif // DOXYGEN_OMIT_INTERNAL
  /**
   * \brief Create a group command with the specified number of modules.
   */
  GroupInfoWrapper(size_t number_of_modules) : internal_(hebiGroupInfoCreate(number_of_modules)) {}
  /**
   * \brief Destructor cleans up group command object as necessary.
   */
  ~GroupInfoWrapper() noexcept {
    if (internal_ != nullptr)
      hebiGroupInfoRelease(internal_);
  }

  // Delete copy (and implied delete of move) constructors/operators
  GroupInfoWrapper(const GroupInfoWrapper&) = delete;
  GroupInfoWrapper& operator=(const GroupInfoWrapper&) = delete;
};

} // namespace hebi
