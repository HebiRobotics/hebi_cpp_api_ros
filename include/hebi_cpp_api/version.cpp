#include "hebi_cpp_api/version.hpp"

#include "hebi.h"

#include <cstdio>
#include <exception>

struct VersionChecker {
  VersionChecker() {
    auto version = hebi::getCVersion();
    // 2.15.0 is supported version; 2.15.x works, or 2.>15.x
    if (version.getMajor() != 2 || version.getMinor() < 15) { // || ... version.getRevision() ...) {
      fprintf(stderr, "ERROR: Loaded an incompatible C API version (%d.%d.%d)\n", version.getMajor(),
              version.getMinor(), version.getRevision());
      std::terminate();
    }
  }
};

static VersionChecker check;

namespace hebi {

VersionNumber getCVersion() {
  int32_t maj, min, rev;
  hebiGetLibraryVersion(&maj, &min, &rev);
  return VersionNumber(maj, min, rev);
}

VersionNumber getCppVersion() { return VersionNumber(3, 10, 0); }

} // namespace hebi
