#include "hebi_cpp_api/version.hpp"

#include "hebi.h"

#include <cstdio>
#include <exception>

struct VersionChecker {
  VersionChecker() {
    auto version = hebi::getCVersion();
    // 2.16.3 is supported version; 2.16.>=3 works, or 2.>16.x
    if (version.getMajor() != 2 || version.getMinor() < 16 || (version.getMinor() == 16 && version.getRevision() < 3)) {
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

int getCBuildVersion() {
  return hebiGetLibraryVersionBuild();
}

VersionNumber getCppVersion() { return VersionNumber(3, 12, 2); }

} // namespace hebi
