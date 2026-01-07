#include "hebi_cpp_api/version.hpp"

#include "hebi.h"

#include <cstdio>
#include <exception>

struct VersionChecker {
  VersionChecker() {
    auto version = hebi::getCVersion();
    // 2.23.1 is supported version; 2.23.>=1 works, or 2.>23.x
    hebi::VersionNumber min_supported_version(2, 23, 1);
    if (version.getMajor() != min_supported_version.getMajor() ||
        version.getMinor() < min_supported_version.getMinor() ||
        (version.getMinor() == min_supported_version.getMinor() && version.getRevision() < min_supported_version.getRevision())) {
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

VersionNumber getCppVersion() { return VersionNumber(3, 16, 0); }

} // namespace hebi
