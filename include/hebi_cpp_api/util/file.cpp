#include "file.hpp"

#include <algorithm>

// Working directory fetching
#ifdef WIN32
#include <direct.h>
namespace {
  static auto getCurrentWorkingDir = _getcwd;
}
#else
#include <unistd.h>
namespace {
  static auto getCurrentWorkingDir = getcwd;
}
#endif
#include <cstdio>
#include <sys/stat.h>

namespace hebi {
namespace util {
namespace file {

// Reads contents of file into string; returns empty string on error.
std::string readIntoString(std::string filename)
{
  auto fp = ::fopen(filename.c_str(), "rb");
  if (!fp) return "";
  ::fseek(fp, 0, SEEK_END);
  long sz = ::ftell(fp);
  std::string res(sz, 0);
  if(sz)
  {
    ::rewind(fp);
    size_t ret = ::fread(&res[0], 1, res.size(), fp);
    if (ret != (size_t)sz)
      return "";
  }
  ::fclose(fp);
  return res;
}

File File::getParentDirectory() const
{
  // find "/" or "\", then take everything before it.  Try both here,
  // to better handle cross platform files.
  auto loc = path_.rfind(getPlatformDelimiter());
  if (loc == (path_.size() - 1)) // If last character is a "/", ignore this and re-search.
  {
    if (path_.size() == 1)
      return path_; // single absolute dir; can't go up! 
    // Otherwise, re-search:
    loc = path_.rfind(getPlatformDelimiter(), path_.size() - 2);
  }
  if (loc == std::string::npos)
    return ""; // No delim? return empty...
  return path_.substr(0, loc);
}

bool File::isAbsolute() const
{
  return !path_.empty() && path_[0] == getPlatformDelimiter();
}

void File::append(std::string file_or_dir)
{
  // Remove "move up" directory commands at beginning of path
  int test_location = 0;
  const std::string updir = std::string("..") + getPlatformDelimiter();
  while (file_or_dir.size() >= test_location + 3 && file_or_dir.substr(test_location, 3) == updir)
  {
    test_location += 3;
    if (path_.empty())
    {
      path_ = file_or_dir;
      return; // :( reached end...can't go up anymore!
    }
    path_ = getParentDirectory().path_;
  }

  // Add delim to end if needed:
  if (path_.size() == 0 || *path_.rbegin() != getPlatformDelimiter())
    path_ += getPlatformDelimiter();
  path_ += file_or_dir.substr(test_location);
}

// Get the path of this file relative to the current working directory
std::string File::getAbsolutePath() const
{
  if (isAbsolute())
    return path_;

  // Get working dir
  char path_buff[FILENAME_MAX];
  getCurrentWorkingDir(path_buff, FILENAME_MAX);
  File current_path = File(std::string(path_buff));
  // Append this file
  current_path.append(path_);
  return current_path.path_;
}

bool File::exists() const
{
  struct stat res;
  return (stat(path_.c_str(), &res) == 0 && !(res.st_mode & S_IFDIR));
}

std::string File::convertDelimiters(std::string path)
{
  std::replace(path.begin(), path.end(), getNonPlatformDelimiter(), getPlatformDelimiter());
  return path;
}

} // namespace file
} // namespace util
} // namespace hebi
