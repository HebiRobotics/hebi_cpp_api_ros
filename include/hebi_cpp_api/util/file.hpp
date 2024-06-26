#pragma once

#include <string>

namespace hebi {
namespace util {
namespace file {

// Reads contents of file into string; returns empty string on error.
std::string readIntoString(std::string filename);

// Small wrapper class to handle paths.  Stores in os-specific encoding.
class File
{
public:
  File(const char* path) : path_(convertDelimiters(std::string(path)))
  { }
  File(std::string path) : path_(convertDelimiters(path))
  { }

  File getParentDirectory() const;

  bool isAbsolute() const;

  // Append, removing "move up" directory commands at the beginning of path
  void append(std::string file_or_dir);

  // Get the path of this file relative to the current working directory
  std::string getAbsolutePath() const;

  bool exists() const;

  static constexpr char WinDelimiter = '\\';
  static constexpr char PosixDelimiter = '/';

  static constexpr char getPlatformDelimiter()
  {
    #ifdef WIN32
    return WinDelimiter;
    #else
    return PosixDelimiter;
    #endif
  }
  static constexpr char getNonPlatformDelimiter()
  {
    #ifdef WIN32
    return PosixDelimiter;
    #else
    return WinDelimiter;
    #endif
  }
private:
  static std::string convertDelimiters(std::string path);

  std::string path_;
};

} // namespace file
} // namespace util
} // namespace hebi
