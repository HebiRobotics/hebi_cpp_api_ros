#pragma once

#include <map>
#include <string>
#include <vector>

namespace hebi {
namespace experimental {
namespace arm {

// Each plugin can have a dictionary of three types of parameters.
struct PluginConfig {
  PluginConfig(const std::string& type, const std::string& name)
    : type_(type), name_(name)
  { }
  std::string type_;
  std::string name_;
  std::map<std::string, bool> bools_;
  std::map<std::string, float> floats_;
  std::map<std::string, std::string> strings_;
  std::map<std::string, std::vector<bool>> bool_lists_;
  std::map<std::string, std::vector<float>> float_lists_;
  std::map<std::string, std::vector<std::string>> string_lists_;
};
    
} // namespace arm
} // namespace experimental
} // namespace hebi