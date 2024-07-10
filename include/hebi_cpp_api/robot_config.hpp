#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "arm/plugin_config.hpp"

namespace hebi {

// A wrapper for loading robot configurations.  (See https://github.com/HebiRobotics/robot-config)
class RobotConfig {
public:
  // Reads the robot config from the given file.  This reads all included
  // parameters/references, and verifies that all referenced paths exist and
  // load properly.
  //
  // Any errors when loading are added to the passed in vector.  For non-fatal
  // errors, an object may still be returned.
  static std::unique_ptr<RobotConfig> loadConfig(std::string filepath, std::vector<std::string>& errors);

  ///// Accessors for standard data fields /////

  // Return names (required)
  const std::vector<std::string>& getNames() const { return names_; }
  // Return families (required)
  const std::vector<std::string>& getFamilies() const { return families_; }
  // Return HRDF absolute file path (optional)
  const std::string& getHrdf() const { return hrdf_; }
  // Return gain for specific key
  std::string getGains(const std::string& key) const { return gains_.count(key) == 0 ? "" : gains_.at(key); }
  // Return all gains (absolute paths; may be empty)
  const std::map<std::string, std::string>& getGains() { return gains_; }
  // Get ordered list of plugin parameters
  const std::vector<experimental::arm::PluginConfig>& getPluginConfigs() const { return plugin_configs_; }

  // Any listed string keys
  const std::map<std::string, std::string>& getUserData() { return user_data_; }

private:
  RobotConfig() = default;

  std::vector<std::string> names_;
  std::vector<std::string> families_;
  // Stored as an absolute path for reading later
  std::string hrdf_;
  // Stored as absolute paths for reading later
  std::map<std::string, std::string> gains_;
  // plugins
  std::vector<experimental::arm::PluginConfig> plugin_configs_;

  std::map<std::string, std::string> user_data_;
};

} // namespace hebi
