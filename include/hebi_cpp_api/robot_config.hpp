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
  const std::map<std::string, std::string>& getGains() const { return gains_; }
  // Get absolute path of the parent directory of the config file
  const std::string& getParentDirectory() const { return location_; }
  // Get ordered list of plugin parameters
  const std::vector<experimental::arm::PluginConfig>& getPluginConfigs() const { return plugin_configs_; }

  // Each user data field can have a dictionary of three types of parameters.
  struct UserData {
    // Optional boolean values
    std::map<std::string, bool> bools_;
    
    // Optional float values
    std::map<std::string, double> floats_;
    
    // Optional string values
    std::map<std::string, std::string> strings_;
    
    // Optional lists of boolean values
    std::map<std::string, std::vector<bool>> bool_lists_;
    
    // Optional lists of float values
    std::map<std::string, std::vector<double>> float_lists_;
    
    // Optional lists of string values
    std::map<std::string, std::vector<std::string>> string_lists_;

    // Getter for a bool value
    bool getBool(const std::string& key, bool default_value = false) const {
      auto it = bools_.find(key);
      if (it != bools_.end()) {
        return it->second;
      }
      return default_value; // Default value if the key is not found
    }

    // Check if a bool key exists
    bool hasBool(const std::string& key) const {
      return bools_.find(key) != bools_.end();
    }

    // Getter for a float value
    double getFloat(const std::string& key, double default_value = 0.0) const {
      auto it = floats_.find(key);
      if (it != floats_.end()) {
        return it->second;
      }
      return default_value; // Default value if the key is not found
    }

    // Check if a float key exists
    bool hasFloat(const std::string& key) const {
      return floats_.find(key) != floats_.end();
    }

    // Getter for a string value
    std::string getString(const std::string& key, std::string default_value = "") const {
      auto it = strings_.find(key);
      if (it != strings_.end()) {
        return it->second;
      }
      return default_value; // Default value if the key is not found
    }

    // Check if a string key exists
    bool hasString(const std::string& key) const {
      return strings_.find(key) != strings_.end();
    }

    // Getter for a bool list
    std::vector<bool> getBoolList(const std::string& key, std::vector<bool> default_value = {}) const {
      auto it = bool_lists_.find(key);
      if (it != bool_lists_.end()) {
        return it->second;
      }
      return default_value; // Default empty list if the key is not found
    }

    // Check if a bool list key exists
    bool hasBoolList(const std::string& key) const {
      return bool_lists_.find(key) != bool_lists_.end();
    }

    // Getter for a float list
    std::vector<double> getFloatList(const std::string& key, std::vector<double> default_value = {}) const {
      auto it = float_lists_.find(key);
      if (it != float_lists_.end()) {
        return it->second;
      }
      return default_value; // Default empty list if the key is not found
    }

    // Check if a float list key exists
    bool hasFloatList(const std::string& key) const {
      return float_lists_.find(key) != float_lists_.end();
    }

    // Getter for a string list
    std::vector<std::string> getStringList(const std::string& key, std::vector<std::string> default_value = {}) const {
      auto it = string_lists_.find(key);
      if (it != string_lists_.end()) {
        return it->second;
      }
      return default_value; // Default empty list if the key is not found
    }

    // Check if a string list key exists
    bool hasStringList(const std::string& key) const {
      return string_lists_.find(key) != string_lists_.end();
    }
  };

  // Any listed user_data keys
  const UserData& getUserData() const { return user_data_; }

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
  // Absolute path of parent directory
  std::string location_;
  // user_data
  UserData user_data_;
};

} // namespace hebi
