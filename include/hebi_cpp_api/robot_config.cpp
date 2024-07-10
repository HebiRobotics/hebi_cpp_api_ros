#include "robot_config.hpp"

#include <set>

#include "hebi_cpp_api/rapidyaml/rapidyaml-0.5.0.hpp"
#include "util/file.hpp"

namespace {

struct ConfigVersion {
  size_t major;
  size_t minor;
};

static constexpr ConfigVersion MinVersion = {1, 0}; // Don't support things below this!
static constexpr ConfigVersion MyVersion = {1, 0}; // Current highest fully supported version

bool getVersionFromString(const std::string str_ver, ConfigVersion& version_out) {
  size_t dot = str_ver.find('.');
  if (dot == std::string::npos || dot == 0 || dot == (str_ver.size() - 1))
    return false; // can't parse! no dot, or it is first or last item.
  version_out.major = 0;
  version_out.minor = 0;
  for (size_t i = 0; i < dot; ++i) {
    if (str_ver[i] < '0' || str_ver[i] > '9') // bad character!
      return false;
    version_out.major = version_out.major * 10 + static_cast<size_t>(str_ver[i] - '0');
  }
  for (size_t i = dot + 1; i < str_ver.size(); ++i) {
    if (str_ver[i] < '0' || str_ver[i] > '9') // bad character!
      return false;
    version_out.minor = version_out.minor * 10 + static_cast<size_t>(str_ver[i] - '0');
  }
  return true;
}

} // anonymous namespace

namespace hebi {

std::unique_ptr<RobotConfig> RobotConfig::loadConfig(std::string filepath, std::vector<std::string>& errors) {
  // Try to load .cfg file:
  util::file::File cfg_file(filepath);
  auto parent_dir_absolute = cfg_file.getParentDirectory().getAbsolutePath();

  if (!cfg_file.exists()) {
    errors.push_back("Could not find config file at " + filepath);
    return {};
  }

  // Parse using rapidyaml library
  std::string config_contents = util::file::readIntoString(filepath);
  if (config_contents.empty()) {
    errors.push_back("Could not read file");
    return {};
  }

  ryml::Tree config_file = ryml::parse_in_arena(ryml::to_csubstr(config_contents));
  ryml::ConstNodeRef root = config_file.rootref();

  // fill in config object:
  std::unique_ptr<RobotConfig> config_result(new RobotConfig());

  if (!root.valid()) {
    errors.push_back("Config file not valid yaml.");
    return {};
  }
  if (root.empty()) {
    errors.push_back("Config file is empty.");
    return {};
  }

  // Version must exist:
  ::ConfigVersion config_version;
  bool strict = true;
  if (root.has_child("version")) {
    auto version = config_file["version"];
    if (!version.is_keyval() || !version.val().has_str()) {
      errors.push_back("'version' not a string value.");
      return {};
    }
    if (!::getVersionFromString(std::string(version.val().str, version.val().len), config_version)) {
      errors.push_back("Could not parse 'version' field");
      return {};
    }
    // API supporting < major version: error that it cannot parse version
    if (config_version.major > ::MyVersion.major) {
      errors.push_back(std::string("API does not support config files with major version above ") + std::to_string(::MyVersion.major));;
      return {};
    }
    if (config_version.major < ::MinVersion.major) {
      errors.push_back(std::string("Major version unsupported or invalid: ") + std::to_string(config_version.major));;
      return {};
    }
    // API supporting this version or above -- parse without error (with strict adherence to allowed/required fields for version)
    if (config_version.major < ::MyVersion.major || (config_version.major == ::MyVersion.major && config_version.minor <= ::MyVersion.minor)) {
      strict = true;
    } else {
      // API supporting major version, but not minor version; attempt to parse with warning that
      // it may not support all features and you should upgrade API if possible.  This is a
      // "non-strict" parsing that ignores unknown fields or subfields or unparseable field
      // values, as they may be dictated by a newer version.
      errors.push_back("API supports major but not minor config file version; attempting to parse anyway.  Note that all features may not be supported and you should upgrade the API if possible.");
      strict = false;
    }
  } else {
    errors.push_back("Required 'version' field not found in config file!");
    return {};
  }

  // "names" must exist and be a sequence of strings:
  if (root.has_child("names")) {
    auto names = config_file["names"];
    if (!names.is_seq()) {
      if (!names.is_keyval() || !names.val().has_str()) {
        errors.push_back("'names' is not a sequence or a string value.");
        if (strict)
          return {};
      } else {
        config_result->names_.push_back(std::string(names.val().str, names.val().len));
      }
    } else {
      for (auto n : names) {
        if (!n.is_val() || !n.val().has_str()) {
          errors.push_back("'names' includes item that is not a string value.");
          if (strict)
            return {};
        } else {
          config_result->names_.push_back(std::string(n.val().str, n.val().len));
        }
      }
    }
  } else {
    errors.push_back("Does not include 'names' field.");
    if (strict)
      return {};
  }

  // "families" must exist and be a single value or sequence of strings:
  if (root.has_child("families")) {
    auto families = config_file["families"];
    if (!families.is_seq()) {
      if (!families.is_keyval() || !families.val().has_str()) {
        errors.push_back("'families' is not a sequence or a string value.");
        if (strict)
          return {};
      } else {
        config_result->families_.push_back(std::string(families.val().str, families.val().len));
      }
    } else {
      for (auto f : families) {
        if (!f.is_val() || !f.val().has_str()) {
          errors.push_back("'families' includes item that is not a string value.");
          return {};
        } else {
          config_result->families_.push_back(std::string(f.val().str, f.val().len));
        }
      }
    }
  } else {
    errors.push_back("Does not include 'families' field.");
    if (strict)
      return {};
  }

  // Returns absolute filepath, or appends to error and returns empty string if file does
  // not exist or is absolute
  auto check_file = [&errors, &parent_dir_absolute](const std::string& type, const ryml::NodeRef& relative_file_node) {
    // Check yaml node type:
    if (!relative_file_node.is_keyval() || !relative_file_node.val().has_str()) {
      errors.push_back("'" + type + "' exists but is not a string value.");
      return std::string();
    }
    // Ensure file is relative:
    std::string relative_filename(relative_file_node.val().str, relative_file_node.val().len);
    if (util::file::File(relative_filename).isAbsolute()) {
      errors.push_back("'" + type + "' exists but provides an absolute path.");
      return std::string();
    }
    // Ensure file exists
    util::file::File absolute_filename(parent_dir_absolute);
    absolute_filename.append(relative_filename);
    if (!absolute_filename.exists()) {
      errors.push_back("'" + type + "' exists but does not point to valid file.");
      return std::string();
    }
    // Return success!
    return absolute_filename.getAbsolutePath();
  };

  // hrdf: if field exists, it must resolve to an existing file relative to the config file.
  // store absolute file path here
  if (root.has_child("hrdf")) {
    config_result->hrdf_ = check_file("hrdf", config_file["hrdf"]);
    if (config_result->hrdf_.empty())
      errors.push_back("HRDF element parsing resulting in empty field");
  }
  // gains: if field exists, all paths must resolve to an existing file relative to the config file.
  // store absolute file paths here
  if (root.has_child("gains")) {
    auto gains = config_file["gains"];
    // First, check single value:
    if (gains.is_keyval()) {
      auto default_gains = check_file("gains", config_file["gains"]);
      if (!default_gains.empty())
        config_result->gains_["default"] = default_gains;
    } else if (gains.is_map()) {
      for (auto g : gains) {
        auto new_gains = check_file("gains", g);
        if (!new_gains.empty()) {
          std::string key(g.key().str, g.key().len);
          config_result->gains_[key] = new_gains;
        }
      }
    } else {
      errors.push_back("gains is not a string or a map of string values");
    }
  }

  // Plugins
  if (root.has_child("plugins")) {
    auto plugins = config_file["plugins"];
    // Plugins can have
    if (!plugins.is_seq()) {
      errors.push_back("'plugins' not a sequence");
    } else {
      // Each plugin can have a series of dictionaries; each
      // dictionary must have a `name` and `type`.  Plugin
      // parameters are not validated here, but in plugin
      // constructors instead.
      for (auto plugin : plugins) {
        if (!plugin.is_map()) {
          errors.push_back("individual plugin not a map");
          continue;
        }
        if (!plugin.has_child("name") || !plugin.has_child("type")) {
          errors.push_back("plugin missing 'name' or 'type' field");
          continue;
        }
        auto plugin_name = plugin["name"];
        if (!plugin_name.is_keyval() || !plugin_name.val().has_str()) {
          errors.push_back("plugin 'name' field not a string");
          continue;
        }
        auto plugin_type = plugin["type"];
        if (!plugin_type.is_keyval() || !plugin_type.val().has_str()) {
          errors.push_back("plugin 'type' field not a string");
          continue;
        }
        experimental::arm::PluginConfig plugin_config(std::string(plugin_type.val().str, plugin_type.val().len),
                                                      std::string(plugin_name.val().str, plugin_name.val().len));
        for (auto param : plugin) {
          if (param.key_is_null()) {
            errors.push_back("Invalid plugin parameter - no key found!");
            continue;
          }
          std::string key(param.key().str, param.key().len);
          // Already handled above...
          if (key == "name" || key == "type")
            continue;
          if (param.is_keyval()) // string or int values
          {
            if (param.val().is_number()) {
              float v;
              param >> v;
              plugin_config.floats_[key] = v;
              continue;
            } else if (param.val().has_str()) {
              std::string value(param.val().str, param.val().len);
              if (value == "true")
                plugin_config.bools_[key] = true;
              else if (value == "false")
                plugin_config.bools_[key] = false;
              else
                plugin_config.strings_[key] = value;
              continue;
            } else {
              errors.push_back("bad value for plugin parameter!");
              continue;
            }
          } else if (param.is_seq()) // Sequence could be anything, determined by first value.
          {
            bool has_bool{};
            std::vector<bool> bool_values;
            bool has_float{};
            std::vector<float> float_values;
            bool has_string{};
            std::vector<std::string> string_values;

            auto parse_as_bool = [](c4::yml::NodeRef p, bool& v) {
              if (p.is_val() && p.val().has_str()) {
                std::string v_tmp;
                p >> v_tmp;
                if (v_tmp == "true")
                  v = true;
                else if (v_tmp == "false")
                  v = false;
                else
                  return false;
                return true;
              }
              return false;
            };
            auto parse_as_float = [](c4::yml::NodeRef p, float& v) {
              if (p.is_val() && p.val().is_number()) {
                p >> v;
                return true;
              }
              return false;
            };
            auto parse_as_string = [](c4::yml::NodeRef p, std::string& v) {
              if (p.is_val() && p.val().has_str()) {
                p >> v;
                return true;
              }
              return false;
            };

            bool tmp_bool{};
            float tmp_float{};
            std::string tmp_string{};
            for (auto p : param) {
              if (has_bool) {
                if (parse_as_bool(p, tmp_bool))
                  bool_values.push_back(tmp_bool);
                else
                  errors.push_back("Invalid boolean value in plugin param list.");
              } else if (has_float) {
                if (parse_as_float(p, tmp_float))
                  float_values.push_back(tmp_float);
                else
                  errors.push_back("Invalid float value in plugin param list.");
              } else if (has_string) {
                if (parse_as_string(p, tmp_string))
                  string_values.push_back(tmp_string);
                else
                  errors.push_back("Invalid string value in plugin param list.");
              } else // Nothing yet...
              {
                if (parse_as_bool(p, tmp_bool)) {
                  has_bool = true;
                  bool_values.push_back(tmp_bool);
                } else if (parse_as_float(p, tmp_float)) {
                  has_float = true;
                  float_values.push_back(tmp_float);
                } else if (parse_as_string(p, tmp_string)) {
                  has_string = true;
                  string_values.push_back(tmp_string);
                } else {
                  errors.push_back("Invalid value in plugin param list.");
                }
              }
            }
            // After all parameters iterated through, save into config
            if (has_bool)
              plugin_config.bool_lists_[key] = bool_values;
            else if (has_float)
              plugin_config.float_lists_[key] = float_values;
            else if (has_string)
              plugin_config.string_lists_[key] = string_values;
            else
              errors.push_back("No valid values found in plugin param list.");
            continue;
          }
          errors.push_back("Invalid plugin parameter");
        }
        config_result->plugin_configs_.push_back(plugin_config);
      }
    }
  }

  if (root.has_child("user_data")) {
    auto user_data = config_file["user_data"];
    // First, check single value:
    if (user_data.is_map()) {
      for (auto ud : user_data) {
        if (ud.is_keyval()) {
          // Note -- we currently just load all values as strings.
          std::string key(ud.key().str, ud.key().len);
          std::string value(ud.val().str, ud.val().len);
          config_result->user_data_[key] = value;
        } else if (ud.is_seq()) {
          errors.push_back("List not yet supported in user_data");
        } else {
          errors.push_back("Unparseable data in user_data map");
        }
      }
    } else {
      errors.push_back("user_data is not a map of values");
      if (strict)
        return {};
    }
  }

  for (auto elem : root) {
    const std::set<std::string> known_keys{"version", "names", "families", "hrdf", "gains", "plugins", "user_data"};
    std::string key(elem.key().str, elem.key().len);
    // Key must be one of known types:
    if (known_keys.count(key) == 0) {
      errors.push_back("root element has unknown key: " + key);
      if (strict)
        return {};
    }
  }

  // Note -- waypoints, paths, etc in future versions.

  return config_result;
}

} // namespace hebi
