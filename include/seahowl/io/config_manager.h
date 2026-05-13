// SPDX-License-Identifier: Apache-2.0
// Copyright 2022–2026 TotalEnergies
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#pragma once

// Standard library
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace seahowl {
namespace io {
namespace app {

// ConfigManager
/**
 * @brief Specification of a variable in the configuration manager.
 */
struct ConfigManagerVariableSpec {
    std::string name;
    std::vector<ConfigManagerVariableSpec> children = {};
    std::string description = "";
    std::string type = "";
    std::string defaultValue = "";
    bool has_env_var = false;
    bool has_config_file_var = false;
    bool has_option_var = false;
};

/**
 * @brief Options for the configuration manager.
 */
struct ConfigManagerOptions {
    std::string env_var_prefix = "";
    std::string ini_filepath = "";
    std::string json_filepath = "";
    ConfigManagerVariableSpec variable_specs = {};
};

/**
 * @brief Specification of a computed variable in the configuration manager.
 */
struct SpecComputed {
    ConfigManagerVariableSpec spec;
    std::string description = "";
    std::string env_var = "";
    std::string config_file_var = "";
    std::string command_option_var = "";
};

/**
 * @brief Value of a computed variable in the configuration manager.
 */
struct ValueComputed {
    std::string origin = "";
    std::string value;
    int int_value;
    double double_value;
    bool bool_value;
};

class ConfigManagerPath;  // Forward declaration

class ConfigManagerImpl;  // Forward declaration PIMPL

/**
 * @brief Configuration manager class.
 */
class ConfigManager {
  public:
    ConfigManager();
    explicit ConfigManager(const ConfigManagerOptions& options);
    ~ConfigManager();

    // Enable move semantics (defined in source file due to PIMPL)
    ConfigManager(ConfigManager&&) noexcept;
    ConfigManager& operator=(ConfigManager&&) noexcept;

    // Disable copy
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    /**
     * @brief Compute the configuration manager.
     * @param argc Number of arguments.
     * @param argv Array of arguments.
     */
    void compute(int argc, char** argv);
    /**
     * @brief Print the specification of the configuration manager.
     */
    void compute();
    /**
     * @brief Print the specification of the configuration manager.
     */
    void print_spec() const;
    /**
     * @brief Print the computed values of the configuration manager.
     */
    void print_compute() const;
    /**
     * @brief Get the value of a key.
     * @param key Key to get the value of.
     */
    std::string get(const std::string& key) const;
    /**
     * @brief Get the string value of a key.
     * @param key Key to get the value of.
     */
    std::string get_string(const std::string& key) const;
    /**
     * @brief Get the integer value of a key.
     * @param key Key to get the value of.
     */
    int get_int(const std::string& key) const;
    /**
     * @brief Get the double value of a key.
     * @param key Key to get the value of.
     */
    double get_double(const std::string& key) const;
    /**
     * @brief Get the boolean value of a key.
     * @param key Key to get the value of.
     */
    bool get_bool(const std::string& key) const;
    /**
     * @brief Check if a key exists.
     * @param key Key to check.
     */
    bool has(const std::string& key) const;
    /**
     * @brief Get the JSON file path.
     */
    std::string get_json_filepath() const;
    /**
     * @brief Set the JSON file path.
     * @param newPath New path to set.
     */
    void set_json_filepath(const std::string& newPath);
    /**
     * @brief Get the JSON data.
     */
    ConfigManagerPath path(std::string prefix);

  private:
    std::unique_ptr<ConfigManagerImpl> pimpl_;  // Pattern PIMPL
};

/**
 * @brief Path class for convenience of accessing nested values.
 */
class ConfigManagerPath {
  private:
    ConfigManager* conf_;
    std::string prefix_;

  public:
    ConfigManagerPath(ConfigManager& conf, std::string prefix) : conf_(&conf), prefix_(prefix) {}

    std::string get(const std::string& key) const { return conf_->get(prefix_ + "." + key); };
    std::string get_string(const std::string& key) const { return conf_->get_string(prefix_ + "." + key); };
    int get_int(const std::string& key) const { return conf_->get_int(prefix_ + "." + key); };
    double get_double(const std::string& key) const { return conf_->get_double(prefix_ + "." + key); };
    bool get_bool(const std::string& key) const { return conf_->get_bool(prefix_ + "." + key); };
    bool has(const std::string& key) const { return conf_->has(prefix_ + "." + key); };
};

}  // namespace app
}  // namespace io
}  // namespace seahowl
