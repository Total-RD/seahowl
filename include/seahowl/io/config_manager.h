#pragma once

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <nlohmann/json.hpp>
// #include <optional>

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
    bool hasEnvVar = false;
    bool hasConfigFileVar = false;
    bool hasOptionVar = false;
};

/**
 * @brief Options for the configuration manager.
 */
struct ConfigManagerOptions {
    std::string envVarPrefix = "";
    std::string iniFilePath = "";
    std::string jsonFilePath = "";
    nlohmann::json jsonData = {};
    ConfigManagerVariableSpec variableSpecs = {};
};

/**
 * @brief Specification of a computed variable in the configuration manager.
 */
struct SpecComputed {
    ConfigManagerVariableSpec spec;
    std::string description = "";
    std::string envVar = "";
    std::string configFileVar = "";
    std::string commandOptionVar = "";
};

/**
 * @brief Value of a computed variable in the configuration manager.
 */
struct ValueComputed {
    std::string origin = "";
    std::string value;
    int intValue;
    double doubleValue;
    bool boolValue;
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
    void printSpec() const;
    /**
     * @brief Print the computed values of the configuration manager.
     */
    void printCompute() const;
    /**
     * @brief Get the value of a key.
     * @param key Key to get the value of.
     */
    std::string get(const std::string& key) const;
    /**
     * @brief Get the string value of a key.
     * @param key Key to get the value of.
     */
    std::string getString(const std::string& key) const;
    /**
     * @brief Get the integer value of a key.
     * @param key Key to get the value of.
     */
    int getInt(const std::string& key) const;
    /**
     * @brief Get the double value of a key.
     * @param key Key to get the value of.
     */
    double getDouble(const std::string& key) const;
    /**
     * @brief Get the boolean value of a key.
     * @param key Key to get the value of.
     */
    bool getBool(const std::string& key) const;
    /**
     * @brief Check if a key exists.
     * @param key Key to check.
     */
    bool has(const std::string& key) const;
    /**
     * @brief Get the JSON file path.
     */
    std::string getJsonFilePath() const;
    /**
     * @brief Set the JSON file path.
     * @param newPath New path to set.
     */
    void setJsonFilePath(const std::string& newPath);
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
    std::string getString(const std::string& key) const { return conf_->getString(prefix_ + "." + key); };
    int getInt(const std::string& key) const { return conf_->getInt(prefix_ + "." + key); };
    double getDouble(const std::string& key) const { return conf_->getDouble(prefix_ + "." + key); };
    bool getBool(const std::string& key) const { return conf_->getBool(prefix_ + "." + key); };
    bool has(const std::string& key) const { return conf_->has(prefix_ + "." + key); };
};

}  // namespace app
