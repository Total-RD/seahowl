#include "seahowl/io/config_manager.h"
#include "seahowl/io/utils_config.h"
#include "seahowl/io/command_parser.h"

#include <vector>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <nlohmann/json.hpp>


namespace fs = std::filesystem;

namespace app {

class ConfigManagerImpl {
  public:
    ConfigManagerOptions options_;
    std::map<std::string, SpecComputed> varspecs_;
    std::map<std::string, ValueComputed> vals_;

    ConfigManagerImpl(const ConfigManagerOptions& options) {
        options_ = options;
        linearize(options_.variableSpecs, "_");
    }

    void linearize(const ConfigManagerVariableSpec& var, std::string prefix) {
        if (prefix.empty()) {
            prefix = var.name;  // utils::toLower(var.name);
        } else if (prefix == "_") {
            prefix = "";
        } else {
            prefix += "." + var.name;  // utils::toLower(var.name);
        }
        if (var.children.empty()) {
            varspecs_[prefix] = {
                var,
                var.description,
                (var.hasEnvVar ? getVarEnvName(prefix) : ""),
                (var.hasConfigFileVar ? getVarConfigFileName(prefix) : ""),
                (var.hasOptionVar ? getVarCommandOptionName(prefix) : ""),
            };
        } else {
            for (const auto& child : var.children) {
                linearize(child, prefix);
            }
        }
    }
    std::string getVarEnvName(const std::string& varname) {
        // return options_.envVarPrefix + utils::camelToUpper(utils::replaceAll(varname, '.', '_'));
        return options_.envVarPrefix + utils::replaceAll(utils::toUpper(varname), '.', '_');
    }

    std::string getVarConfigFileName(const std::string& varname) {
        // return utils::camelToUpper(utils::replaceAll(varname, '.', '_'));
        return varname;
    }

    std::string getVarCommandOptionName(const std::string& varname) {
        // return utils::camelToKebab(utils::replaceAll(utils::replaceAll(varname, '.', '-'), '_', '-'));
        return utils::replaceAll(utils::replaceAll(utils::toLower(varname), '.', '-'), '_', '-');
    }

    void setValueFromString(const std::string& key, const std::string& origin, const std::string& value) {
        vals_[key].origin = origin;
        std::string type = varspecs_[key].spec.type;
        if (type == "int") {
            vals_[key].intValue = std::stoi(value);
        } else if (type == "double") {
            vals_[key].doubleValue = std::stod(value);
        } else if (type == "bool") {
            vals_[key].boolValue = (value == "true");
        } else {
            vals_[key].value = value;
        }
    }
    std::string getValueToString(const std::string& key) {
        std::string type = varspecs_[key].spec.type;
        if (type == "int") {
            return std::to_string(vals_[key].intValue);
        } else if (type == "double") {
            return std::to_string(vals_[key].doubleValue);
        } else if (type == "bool") {
            return vals_[key].boolValue ? "true" : "false";
        } else {
            return vals_[key].value;
        }
    }

    void loadDefaults() {
        for (const auto& [key, var] : varspecs_) {
            // vals_[key] = {"default", var.spec.defaultValue};
            setValueFromString(key, "default", var.spec.defaultValue);
        }
    }

    void loadEnvironmentVariables() {
        for (const auto& [key, var] : varspecs_) {
            if (var.spec.hasEnvVar && !var.envVar.empty()) {
                const char* value = std::getenv(var.envVar.c_str());
                if (value != nullptr) {
                    setValueFromString(key, "env_var", value);
                }
            }
        }
    }

    void loadConfigFile() {
        nlohmann::json jsonData;

        // read file

        // std::cout << "read file" << std::endl;
        // read file
        std::ifstream ifile(options_.jsonFilePath);
        if (!ifile.is_open()) {
            std::cout << "Failed to open config file\n";
            return;
        }
        ifile >> jsonData;
        ifile.close();
        // read JSON data string

        //
        if (!jsonData.empty()) {
            for (const auto& [key, var] : varspecs_) {
                if (var.spec.hasConfigFileVar && !var.configFileVar.empty()) {
                    std::string path = "/" + utils::replaceAll(var.configFileVar, '.', '/');
                    std::string type = varspecs_[key].spec.type;
                    try {
                        auto value = jsonData.at(nlohmann::json::json_pointer(path));
                        vals_[key].origin = "config_file";
                        if (type == "int") {
                            vals_[key].intValue = value.get<int>();
                        } else if (type == "double") {
                            vals_[key].doubleValue = value.get<double>();
                        } else if (type == "bool") {
                            vals_[key].boolValue = value.get<bool>();
                        } else {
                            vals_[key].value = value.get<std::string>();
                            if (type == "path"){
                                auto parent_path = fs::path(options_.jsonFilePath).parent_path();
                                vals_[key].value = parent_path.append(vals_[key].value).string();
                            }
                        }
                    } catch (nlohmann::json::out_of_range& e) {
                        std::cout << "ConfigManager : " << e.what() << std::endl;
                    }
                }
            }
        }
    }

    void loadCommandOptions(int argc, char** argv) {
        auto cmdOptions = getKeysAndTypes();
        auto options = CommandLineParser::parseArgs(argc, argv, cmdOptions);
        for (const auto& [key, var] : varspecs_) {
            if (var.spec.hasOptionVar && !var.commandOptionVar.empty()) {
                auto it = options.find(var.commandOptionVar);
                if (it != options.end()) {
                    setValueFromString(key, "cmd_option", it->second);
                }
            }
        }
    }

    std::map<std::string, SpecComputed> getKeysAndTypes() const {
        std::map<std::string, SpecComputed> keysAndTypes;
        for (const auto& [key, var] : varspecs_) {
            if (!var.commandOptionVar.empty()) {
                keysAndTypes[var.commandOptionVar] = var;
            }
        }

        return keysAndTypes;
    }
};

ConfigManager::ConfigManager() : ConfigManager(ConfigManagerOptions{}) {}
ConfigManager::ConfigManager(const ConfigManagerOptions& options)
    : pimpl_(std::make_unique<ConfigManagerImpl>(options)) {}
ConfigManager::~ConfigManager() = default;

void ConfigManager::compute(int argc, char** argv) {
    pimpl_->loadDefaults();
    pimpl_->loadEnvironmentVariables();
    pimpl_->loadConfigFile();
    pimpl_->loadCommandOptions(argc, argv);
}

void ConfigManager::compute() {
    pimpl_->loadDefaults();
    pimpl_->loadEnvironmentVariables();
    pimpl_->loadConfigFile();
}

std::string ConfigManager::get(const std::string& key) const {
    return pimpl_->getValueToString(key);
}
std::string ConfigManager::getString(const std::string& key) const {
    return pimpl_->vals_[key].value;
}
int ConfigManager::getInt(const std::string& key) const {
    return pimpl_->vals_[key].intValue;
}
double ConfigManager::getDouble(const std::string& key) const {
    return pimpl_->vals_[key].doubleValue;
}

bool ConfigManager::getBool(const std::string& key) const {
    return pimpl_->vals_[key].boolValue;
}

bool ConfigManager::has(const std::string& key) const {
    return pimpl_->vals_.find(key) != pimpl_->vals_.end();
}

void ConfigManager::printSpec() const {
    std::cout << std::endl;
    std::cout << "ConfigManager : Configuration Specifications" << std::endl;
    std::vector<std::vector<std::string>> table = {};
    for (const auto& [key, var] : pimpl_->varspecs_) {
        table.push_back({key, var.description, var.spec.type, var.spec.defaultValue, var.envVar, var.configFileVar,
                         var.commandOptionVar});
    }
    utils::printTable(
        {"Key", "Description", "Type", "0 - Default", "1 - Env Var", "2 - Config File Var", "3 - Cmd Option"}, table,
        25);
}

void ConfigManager::printCompute() const {
    std::cout << std::endl;
    std::cout << "ConfigManager : Configuration Data" << std::endl;
    std::vector<std::vector<std::string>> table = {};
    for (const auto& [key, var] : pimpl_->vals_) {
        table.push_back({key, pimpl_->getValueToString(key), var.origin});
    }
    utils::printTable({"Variable", "Value", "Origin"}, table, 25);
}

// Méthode pour accéder à jsonFilePath
std::string ConfigManager::getJsonFilePath() const {
    return pimpl_->options_.jsonFilePath;
}

// Méthode pour définir jsonFilePath
void ConfigManager::setJsonFilePath(const std::string& newPath) {
    pimpl_->options_.jsonFilePath = newPath;
}

ConfigManagerPath ConfigManager::path(std::string prefix) {
    return ConfigManagerPath(*this, prefix);
}

// ConfigManagerPath

}  // namespace app
