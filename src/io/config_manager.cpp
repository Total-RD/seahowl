#include "seahowl/io/config_manager.h"
#include "seahowl/io/utils_io.h"
#include "seahowl/io/command_parser.h"

#include <vector>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

namespace seahowl {
namespace io {
namespace app {

class ConfigManagerImpl {
  public:
    ConfigManagerOptions options_;
    std::map<std::string, SpecComputed> varspecs_;
    std::map<std::string, ValueComputed> vals_;

    ConfigManagerImpl(const ConfigManagerOptions& options) {
        options_ = options;
        linearize(options_.variable_specs, "_");
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
                (var.has_env_var ? get_var_env_name(prefix) : ""),
                (var.has_config_file_var ? get_var_config_filename(prefix) : ""),
                (var.has_option_var ? get_var_command_option_name(prefix) : ""),
            };
        } else {
            for (const auto& child : var.children) {
                linearize(child, prefix);
            }
        }
    }
    std::string get_var_env_name(const std::string& varname) {
        // return options_.envVarPrefix + utils::camelToUpper(utils::replaceAll(varname, '.', '_'));
        return options_.env_var_prefix + utils::replace_all(utils::to_upper(varname), '.', '_');
    }

    std::string get_var_config_filename(const std::string& varname) {
        // return utils::camelToUpper(utils::replaceAll(varname, '.', '_'));
        return varname;
    }

    std::string get_var_command_option_name(const std::string& varname) {
        // return utils::camelToKebab(utils::replaceAll(utils::replaceAll(varname, '.', '-'), '_', '-'));
        return utils::replace_all(utils::replace_all(utils::to_lower(varname), '.', '-'), '_', '-');
    }

    void set_value_from_string(const std::string& key, const std::string& origin, const std::string& value) {
        vals_[key].origin = origin;
        std::string type = varspecs_[key].spec.type;
        if (type == "int") {
            vals_[key].int_value = std::stoi(value);
        } else if (type == "double") {
            vals_[key].double_value = std::stod(value);
        } else if (type == "bool") {
            vals_[key].bool_value = (value == "true");
        } else {
            vals_[key].value = value;
        }
    }
    std::string get_value_from_string(const std::string& key) {
        std::string type = varspecs_[key].spec.type;
        if (type == "int") {
            return std::to_string(vals_[key].int_value);
        } else if (type == "double") {
            return std::to_string(vals_[key].double_value);
        } else if (type == "bool") {
            return vals_[key].bool_value ? "true" : "false";
        } else {
            return vals_[key].value;
        }
    }

    void load_defaults() {
        for (const auto& [key, var] : varspecs_) {
            // vals_[key] = {"default", var.spec.defaultValue};
            set_value_from_string(key, "default", var.spec.defaultValue);
        }
    }

    void load_environment_variables() {
        for (const auto& [key, var] : varspecs_) {
            if (var.spec.has_env_var && !var.env_var.empty()) {
                const char* value = std::getenv(var.env_var.c_str());
                if (value != nullptr) {
                    set_value_from_string(key, "env_var", value);
                }
            }
        }
    }

    void load_config_file() {
        nlohmann::json jsonData;

        // read file

        // std::cout << "read file" << std::endl;
        // read file
        std::ifstream ifile(options_.json_filepath);
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
                if (var.spec.has_config_file_var && !var.config_file_var.empty()) {
                    std::string path = "/" + utils::replace_all(var.config_file_var, '.', '/');
                    std::string type = varspecs_[key].spec.type;
                    try {
                        auto value = jsonData.at(nlohmann::json::json_pointer(path));
                        vals_[key].origin = "config_file";
                        if (type == "int") {
                            vals_[key].int_value = value.get<int>();
                        } else if (type == "double") {
                            vals_[key].double_value = value.get<double>();
                        } else if (type == "bool") {
                            vals_[key].bool_value = value.get<bool>();
                        } else {
                            vals_[key].value = value.get<std::string>();
                            if (type == "path") {
                                auto parent_path = fs::path(options_.json_filepath).parent_path();
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

    void load_command_options(int argc, char** argv) {
        auto cmdOptions = get_keys_and_types();
        auto options = CommandLineParser::parse_args(argc, argv, cmdOptions);
        for (const auto& [key, var] : varspecs_) {
            if (var.spec.has_option_var && !var.command_option_var.empty()) {
                auto it = options.find(var.command_option_var);
                if (it != options.end()) {
                    set_value_from_string(key, "cmd_option", it->second);
                }
            }
        }
    }

    std::map<std::string, SpecComputed> get_keys_and_types() const {
        std::map<std::string, SpecComputed> keysAndTypes;
        for (const auto& [key, var] : varspecs_) {
            if (!var.command_option_var.empty()) {
                keysAndTypes[var.command_option_var] = var;
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
    pimpl_->load_defaults();
    pimpl_->load_environment_variables();
    pimpl_->load_config_file();
    pimpl_->load_command_options(argc, argv);
}

void ConfigManager::compute() {
    pimpl_->load_defaults();
    pimpl_->load_environment_variables();
    pimpl_->load_config_file();
}

std::string ConfigManager::get(const std::string& key) const {
    return pimpl_->get_value_from_string(key);
}
std::string ConfigManager::get_string(const std::string& key) const {
    return pimpl_->vals_[key].value;
}
int ConfigManager::get_int(const std::string& key) const {
    return pimpl_->vals_[key].int_value;
}
double ConfigManager::get_double(const std::string& key) const {
    return pimpl_->vals_[key].double_value;
}

bool ConfigManager::get_bool(const std::string& key) const {
    return pimpl_->vals_[key].bool_value;
}

bool ConfigManager::has(const std::string& key) const {
    return pimpl_->vals_.find(key) != pimpl_->vals_.end();
}

void ConfigManager::print_spec() const {
    std::cout << std::endl;
    std::cout << "ConfigManager : Configuration Specifications" << std::endl;
    std::vector<std::vector<std::string>> table = {};
    for (const auto& [key, var] : pimpl_->varspecs_) {
        table.push_back({key, var.description, var.spec.type, var.spec.defaultValue, var.env_var, var.config_file_var,
                         var.command_option_var});
    }
    utils::print_table(
        {"Key", "Description", "Type", "0 - Default", "1 - Env Var", "2 - Config File Var", "3 - Cmd Option"}, table,
        25);
}

void ConfigManager::print_compute() const {
    std::cout << std::endl;
    std::cout << "ConfigManager : Configuration Data" << std::endl;
    std::vector<std::vector<std::string>> table = {};
    for (const auto& [key, var] : pimpl_->vals_) {
        table.push_back({key, pimpl_->get_value_from_string(key), var.origin});
    }
    utils::print_table({"Variable", "Value", "Origin"}, table, 25);
}

// Méthode pour accéder à jsonFilePath
std::string ConfigManager::get_json_filepath() const {
    return pimpl_->options_.json_filepath;
}

// Méthode pour définir jsonFilePath
void ConfigManager::set_json_filepath(const std::string& newPath) {
    pimpl_->options_.json_filepath = newPath;
}

ConfigManagerPath ConfigManager::path(std::string prefix) {
    return ConfigManagerPath(*this, prefix);
}

// ConfigManagerPath

}  // namespace app
}  // namespace io
}  // namespace seahowl
