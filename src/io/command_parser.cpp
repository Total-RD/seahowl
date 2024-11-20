#include "seahowl/io/command_parser.h"
#include <iostream>
#include <string_view>
#include <stdexcept>
#include <filesystem>

namespace fs = std::filesystem;

void CommandLineParser::printHelper(const std::map<std::string, app::SpecComputed>& cmdOptions) {
    std::cout << "Usage: file_input [options]\n";
    std::cout << "Options:\n";
    std::cout << "  -h        Display this help message\n";
    for (const auto& [key, var] : cmdOptions) {
        auto type = var.spec.type;
        if ( type == "path") {
            type = "string";
        }
        std::cout << "  --" << key << " <value> ";
        std::cout << var.spec.description << " (" << type << ")\n";
    }
}

std::map<std::string, std::string>
CommandLineParser::parseArgs(int argc, char* argv[], const std::map<std::string, app::SpecComputed>& cmdOptions) {
    std::map<std::string, std::string> options;

    for (int i = 1; i < argc; i++) {
        std::string_view arg(argv[i]);

        // Check if the argument is "-h"
        if (arg == "-h") {
            printHelper(cmdOptions);
            std::exit(EXIT_SUCCESS);
        }

        // Check if the argument starts with "--"
        if (arg.substr(0, 2) == "--") {
            std::string key = std::string(arg.substr(2));
            std::string value;

            // Check if the key is valid
            auto it = cmdOptions.find(key);
            if (it == cmdOptions.end()) {
                std::cerr << "Error: Invalid option '" << key << "'\n";
                std::exit(EXIT_FAILURE);
            }

            // Check if there is a value in the next argument
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                value = argv[i + 1];
                i++;  // Skip the value in the next iteration

                // Check if the value matches the expected type
                if (!isValidType(value, it->second.spec.type)) {
                    std::cerr << "Error: Invalid value type for option '" << key
                              << "'. Expected type: " << it->second.spec.type << "\n";
                    std::exit(EXIT_FAILURE);
                }
                if (it->second.spec.type == "path") {
                    value = fs::current_path().append(value).string();
                }
            } else {
                std::cerr << "Error: Missing value for option '" << key << "'\n";
                std::exit(EXIT_FAILURE);
            }

            options[key] = value;
        }
    }

    return options;
}

bool CommandLineParser::isValidType(const std::string& value, const std::string& type) {
    try {
        if (type == "int") {
            std::stoi(value);
        } else if (type == "double") {
            std::stod(value);
        } else if (type == "bool") {
            if (value != "true" && value != "false") {
                return false;
            }
        } else if (type == "string" || type == "path") {
            // No validation needed for strings
        } else {
            return false;
        }
    } catch (const std::invalid_argument& e) {
        return false;
    } catch (const std::out_of_range& e) {
        return false;
    }

    return true;
}
