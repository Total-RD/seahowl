#include "seahowl/io/utils_io.h"

#include <filesystem>
#include <string>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <vector>
#include <map>
#include <spdlog/spdlog.h>
#include <iomanip>

namespace fs = std::filesystem;

namespace seahowl {
namespace io {
namespace utils {

void check_file_exists(const std::string& filepath) {
    if (!fs::exists(filepath)) {
        throw std::runtime_error("File \"" + filepath + "\" does not exist.");
    }
}

std::string copy_file_and_increment(const std::string& filepath, const std::string& destination_dir) {
    if (!destination_dir.empty()) {
        fs::create_directories(destination_dir);
    }
    fs::path pfilepath = fs::path(filepath);
    fs::path filecopypath;
    if (fs::exists(destination_dir / pfilepath.filename())) {
        auto filename = pfilepath.stem().generic_string();
        auto fileext = pfilepath.extension().generic_string();
        bool copied = false;
        int file_idx = 0;
        while (!copied) {
            filecopypath = fs::path(destination_dir) / (filename + std::to_string(file_idx) + fileext);
            if (!fs::exists(filecopypath)) {
                fs::copy(pfilepath, filecopypath);
                pfilepath = filecopypath;
                copied = true;
            } else {
                file_idx += 1;
            }
        }
    } else {
        filecopypath = destination_dir / pfilepath.filename();
        fs::copy(pfilepath, filecopypath);
    }
    return filecopypath.generic_string();
}

std::string to_upper(const std::string& input) {
    std::string result = input;
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return std::toupper(c); });
    return result;
}

std::string to_lower(const std::string& input) {
    std::string result = input;
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return std::tolower(c); });
    return result;
}

std::string replace_all(std::string input, char from, char to) {
    for (size_t i = 0; i < input.length(); ++i) {
        if (input[i] == from) {
            input[i] = to;
        }
    }
    return input;
}

std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t");
    if (first == std::string::npos)
        return "";
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

std::string camel_to_kebab(const std::string& camel_case) {
    std::string kebab_case;
    for (char ch : camel_case) {
        if (std::isupper(ch)) {
            if (!kebab_case.empty()) {
                kebab_case += '-';
            }
            kebab_case += std::tolower(ch);
        } else {
            kebab_case += ch;
        }
    }
    return kebab_case;
};

std::string camel_to_upper(const std::string& camel_case) {
    std::string upper_case;
    for (char ch : camel_case) {
        if (std::isupper(ch)) {
            upper_case += '_';
        }
        upper_case += std::toupper(ch);
    }
    return upper_case;
};

void print_table(const std::vector<std::string>& headers,
                 const std::vector<std::vector<std::string>>& rows,
                 int column_width) {
    // Separator line

    std::stringstream output_sstring;
    output_sstring << std::string((column_width)*headers.size(), '-');
    spdlog::info(output_sstring.str());
    output_sstring.str("");
    // Headers
    for (const auto& header : headers) {
        if (header.length() > column_width) {
            output_sstring << std::setw(column_width) << std::left << header.substr(0, column_width - 1) + "."
                           << " ";
        } else {
            output_sstring << std::setw(column_width) << std::left << header << " ";
        }
    }
    spdlog::info(output_sstring.str());
    output_sstring.str("");
    // Separator line
    output_sstring << std::string((column_width)*headers.size(), '-');
    spdlog::info(output_sstring.str());
    output_sstring.str("");
    // rows
    for (const auto& row : rows) {
        for (const auto& item : row) {
            if (item.length() > column_width) {
                output_sstring << std::setw(column_width) << std::left << item.substr(0, column_width - 1) + "."
                               << " ";
            } else {
                output_sstring << std::setw(column_width) << std::left << item << " ";
            }
        }
        spdlog::info(output_sstring.str());
        output_sstring.str("");
    }
    // Separator line
    output_sstring << std::string((column_width)*headers.size(), '-');
    spdlog::info(output_sstring.str());
    output_sstring.str("");
}

std::map<std::string, std::string> parse_args(int argc, char* argv[]) {
    std::map<std::string, std::string> options;

    for (int i = 1; i < argc; i++) {
        std::string_view arg(argv[i]);

        // Check if argument starts with "--"
        if (arg.substr(0, 2) == "--") {
            std::string key = std::string(arg.substr(2));
            std::string value;

            // Check if there's a value in the next argument
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                value = argv[i + 1];
                i++;  // Skip the value in next iteration
            }

            options[key] = value;
        }
    }

    return options;
}

}  // namespace utils
}  // namespace io
}  // namespace seahowl
