#pragma once

#include <fstream>
#include <iostream>
#include <iomanip>
#include <cctype>
#include <algorithm>

namespace utils {

std::string toUpper(const std::string& input) {
    std::string result = input;
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return std::toupper(c); });
    return result;
}

std::string toLower(const std::string& input) {
    std::string result = input;
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return std::tolower(c); });
    return result;
}

std::string replaceAll(std::string str, char from, char to) {
    for (size_t i = 0; i < str.length(); ++i) {
        if (str[i] == from) {
            str[i] = to;
        }
    }
    return str;
}

std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t");
    if (first == std::string::npos)
        return "";
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

std::string camelToKebab(const std::string& camelCase) {
    std::string kebabCase;
    for (char ch : camelCase) {
        if (std::isupper(ch)) {
            if (!kebabCase.empty()) {
                kebabCase += '-';
            }
            kebabCase += std::tolower(ch);
        } else {
            kebabCase += ch;
        }
    }
    return kebabCase;
};

std::string camelToUpper(const std::string& camelCase) {
    std::string upperCase;
    for (char ch : camelCase) {
        if (std::isupper(ch)) {
            upperCase += '_';
        }
        upperCase += std::toupper(ch);
    }
    return upperCase;
};

// cout print table
void printTable(const std::vector<std::string>& headers,
                const std::vector<std::vector<std::string>>& rows,
                int columnWidth) {
    // Separator line
    std::cout << std::string((columnWidth + 1) * headers.size(), '-') << std::endl;
    // Headers
    for (const auto& header : headers) {
        if (header.length() > columnWidth) {
            std::cout << std::setw(columnWidth) << std::left << header.substr(0, columnWidth - 1) + "."
                      << " ";
        } else {
            std::cout << std::setw(columnWidth) << std::left << header << " ";
        }
    }
    std::cout << std::endl;
    // Separator line
    std::cout << std::string((columnWidth + 1) * headers.size(), '-') << std::endl;
    // rows
    for (const auto& row : rows) {
        for (const auto& item : row) {
            if (item.length() > columnWidth) {
                std::cout << std::setw(columnWidth) << std::left << item.substr(0, columnWidth - 1) + "."
                          << " ";
            } else {
                std::cout << std::setw(columnWidth) << std::left << item << " ";
            }
        }
        std::cout << std::endl;
    }
    // Separator line
    std::cout << std::string((columnWidth + 1) * headers.size(), '-') << std::endl;
}

// Fetures
// - Handles both flags and key-value pairs
// - Supports values with spaces if quoted in command line
// - Uses string_view for efficient string operations
// TODO : ??
// - Short options (single dash)
// - Help text generation
std::map<std::string, std::string> parseArgs(int argc, char* argv[]) {
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
