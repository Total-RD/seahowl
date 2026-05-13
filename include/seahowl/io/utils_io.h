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
#include <string>
#include <vector>

namespace seahowl {
namespace io {
namespace utils {

/**
 * @brief Checks if file exists, throw error otherwise.
 *
 * @param filepath Path to file.
 */
void check_file_exists(const std::string& filepath);

/**
 * @brief Copy file to destination dir, increment file name if already exists, and return path to new copied file.
 *
 * @param filepath Path to original file.
 * @param destination_dir Path to directory for copying file.
 */
std::string copy_file_and_increment(const std::string& filepath, const std::string& destination_dir);

/**
 * @brief Transforms string to upper case.
 *
 * @param input String to transform.
 */
std::string to_upper(const std::string& input);

/**
 * @brief Transforms string to lower case.
 *
 * @param input String to transform.
 */
std::string to_lower(const std::string& input);

/**
 * @brief Replaces in string.
 *
 * @param input String to transform.
 * @param from Character to replace.
 * @param to Replacement character.
 */
std::string replace_all(std::string input, char from, char to);

/**
 * @brief Trims string.
 *
 * @param input String to transform.
 */
std::string trim(const std::string& str);

/**
 * @brief Transforms string from camel to kebab case.
 *
 * @param camel_case String to transform.
 */
std::string camel_to_kebab(const std::string& camel_case);

/**
 * @brief Transforms string from camel to upper case.
 *
 * @param camel_case String to transform.
 */
std::string camel_to_upper(const std::string& camel_case);

/**
 * @brief Prints table.
 *
 * @param headers Headers to print.
 * @param rows Rows to print.
 * @param column_width Width of columns.
 */
void print_table(const std::vector<std::string>& headers,
                 const std::vector<std::vector<std::string>>& rows,
                 int column_width);

/**
 * @brief Parse args.
 *
 * - Handles both flags and key-value pairs
 *- Supports values with spaces if quoted in command line
 * - Uses string_view for efficient string operations
 *
 * @param argc Number of arguments.
 * @param argv Arguments.
 */
// TODO : ??
// - Short options (single dash)
// - Help text generation
std::map<std::string, std::string> parse_args(int argc, char* argv[]);

}  // namespace utils
}  // namespace io
}  // namespace seahowl
