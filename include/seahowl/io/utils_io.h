#pragma once

#include <string>

/**@brief Checks if file exists, throw error otherwise.
 */
void check_file_exists(const std::string& filepath);

/**@brief Copy file to destination dir, increment file name if already exists, and return path to new copid file.
 */
std::string copy_file_and_increment(const std::string& filepath, const std::string& destination_dir);
