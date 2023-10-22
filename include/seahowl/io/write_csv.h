#pragma once

#include <string>

// forward declarations
namespace seahowl {
namespace core {
class System;
}  // namespace core
}  // namespace seahowl

/**
 * @brief Writes general CSV info to file.
 *
 * @param[in] filename Path of output file.
 * @param[in] ssystem System to output.
 * @param[in] time Time of simulation.
 */
void write_turbine_info_to_csv(std::string fileprefix, const seahowl::core::System& ssystem);
