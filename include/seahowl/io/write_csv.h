#include <string>

#include <seahowl/core/system.h>

/**
 * @brief Writes general CSV info to file.
 *
 * @param[in] filename Path of output file.
 * @param[in] ssystem System to output.
 * @param[in] time Time of simulation.
 */
void write_turbine_info_to_csv(std::string filename, const seahowl::core::System& ssystem, double time);
