#pragma once

#include <seahowl/commons/numerics.h>
#include <seahowl/io/store_db_models.h>

namespace seahowl {
namespace io {
/**
 * @brief Read tower data from a file.
 * @param filepath Path to the tower data file.
 */
TowerDb read_tower_db(const std::string& filepath);

/**
 * @brief Read blade data from a file.
 * @param filepath Path to the blade data file.
 */
BladeDb read_blade_db(const std::string& filepath);

/**
 * @brief Read RNA data from a file.
 * @param filepath Path to the RNA data file.
 */
RnaDb read_rna_db(const std::string& filepath);

/**
 * @brief Read environment data from a file.
 * @param filepath Path to the environment data file.
 */
EnvironmentDb read_environment_db(const std::string& filepath);

/**
 * @brief Read turbine data from a file.
 * @param filepath Path to the wind data file.
 */
TurbineDb read_turbine_db(const std::string& filepath);

}  // namespace io
}  // namespace seahowl
