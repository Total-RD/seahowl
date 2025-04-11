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

}  // namespace io
}  // namespace seahowl
