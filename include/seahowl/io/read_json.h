#pragma once

namespace seahowl {
namespace core {
class Blade;
class Rotor;
class Tower;
class Turbine;
class System;
struct BladeReferencePoint;
struct TowerReferencePoint;
}  // namespace core
namespace elasto {
class SystemElasto;
class MeshElasto;
}  // namespace elasto
}  // namespace seahowl

#include <vector>
#include <string>
#include <memory>

/**
 * @brief Returns blade reference points given a json file.
 *
 * @param[in] filepath Path of the json file describing the blade.
 */
std::vector<seahowl::core::BladeReferencePoint> get_blade_reference_points_from_json(std::string filepath);

/**
 * @brief Returns Blade instance given a json file.
 *
 * @param[in] filepath Path of the json file describing the blade.
 */
seahowl::core::Blade get_blade_from_json(std::string filepath);

/**
 * @brief Returns tower reference points given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 */
std::vector<seahowl::core::TowerReferencePoint> get_tower_reference_points_from_json(std::string filepath);

/**
 * @brief Returns Tower instance given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 */
seahowl::core::Tower get_tower_from_json(std::string filepath);

/**
 * @brief Returns RotorNacelleAssembly instance given a json file.
 *
 * @param[in] filepath Path of the json file describing the RNA.
 */
seahowl::core::Rotor get_rotor_from_json(std::string filepath);

/**
 * @brief Returns Turbine instance given json files.
 *
 * @param[in] filepath_blades Paths of the json file of each blade.
 * @param[in] filepath_rotor Path of the json file of the RNA.
 * @param[in] filepath_tower Path of the json file of the tower.
 */
seahowl::core::Turbine get_turbine_from_json_files(std::vector<std::string> filepaths_blades,
                                                   std::string filepath_rotor,
                                                   std::string filepath_tower);

/**
 * @brief Returns Turbine instance given a json file.
 *
 * @param[in] filepath Path of the json file describing the turbine.
 */
seahowl::core::Turbine get_turbine_from_json(std::string main_filepath);

/**
 * @brief Returns System instance given a json file.
 *
 * @param[in] filepath Path of the json file describing the system.
 */
seahowl::core::System get_system_from_json(std::string filepath_main,
                                           seahowl::elasto::SystemElasto& system,
                                           std::shared_ptr<seahowl::elasto::MeshElasto> mesh);
