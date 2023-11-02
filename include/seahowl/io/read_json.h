#pragma once

namespace seahowl {
namespace core {
class Blade;
class RotorNacelleAssembly;
class Tower;
class Turbine;
class System;
struct BladeReferencePoint;
struct TowerReferencePoint;
}  // namespace core
namespace elasto {
class SystemElasto;
class MeshElasto;
class BladeElasto;
class TowerElasto;
class RotorNacelleAssemblyElasto;
struct BladeReferencePointElasto;
struct TowerReferencePointElasto;
}  // namespace elasto
namespace aero {
class BladeAero;
class TowerAero;
class RotorNacelleAssemblyAero;
struct BladeReferencePointAero;
struct TowerReferencePointAero;
}  // namespace aero
}  // namespace seahowl

#include <vector>
#include <string>
#include <memory>

/**
 * @brief Returns blade elasto reference points given a json file.
 *
 * @param[in] filepath Path of the json file describing the blade.
 */
std::vector<seahowl::elasto::BladeReferencePointElasto> get_blade_elasto_reference_points_from_json(
    const std::string& filepath);

/**
 * @brief Returns blade aero reference points given a json file.
 *
 * @param[in] filepath Path of the json file describing the blade.
 */
std::vector<seahowl::aero::BladeReferencePointAero> get_blade_aero_reference_points_from_json(
    const std::string& filepath);

/**
 * @brief Populates blade elasto given a json file.
 *
 * @param[in] filepath Path of the json file describing the blade.
 * @param[out] blade Blade to populate.
 */
void populate_blade_elasto_from_json(const std::string& filepath, seahowl::elasto::BladeElasto& blade);

/**
 * @brief Populates blade aero given a json file.
 *
 * @param[in] filepath Path of the json file describing the blade.
 * @param[out] blade Blade to populate.
 */
void populate_blade_aero_from_json(const std::string& filepath, seahowl::aero::BladeAero& blade);

/**
 * @brief Populates blade given a json file.
 *
 * @param[in] filepath Path of the json file describing the blade.
 * @param[out] blade Blade to populate.
 */
void populate_blade_from_json(const std::string& filepath, seahowl::core::Blade& blade);

/**
 * @brief Returns tower elasto reference points given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 */
std::vector<seahowl::elasto::TowerReferencePointElasto> get_tower_elasto_reference_points_from_json(
    const std::string& filepath);

/**
 * @brief Returns tower aero reference points given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 */
std::vector<seahowl::aero::TowerReferencePointAero> get_tower_aero_reference_points_from_json(
    const std::string& filepath);

/**
 * @brief Populates tower elasto given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 * @param[out] tower Tower to populate.
 */
void populate_tower_elasto_from_json(const std::string& filepath, seahowl::elasto::TowerElasto& tower);

/**
 * @brief Populates tower aero given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 * @param[out] tower Tower aero to populate.
 */
void populate_tower_aero_from_json(const std::string& filepath, seahowl::aero::TowerAero& tower);

/**
 * @brief Populates tower given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 * @param[out] tower Tower to populate.
 */
void populate_tower_from_json(const std::string& filepath, seahowl::core::Tower& tower);

/**
 * @brief Populates RNA elasto given a json file.
 *
 * @param[in] filepath Path of the json file describing the rotor.
 * @param[out] rotor RNA to populate.
 */
void populate_rna_elasto_from_json(const std::string& filepath, seahowl::elasto::RotorNacelleAssemblyElasto& rna);

/**
 * @brief Populates RNA aero given a json file.
 *
 * @param[in] filepath Path of the json file describing the rotor.
 * @param[out] rna RNA to populate.
 */
void populate_rna_aero_from_json(const std::string& filepath, seahowl::aero::RotorNacelleAssemblyAero& rna);

/**
 * @brief Populates RNA given a json file.
 *
 * @param[in] filepath Path of the json file describing the rotor.
 * @param[out] rna RNA to populate.
 */
void populate_rna_from_json(const std::string& filepath, seahowl::core::RotorNacelleAssembly& rna);

/**
 * @brief Populates turbine given a json file.
 *
 * @param[in] filepath Path of the json file describing the turbine.
 * @param[out] system_core System to add the new turbine.
 */
void add_turbine_to_system_from_json(const std::string& filepath, seahowl::core::System& system_core);

/**
 * @brief Populates turbine given a json file.
 *
 * @param[in] filepath Path of the json file describing the turbine.
 * @param[out] turbine Turbine to populate.
 */
void populate_turbine_from_json(const std::string& filepath, seahowl::core::Turbine& turbine);

/**
 * @brief Populates environmental conditions given a json file and a system.
 *
 * @param[in] filepath Path of the json file describing the environmental conditions.
 * @param[out] system_core System to populate.
 */
void populate_environmental_conditions_from_json(const std::string& filepath, seahowl::core::System& system_core);

/**
 * @brief Returns System instance given a json file.
 *
 * @param[in] filepath Path of the json file describing the system.
 * @param[out] system_core System to populate.
 */
void populate_system_from_json(const std::string& filepath_main, seahowl::core::System& system_core);
