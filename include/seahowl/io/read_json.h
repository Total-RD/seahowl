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
class BladeElasto;
class TowerElasto;
class RotorElasto;
struct BladeReferencePointElasto;
struct TowerReferencePointElasto;
}  // namespace elasto
namespace aero {
class BladeAero;
class TowerAero;
class RotorAero;
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
    std::string filepath);

/**
 * @brief Returns blade aero reference points given a json file.
 *
 * @param[in] filepath Path of the json file describing the blade.
 */
std::vector<seahowl::aero::BladeReferencePointAero> get_blade_aero_reference_points_from_json(std::string filepath);

/**
 * @brief Populates blade elasto given a json file.
 *
 * @param[in] filepath Path of the json file describing the blade.
 * @param[out] blade Blade to populate.
 */
void populate_blade_elasto_from_json(std::string filepath, seahowl::elasto::BladeElasto& blade);

/**
 * @brief Populates blade aero given a json file.
 *
 * @param[in] filepath Path of the json file describing the blade.
 * @param[out] blade Blade to populate.
 */
void populate_blade_aero_from_json(std::string filepath, seahowl::aero::BladeAero& blade);

/**
 * @brief Populates blade given a json file.
 *
 * @param[in] filepath Path of the json file describing the blade.
 * @param[out] blade Blade to populate.
 */
void populate_blade_from_json(std::string filepath, seahowl::core::Blade& blade);

/**
 * @brief Returns tower elasto reference points given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 */
std::vector<seahowl::elasto::TowerReferencePointElasto> get_tower_elasto_reference_points_from_json(
    std::string filepath);

/**
 * @brief Returns tower aero reference points given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 */
std::vector<seahowl::aero::TowerReferencePointAero> get_tower_aero_reference_points_from_json(std::string filepath);

/**
 * @brief Populates tower elasto given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 * @param[out] tower Tower to populate.
 */
void populate_tower_elasto_from_json(std::string filepath, seahowl::elasto::TowerElasto& tower);

/**
 * @brief Populates tower aero given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 * @param[out] tower Tower aero to populate.
 */
void populate_tower_aero_from_json(std::string filepath, seahowl::aero::TowerAero& tower);

/**
 * @brief Populates tower given a json file.
 *
 * @param[in] filepath Path of the json file describing the tower.
 * @param[out] tower Tower to populate.
 */
void populate_tower_from_json(std::string filepath, seahowl::core::Tower& tower);

/**
 * @brief Populates rotor elasto given a json file.
 *
 * @param[in] filepath Path of the json file describing the rotor.
 * @param[out] rotor Rotor to populate.
 */
void populate_rotor_elasto_from_json(std::string filepath, seahowl::elasto::RotorElasto& rotor);

/**
 * @brief Populates rotor elasto given a json file.
 *
 * @param[in] filepath Path of the json file describing the rotor.
 * @param[out] rotor Rotor to populate.
 */
void populate_rotor_aero_from_json(std::string filepath, seahowl::aero::RotorAero& rotor);

/**
 * @brief Populates rotor given a json file.
 *
 * @param[in] filepath Path of the json file describing the rotor.
 * @param[out] rotor Rotor to populate.
 */
void populate_rotor_from_json(std::string filepath, seahowl::core::Rotor& rotor);

/**
 * @brief Populates turbine given a json file.
 *
 * @param[in] filepath Path of the json file describing the turbine.
 * @param[out] turbine Turbine to populate.
 */
void populate_turbine_from_json(std::string filepath, seahowl::core::Turbine& turbine);

/**
 * @brief Returns System instance given a json file.
 *
 * @param[in] filepath Path of the json file describing the system.
 * @param[out] system System to populate.
 */
void populate_system_from_json(std::string filepath_main, seahowl::core::System& system_core);
