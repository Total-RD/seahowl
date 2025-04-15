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
namespace env {
class FluidSoilModel;
}  // namespace env
}  // namespace seahowl

#include <vector>
#include <string>
#include <memory>
#include <seahowl/io/config_manager.h>
#include <seahowl/io/json_db.h>
#include <filesystem>

namespace fs = std::filesystem;

namespace seahowl {
namespace io {

/**
 * @brief Returns blade elasto reference points given a json file.
 * @param[in] blade_db Blade database.
 */
std::vector<seahowl::elasto::BladeReferencePointElasto> get_blade_elasto_reference_points_from_db(
    const BladeDb& blade_db);

/**
 * @brief Returns blade aero reference points given a json file.
 * @param[in] blade_db Blade database.
 */
std::vector<seahowl::aero::BladeReferencePointAero> get_blade_aero_reference_points_from_db(const BladeDb& blade_db);

/**
 * @brief Returns blade elasto reference points given a json file.
 * @param[in] filepath Path of the json file describing the blade.
 * @param[out] blade Blade to populate.
 */
std::vector<seahowl::elasto::BladeReferencePointElasto> get_blade_elasto_reference_points_from_json(
    const std::string& filepath);

/**
 * @brief Returns blade aero reference points given a json file.
 * @param[in] filepath Path of the json file describing the blade.
 * @param[out] blade Blade to populate.
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
 * @brief Returns tower elasto reference points given a TowerData object.
 * @param[in] tower_data TowerData object.
 */
std::vector<seahowl::elasto::TowerReferencePointElasto> get_tower_elasto_reference_points(const TowerDb& tower_data);
/**
 * @brief Returns tower aero reference points given a json file.
 * @param[in] tower_data TowerData object.
 */
std::vector<seahowl::aero::TowerReferencePointAero> get_tower_aero_reference_points(const TowerDb& tower_data);

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
 * @brief Populates RNA elasto given a json file.
 *
 * @param[in] rna_db RNA database.
 * @param[out] rna RNA to populate.
 */
void populate_rna_elasto_from_db(const RnaDb& rna_db, seahowl::elasto::RotorNacelleAssemblyElasto& rna);

/**
 * @brief Populates RNA aero given a json file.
 *
 * @param[in] filepath Path of the json file describing the rotor.
 * @param[out] rna RNA to populate.
 */
void populate_rna_aero_from_json(const std::string& filepath, seahowl::aero::RotorNacelleAssemblyAero& rna);

/**
 * @brief Populates RNA aero given a json file.
 *
 * @param[in] rna_db RNA database.
 * @param[out] rna RNA to populate.
 */
void populate_rna_aero_from_db(const RnaDb& rna_db, seahowl::aero::RotorNacelleAssemblyAero& rna);

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
 * @brief Creates and returns environmental conditions given a json file.
 *
 * @param[in] filepath Path of the json file describing the environmental conditions.
 */
std::shared_ptr<seahowl::env::FluidSoilModel> get_environmental_model_from_json(const std::string& filepath);

/**
 * @brief Creates and returns environmental conditions g
 *
 * @param[in] environment_db Environment database.
 * @param[in] DATADIR Path of the directory containing the json file.
 */
std::shared_ptr<seahowl::env::FluidSoilModel> get_environmental_model(const EnvironmentDb& environment_db,
                                                                      const fs::path& DATADIR);

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

/**
 * @brief Populates System instance form config manager.
 * @param[in] config Config manager.
 * @param[out] system_core System to populate.
 */
void populate_system_from_config(const app::ConfigManager& config, seahowl::core::System& system_core);

/**
 * @brief Populates System instance given a json file.
 *
 * @param[in] filepath Path of the json file describing the system.
 * @param[out] system_core System to populate.
 * @param[out] output_folder Output folder.
 */
void populate_system(const std::string& filepath, seahowl::core::System& system_core, std::string& output_folder);

}  // namespace io
}  // namespace seahowl
