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
}  // namespace seahowl

#include <vector>
#include <string>
#include <memory>

#include <chrono/physics/ChSystemSMC.h>
#include <chrono/fea/ChMesh.h>

std::vector<seahowl::core::BladeReferencePoint> get_blade_reference_points_from_json(std::string filepath);

seahowl::core::Blade get_blade_from_json(std::string filepath);

std::vector<seahowl::core::TowerReferencePoint> get_tower_reference_points_from_json(std::string filepath);

seahowl::core::Tower get_tower_from_json(std::string filepath);

seahowl::core::Rotor get_rotor_from_json(std::string filepath);

seahowl::core::Turbine get_turbine_from_json_files(std::vector<std::string> filepaths_blades,
                                                   std::string filepath_rotor,
                                                   std::string filepath_tower);

seahowl::core::Turbine get_turbine_from_json(std::string main_filepath);

seahowl::core::System get_system_from_json(std::string filepath_main,
                                           chrono::ChSystemSMC& system,
                                           std::shared_ptr<chrono::fea::ChMesh> mesh);