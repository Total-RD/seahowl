#pragma once

namespace seahowl {
namespace core {
class Blade;
class Rotor;
class Tower;
class Turbine;
struct BladeReferencePoint;
struct TowerReferencePoint;
}  // namespace core
}  // namespace seahowl

#include <vector>
#include <string>
#include <memory>

std::vector<seahowl::core::BladeReferencePoint> get_blade_reference_points_from_json(std::string filepath);

seahowl::core::Blade get_blade_from_json(std::string filepath);

std::vector<seahowl::core::TowerReferencePoint> get_tower_reference_points_from_json(std::string filepath);

seahowl::core::Tower get_tower_from_json(std::string filepath);

seahowl::core::Rotor get_rotor_from_json(std::string filepath);

seahowl::core::Turbine get_turbine_from_json(std::vector<std::string> filepaths_blades,
                                             std::string filepath_rotor,
                                             std::string filepath_tower);


seahowl::core::Turbine get_turbine_from_main_file(std::string main_filepath);