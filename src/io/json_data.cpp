#include "seahowl/io/json_data.h"

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <nlohmann/detail/macro_scope.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

using json = nlohmann::json;

#define JSONtoCPP NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT

namespace seahowl {
namespace io {

json csv_to_json(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Impossible d'ouvrir le fichier CSV");
    }

    std::string line;
    json result = json::array();  // Tableau JSON pour stocker les données

    // Lire la première ligne pour obtenir les noms des colonnes
    if (!std::getline(file, line)) {
        throw std::runtime_error("Le fichier CSV est vide");
    }
    std::stringstream header_stream(line);
    std::vector<std::string> column_names;
    std::string column_name;

    while (std::getline(header_stream, column_name, ',')) {
        column_names.push_back(column_name);  // Stocker les noms des colonnes
    }

    // Lire les lignes suivantes et les convertir en objets JSON
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        json row = json::object();  // Objet JSON pour une ligne

        for (size_t i = 0; i < column_names.size(); i++) {
            if (!std::getline(ss, cell, ',')) {
                throw std::runtime_error("Nombre de colonnes incohérent dans le fichier CSV");
            }
            row[column_names[i]] = std::stod(cell);  // Associer la cellule au nom de la colonne
        }

        result.push_back(row);  // Ajouter la ligne au tableau JSON
    }

    file.close();
    return json{{"reference_points", result}};
}

void from_json(const json& js, ReferencePointTower& ref_point) {
    if (js.contains("position")) {
        std::vector<double> pos = js["position"];
        ref_point.position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    } else {
        ref_point.position = Eigen::Vector3d(js["position_x"], js["position_y"], js["position_z"]);
    }

    ref_point.diameter = js["diameter"];
    ref_point.thickness = js["thickness"];
    ref_point.density = js.value("density", 0.0);
    ref_point.young_modulus = js.value("young_modulus", 0.0);
    ref_point.poisson_ratio = js.value("poisson_ratio", 0.0);
    ref_point.drag_coefficient_normal = js.value("drag_coefficient_normal", 0.0);
    ref_point.drag_coefficient_axial = js.value("drag_coefficient_axial", 0.0);
    ref_point.added_mass_coefficient_normal = js.value("added_mass_coefficient_normal", 0.0);
    ref_point.added_mass_coefficient_axial = js.value("added_mass_coefficient_axial", 0.0);
    ref_point.buoyancy_factor = js.value("buoyancy_factor", 0.0);
    ref_point.damping_foreaft = js.value("damping_foreaft", 0.0);
    ref_point.damping_sideside = js.value("damping_sideside", 0.0);
    ref_point.damping_axial = js.value("damping_axial", 0.0);
    ref_point.damping_torsion = js.value("damping_torsion", 0.0);
    ref_point.damping_mass = js.value("damping_mass", 0.0);
}

JSONtoCPP(GlobalVariablesTower,
          density,
          young_modulus,
          poisson_ratio,
          drag_coefficient_normal,
          drag_coefficient_axial,
          added_mass_coefficient_normal,
          added_mass_coefficient_axial,
          buoyancy_factor,
          damping_foreaft,
          damping_sideside,
          damping_axial,
          damping_torsion,
          damping_mass)

    void from_json(const json& js, TowerData& tower_data) {
    tower_data.reference_points = js["reference_points"];
    if (js.contains("global_variables")) {
        tower_data.global_variables = js["global_variables"];
        for (auto& point : tower_data.reference_points) {
            point.density = tower_data.global_variables.density;
            point.young_modulus = tower_data.global_variables.young_modulus;
            point.poisson_ratio = tower_data.global_variables.poisson_ratio;
            point.drag_coefficient_normal = tower_data.global_variables.drag_coefficient_normal;
            point.drag_coefficient_axial = tower_data.global_variables.drag_coefficient_axial;
            point.added_mass_coefficient_normal = tower_data.global_variables.added_mass_coefficient_normal;
            point.added_mass_coefficient_axial = tower_data.global_variables.added_mass_coefficient_axial;
            point.buoyancy_factor = tower_data.global_variables.buoyancy_factor;
            point.damping_foreaft = tower_data.global_variables.damping_foreaft;
            point.damping_sideside = tower_data.global_variables.damping_sideside;
            point.damping_axial = tower_data.global_variables.damping_axial;
            point.damping_torsion = tower_data.global_variables.damping_torsion;
            point.damping_mass = tower_data.global_variables.damping_mass;
        }
    }
}

json get_json(const std::string& filepath) {
    json json_data;
    fs::path file_path(filepath);
    std::string extension = file_path.extension().string();
    if (extension == ".csv") {
        // Convert CSV to JSON
        json_data = csv_to_json(filepath);
    } else if (extension == ".json") {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Impossible d'ouvrir le fichier JSON");
        }
        file >> json_data;
        file.close();

    } else {
        throw std::runtime_error("Unsupported file format: " + extension);
    }
    return json_data;
}

TowerData read_tower(const std::string& filepath) {
    TowerData tower_data;
    json json_data = get_json(filepath);
    from_json(json_data, tower_data);
    return tower_data;
}

void from_json(const json& js, GlobalVariablesBlade& global_vars) {
    global_vars.damping_flapwise = js.at("damping_flapwise").get<double>();
    global_vars.damping_edgewise = js.at("damping_edgewise").get<double>();
    global_vars.damping_axial = js.at("damping_axial").get<double>();
    global_vars.damping_torsion = js.at("damping_torsion").get<double>();
    global_vars.damping_mass = js.at("damping_mass").get<double>();
    global_vars.offset_gravity = Eigen::Vector2d(js.at("offset_gravity")[0], js.at("offset_gravity")[1]);
    global_vars.offset_elastic = Eigen::Vector2d(js.at("offset_elastic")[0], js.at("offset_elastic")[1]);
}

void from_json(const json& js, ReferencePointBlade& ref_point) {
    ref_point.coordinates = Eigen::Vector3d(js.at("coordinates")[0], js.at("coordinates")[1], js.at("coordinates")[2]);
    ref_point.twist = js.at("twist").get<double>();
    ref_point.fraction = js.at("fraction").get<double>();

    // Convert stiffness_matrix
    const auto& stiffness = js.at("stiffness_matrix");
    ref_point.stiffness_matrix = Eigen::MatrixXd(stiffness.size(), stiffness[0].size());
    for (size_t i = 0; i < stiffness.size(); ++i) {
        for (size_t j = 0; j < stiffness[i].size(); ++j) {
            ref_point.stiffness_matrix(i, j) = stiffness[i][j];
        }
    }

    // Convert mass_matrix
    const auto& mass = js.at("mass_matrix");
    ref_point.mass_matrix = Eigen::MatrixXd(mass.size(), mass[0].size());
    for (size_t i = 0; i < mass.size(); ++i) {
        for (size_t j = 0; j < mass[i].size(); ++j) {
            ref_point.mass_matrix(i, j) = mass[i][j];
        }
    }

    ref_point.chord = js.at("chord").get<double>();
    ref_point.airfoil_file = js.at("airfoil_file").get<std::string>();

    auto main_directory = fs::path(filepath).parent_path();
    auto airfoil_filepath = main_directory / ref_point.airfoil_file;
    std::vector<AirfoilData> airfoil_data_list = read_airfoil(airfoil_filepath.u8string());
    ref_point.offset_aero = Eigen::Vector2d(js.at("offset_aero")[0], js.at("offset_aero")[1]);
}

void from_json(const json& js, BladeData& blade_data) {
    blade_data.global_variables = js.at("global_variables").get<GlobalVariablesBlade>();
    blade_data.reference_points = js.at("reference_points").get<std::vector<ReferencePointBlade>>();
}

BladeData read_blade(const std::string& filepath) {
    BladeData blade_data;
    json json_data = get_json(filepath);
    from_json(json_data, blade_data);
    return blade_data;
}

void from_json(const json& js, AirfoilData& airfoil_data) {
    airfoil_data.reynolds_number = js.at("reynolds_number").get<double>();
    airfoil_data.header = js.at("header").get<std::vector<std::string>>();
    airfoil_data.coefficients = js.at("coefficients").get<std::vector<std::vector<double>>>();
}

std::vector<AirfoilData> read_airfoil(const std::string& filepath) {
    AirfoilData airfoil_data;
    json json_data = get_json(filepath);
    return json_data.get<std::vector<AirfoilData>>();
}

void from_json(const json& js, EnvironmentDb& env) {
    env.gravity = Eigen::Vector3d(js.at("gravity")[0], js.at("gravity")[1], js.at("gravity")[2]);
    env.ramp_start = js.at("ramp_start").get<double>();
    env.ramp_end = js.at("ramp_end").get<double>();

    if (js.contains("wind") && !js["wind"].is_null()) {
        env.wind = js.at("wind").get<WindDb>();
    } else {
        env.wind = std::nullopt;
    }

    if (js.contains("sea") && !js["sea"].is_null()) {
        env.sea = js.at("sea").get<SeaDb>();
    } else {
        env.sea = std::nullopt;
    }

    if (js.contains("soil") && !js["soil"].is_null()) {
        env.soil = js.at("soil").get<SoilDb>();
    } else {
        env.soil = std::nullopt;
    }
}

}  // namespace io
}  // namespace seahowl
