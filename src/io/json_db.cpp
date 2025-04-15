#include "seahowl/io/json_db.h"
#include "seahowl/io/store_db_models.h"

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <nlohmann/detail/macro_scope.hpp>
#include <spdlog/spdlog.h>
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

Eigen::MatrixX<double> get_matrix_from_vector_of_vectorsjj(std::vector<std::vector<double>> matvec) {
    int mat_nrows = matvec.size();
    int mat_ncols = 0;
    for (int irow = 0; irow < matvec.size(); irow++) {
        mat_ncols = matvec[0].size();
        if (matvec[irow].size() != mat_ncols) {
            throw std::runtime_error(
                "Matrix does not have a consistant number of columns: " + std::to_string(mat_ncols) +
                " columns at row 1 and " + std::to_string(matvec[irow].size()) + " columns at row " +
                std::to_string(irow + 1) + ".");
        }
    }
    Eigen::MatrixX<double> mat(mat_nrows, mat_ncols);
    for (int irow = 0; irow < mat_nrows; irow++) {
        for (int icol = 0; icol < mat_ncols; icol++) {
            mat(irow, icol) = matvec[irow][icol];
        }
        std::cout << std::endl;
    }
    return mat;
}

void from_json(const json& js, ReferencePointTowerDb& ref_point) {
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

JSONtoCPP(GlobalVariablesTowerDb,
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

    void from_json(const json& js, TowerDb& tower_db) {
    tower_db.reference_points = js["reference_points"];
    if (js.contains("global_variables")) {
        tower_db.global_variables = js["global_variables"];
        for (auto& point : tower_db.reference_points) {
            point.density = tower_db.global_variables.density;
            point.young_modulus = tower_db.global_variables.young_modulus;
            point.poisson_ratio = tower_db.global_variables.poisson_ratio;
            point.drag_coefficient_normal = tower_db.global_variables.drag_coefficient_normal;
            point.drag_coefficient_axial = tower_db.global_variables.drag_coefficient_axial;
            point.added_mass_coefficient_normal = tower_db.global_variables.added_mass_coefficient_normal;
            point.added_mass_coefficient_axial = tower_db.global_variables.added_mass_coefficient_axial;
            point.buoyancy_factor = tower_db.global_variables.buoyancy_factor;
            point.damping_foreaft = tower_db.global_variables.damping_foreaft;
            point.damping_sideside = tower_db.global_variables.damping_sideside;
            point.damping_axial = tower_db.global_variables.damping_axial;
            point.damping_torsion = tower_db.global_variables.damping_torsion;
            point.damping_mass = tower_db.global_variables.damping_mass;
        }
    }
}

json get_json(const std::string& filepath) {
    json json_db;
    fs::path file_path(filepath);
    std::string extension = file_path.extension().string();
    if (extension == ".csv") {
        // Convert CSV to JSON
        json_db = csv_to_json(filepath);
    } else if (extension == ".json") {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Impossible d'ouvrir le fichier JSON");
        }
        file >> json_db;
        file.close();

    } else {
        throw std::runtime_error("Unsupported file format: " + extension);
    }
    return json_db;
}

TowerDb read_tower_db(const std::string& filepath) {
    TowerDb tower_db;
    json json_db = get_json(filepath);
    from_json(json_db, tower_db);
    return tower_db;
}

void from_json(const json& js, GlobalVariablesBladeDb& global_vars) {
    global_vars.damping_flapwise = js.at("damping_flapwise").get<double>();
    global_vars.damping_edgewise = js.at("damping_edgewise").get<double>();
    global_vars.damping_axial = js.at("damping_axial").get<double>();
    global_vars.damping_torsion = js.at("damping_torsion").get<double>();
    global_vars.damping_mass = js.at("damping_mass").get<double>();
    global_vars.offset_gravity = Eigen::Vector2d(js.at("offset_gravity")[0], js.at("offset_gravity")[1]);
    global_vars.offset_elastic = Eigen::Vector2d(js.at("offset_elastic")[0], js.at("offset_elastic")[1]);
}

void from_json(const json& js, ReferencePointBladeDb& ref_point) {
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
    ref_point.offset_aero = Eigen::Vector2d(js.at("offset_aero")[0], js.at("offset_aero")[1]);
}

void from_json(const json& js, BladeDb& blade_db) {
    blade_db.global_variables = js.at("global_variables").get<GlobalVariablesBladeDb>();
    blade_db.reference_points = js.at("reference_points").get<std::vector<ReferencePointBladeDb>>();
}

void from_json(const json& js, AirfoilDb& airfoil_db) {
    airfoil_db.reynolds_number = js.at("reynolds_number").get<double>();
    airfoil_db.header = js.at("header").get<std::vector<std::string>>();
    airfoil_db.coefficients = js.at("coefficients").get<std::vector<std::vector<double>>>();
}

std::vector<AirfoilDb> read_airfoil_db(const std::string& filepath) {
    AirfoilDb airfoil_db;
    json json_db = get_json(filepath);
    return json_db.get<std::vector<AirfoilDb>>();
}

BladeDb read_blade_db(const std::string& filepath) {
    BladeDb blade_db;
    json json_db = get_json(filepath);
    from_json(json_db, blade_db);

    auto main_directory = fs::path(filepath).parent_path();

    for (auto& ref_point : blade_db.reference_points) {
        if (!ref_point.airfoil_file.empty()) {
            auto airfoil_filepath = main_directory / ref_point.airfoil_file;
            ref_point.airfoil_db_list = read_airfoil_db(airfoil_filepath.u8string());
        }
    }
    return blade_db;
}

void from_json(const json& js, ShaftDb& shaft) {
    shaft.tilt = js.at("tilt").get<double>();
    shaft.distance_from_towertop = js.at("distance_from_towertop").get<double>();
}

void from_json(const json& js, NacelleDb& nacelle) {
    nacelle.position_from_towertop = Eigen::Vector3d(
        js.at("position_from_towertop")[0], js.at("position_from_towertop")[1], js.at("position_from_towertop")[2]);
    nacelle.mass = js.at("mass").get<double>();

    const auto& inertia = js.at("inertia");
    nacelle.inertia = get_matrix_from_vector_of_vectorsjj(inertia);
    nacelle.yaw_bearing_mass = js.at("yaw_bearing_mass").get<double>();
}

void from_json(const json& js, DrivetrainDb& drivetrain) {
    drivetrain.generator_inertia = js.at("generator_inertia").get<double>();
    drivetrain.gearbox_efficiency = js.at("gearbox_efficiency").get<double>();
    drivetrain.gearbox_ratio = js.at("gearbox_ratio").get<double>();
    drivetrain.generator_efficiency = js.at("generator_efficiency").get<double>();
}

void from_json(const json& js, HubDb& hub) {
    hub.radius = js.at("radius").get<double>();
    hub.position_from_apex =
        Eigen::Vector3d(js.at("position_from_apex")[0], js.at("position_from_apex")[1], js.at("position_from_apex")[2]);
    hub.overhang = js.at("overhang").get<double>();
    hub.mass = js.at("mass").get<double>();

    const auto& inertia = js.at("inertia");
    hub.inertia = get_matrix_from_vector_of_vectorsjj(inertia);
}

void from_json(const json& js, RnaDb& rna) {
    rna.shaft = js.at("shaft").get<ShaftDb>();
    rna.nacelle = js.at("nacelle").get<NacelleDb>();
    rna.drivetrain = js.at("drivetrain").get<DrivetrainDb>();
    rna.hub = js.at("hub").get<HubDb>();
}

RnaDb read_rna_db(const std::string& filepath) {
    RnaDb rna_db;
    json json_db = get_json(filepath);
    from_json(json_db, rna_db);
    return rna_db;
}

void from_json(const json& js, WindOptionDb& options) {
    options.reference_height = js.at("reference_height").get<double>();
    options.shear_coefficient = js.at("shear_coefficient").get<double>();
    options.velocity_start =
        Eigen::Vector3d(js.at("velocity_start")[0], js.at("velocity_start")[1], js.at("velocity_start")[2]);
    options.velocity_end =
        Eigen::Vector3d(js.at("velocity_end")[0], js.at("velocity_end")[1], js.at("velocity_end")[2]);
    options.time_start = js.at("time_start").get<double>();
    options.time_end = js.at("time_end").get<double>();
}

void from_json_inflowwind(const json& js, WindOptionDb& options) {
    if (js.contains("file_inflowwind")) {
        options.file_inflowwind = js.at("file_inflowwind").get<std::string>();
    } else {
        throw std::runtime_error("InflowWind file not defined.");
    }

    if (js.contains("zmin")) {
        options.zmin = js.at("zmin").get<double>();
    } else {
        spdlog::warn(
            "Minimum height for wind speed calculation not defined for InflowWind model, using default zmin={}.");
    }
}

void from_json(const json& js, WindDb& wind) {
    wind.type = js.at("type").get<std::string>();
    wind.air_density = js.at("air_density").get<double>();
    if (wind.type == "ramp") {
        wind.options = js.at("options").get<WindOptionDb>();
    } else if (wind.type == "inflowwind") {
        from_json_inflowwind(js.at("options"), wind.options);
    } else {
        throw std::runtime_error("Unknown wind type: " + wind.type);
    }
}

void from_json(const json& js, SeaOptionDb& options) {
    options.type = js.at("type").get<std::string>();
    if (options.type == "regular") {
        options.wave_height = js.at("wave_height").get<double>();
        options.wave_period = js.at("wave_period").get<double>();

    } else if (options.type == "irregular") {
        options.wave_height = js.at("wave_height").get<double>();
        options.wave_period = js.at("wave_period").get<double>();
        options.num_bodies = js.at("num_bodies").get<int>();
        options.frequency_min = js.at("frequency_min").get<double>();
        options.frequency_max = js.at("frequency_max").get<double>();
        options.nfrequencies = js.at("nfrequencies").get<int>();
        options.peak_enhancement_factor = js.at("peak_enhancement_factor").get<double>();
        options.is_normalized = js.at("is_normalized").get<bool>();
        options.seed = js.at("seed").get<int>();
        options.dt = js.at("dt").get<double>();
        options.duration = js.at("duration").get<double>();
        options.wave_stretching = js.at("wave_stretching").get<bool>();
    }
}
void from_json_current(const json& js, SeaOptionDb& options) {
    options.type = js.at("type").get<std::string>();
    options.direction = Eigen::Vector3d(js.at("direction")[0], js.at("direction")[1], js.at("direction")[2]);
    options.velocity_surface = js.at("velocity_surface").get<double>();
    options.velocity_seabed = js.at("velocity_seabed").get<double>();
}

void from_json(const json& js, SeaDb& sea) {
    sea.type = js.at("type").get<std::string>();
    sea.water_density = js.at("water_density").get<double>();
    sea.mean_water_level = js.at("mean_water_level").get<double>();
    sea.water_depth = js.at("water_depth").get<double>();
    if (sea.type == "HydroChrono" || sea.type == "hydrochrono") {
        sea.options = js.at("options").get<SeaOptionDb>();
    } else if (sea.type == "current") {
        from_json_current(js.at("options"), sea.options);
    }
}

void from_json(const json& js, SoilOptionDb& options) {
    options.stiffness_normal = js.at("stiffness_normal").get<double>();
    options.stiffness_shear = js.at("stiffness_shear").get<double>();
    options.soil_position = js.at("soil_position").get<double>();
}

void from_json(const json& js, SoilDb& soil) {
    soil.type = js.at("type").get<std::string>();
    soil.options = js.at("options").get<SoilOptionDb>();
}

void from_json(const json& js, EnvironmentDb& env) {
    env.gravity = Eigen::Vector3d(js.at("gravity")[0], js.at("gravity")[1], js.at("gravity")[2]);
    env.wind = js.at("wind").get<WindDb>();

    if (js.contains("ramp_start")) {
        env.ramp_start = js.at("ramp_start").get<double>();
    } else {
        env.ramp_start = std::nullopt;
    }
    if (js.contains("ramp_end")) {
        env.ramp_end = js.at("ramp_end").get<double>();
    } else {
        env.ramp_end = std::nullopt;
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

EnvironmentDb read_environment_db(const std::string& filepath) {
    EnvironmentDb env_db;
    json json_db = get_json(filepath);
    from_json(json_db, env_db);
    return env_db;
}

}  // namespace io
}  // namespace seahowl
