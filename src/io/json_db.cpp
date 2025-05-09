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

/**
 * @brief Convert a string to lowercase
 * @param input The input string
 * @return The lowercase string
 */
std::string to_lowercase(const std::string& input) {
    std::string result = input;
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return std::tolower(c); });
    return result;
}

/**
 * @brief read optional value from json
 * @param js json object
 * @param key key to read
 * @return optional value
 */
std::optional<double> read_optional_value(const json& js, const std::string& key) {
    if (js.contains(key)) {
        return js.at(key).get<double>();
    } else {
        return std::nullopt;
    }
}

/**
 * @brief read value from json with default global value
 * @details If the key is not found in the json object, it will return the value from the global variables if it exists.
 *          If the key is not found and the global variable is not set, it will throw an error.
 * @param js json object
 * @param key key to read
 * @param global_vars global variables
 * @return value
 */
double read_value(const json& js, const std::string& key, const std::optional<double>& global_vars) {
    if (js.contains(key)) {
        return js.at(key).get<double>();
    } else {
        if (global_vars.has_value())
            return global_vars.value();
        else
            throw std::runtime_error(
                key + " is not defined in the JSON file and no default value is provided in the global variables.");
    }
}

/**
 * @brief Convert CSV file to JSON format
 * @param filename The name of the CSV file
 * @return JSON object containing the data
 */
json csv_to_json(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open the CSV file");
    }

    std::string line;
    json result = json::array();  // JSON array to store the data

    // Read the first line to get the column names
    if (!std::getline(file, line)) {
        throw std::runtime_error("The CSV file is empty");
    }
    std::stringstream header_stream(line);
    std::vector<std::string> column_names;
    std::string column_name;

    while (std::getline(header_stream, column_name, ',')) {
        column_names.push_back(column_name);  // Store the column names
    }

    // Read the subsequent lines and convert them into JSON objects
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        json row = json::object();  // JSON object for a row

        for (size_t i = 0; i < column_names.size(); i++) {
            if (!std::getline(ss, cell, ',')) {
                throw std::runtime_error("Inconsistent number of columns in the CSV file");
            }
            row[column_names[i]] = std::stod(cell);  // Map the cell to the column name
        }

        result.push_back(row);  // Add the row to the JSON array
    }

    file.close();
    return json{{"reference_points", result}};
}

Eigen::MatrixX<double> get_matrix_from_vector_of_vectors(std::vector<std::vector<double>> matvec) {
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

/**
 * @brief Get JSON data from a file
 * @details This function reads a file (Json or Csv) and converts it to a JSON object.
 * @param filepath The path to the file
 * @return JSON object containing the data
 */
json get_json(const std::string& filepath) {
    if (!fs::exists(filepath)) {
        throw std::runtime_error("File does not exist: " + filepath);
    }

    json json_db;
    fs::path file_path(filepath);
    std::string extension = file_path.extension().string();
    if (extension == ".csv") {
        // Convert CSV to JSON
        json_db = csv_to_json(filepath);
    } else if (extension == ".json") {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to open the JSON file" + filepath);
        }
        file >> json_db;
        file.close();

    } else {
        throw std::runtime_error("Unsupported file format: " + extension);
    }
    return json_db;
}

void from_json(const json& js, ReferencePointTowerDb& ref_point, GlobalVariablesTowerDb& global_vars) {
    if (js.contains("position")) {
        std::vector<double> pos = js["position"];
        ref_point.position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    } else {
        ref_point.position = Eigen::Vector3d(js["position_x"], js["position_y"], js["position_z"]);
    }

    ref_point.diameter = js.at("diameter").get<double>();
    ref_point.thickness = js.at("thickness").get<double>();
    ref_point.density = read_value(js, "density", global_vars.density);
    ref_point.young_modulus = read_value(js, "young_modulus", global_vars.young_modulus);
    ref_point.poisson_ratio = read_value(js, "poisson_ratio", global_vars.poisson_ratio);
    ref_point.drag_coefficient_normal = read_value(js, "drag_coefficient_normal", global_vars.drag_coefficient_normal);
    ref_point.drag_coefficient_axial = read_value(js, "drag_coefficient_axial", global_vars.drag_coefficient_axial);
    ref_point.added_mass_coefficient_normal =
        read_value(js, "added_mass_coefficient_normal", global_vars.added_mass_coefficient_normal);
    ref_point.added_mass_coefficient_axial =
        read_value(js, "added_mass_coefficient_axial", global_vars.added_mass_coefficient_axial);
    ref_point.buoyancy_factor = read_value(js, "buoyancy_factor", global_vars.buoyancy_factor);
    ref_point.damping_foreaft = read_value(js, "damping_foreaft", global_vars.damping_foreaft);
    ref_point.damping_sideside = read_value(js, "damping_sideside", global_vars.damping_sideside);
    ref_point.damping_axial = read_value(js, "damping_axial", global_vars.damping_axial);
    ref_point.damping_torsion = read_value(js, "damping_torsion", global_vars.damping_torsion);
    ref_point.damping_mass = read_value(js, "damping_mass", global_vars.damping_mass);
}

void from_json(const json& js, GlobalVariablesTowerDb& global_vars) {
    global_vars.density = read_optional_value(js, "density");
    global_vars.young_modulus = read_optional_value(js, "young_modulus");
    global_vars.poisson_ratio = read_optional_value(js, "poisson_ratio");
    global_vars.drag_coefficient_normal = read_optional_value(js, "drag_coefficient_normal");
    global_vars.drag_coefficient_axial = read_optional_value(js, "drag_coefficient_axial");
    global_vars.added_mass_coefficient_normal = read_optional_value(js, "added_mass_coefficient_normal");
    global_vars.added_mass_coefficient_axial = read_optional_value(js, "added_mass_coefficient_axial");
    global_vars.buoyancy_factor = read_optional_value(js, "buoyancy_factor");
    global_vars.damping_foreaft = read_optional_value(js, "damping_foreaft");
    global_vars.damping_sideside = read_optional_value(js, "damping_sideside");
    global_vars.damping_axial = read_optional_value(js, "damping_axial");
    global_vars.damping_torsion = read_optional_value(js, "damping_torsion");
    global_vars.damping_mass = read_optional_value(js, "damping_mass");
}

void from_json(const json& js, TowerDb& tower_db) {
    if (js.contains("global_variables")) {
        tower_db.global_variables = js["global_variables"];
    }
    for (const auto& ref_point_json : js["reference_points"]) {
        ReferencePointTowerDb ref_point;
        from_json(ref_point_json, ref_point, tower_db.global_variables);
        tower_db.reference_points.push_back(ref_point);
    }
}

TowerDb read_tower_json(const std::string& filepath) {
    TowerDb tower_db;
    json json_db = get_json(filepath);
    from_json(json_db, tower_db);
    return tower_db;
}

void from_json(const json& js, GlobalVariablesBladeDb& global_vars) {
    global_vars.damping_flapwise = read_optional_value(js, "damping_flapwise");
    global_vars.damping_edgewise = read_optional_value(js, "damping_edgewise");
    global_vars.damping_axial = read_optional_value(js, "damping_axial");
    global_vars.damping_torsion = read_optional_value(js, "damping_torsion");
    global_vars.damping_mass = read_optional_value(js, "damping_mass");

    if (js.contains("offset_gravity")) {
        global_vars.offset_gravity = Eigen::Vector2d(js.at("offset_gravity")[0], js.at("offset_gravity")[1]);
    } else {
        global_vars.offset_gravity = std::nullopt;
    }
    if (js.contains("offset_elastic")) {
        global_vars.offset_elastic = Eigen::Vector2d(js.at("offset_elastic")[0], js.at("offset_elastic")[1]);
    } else {
        global_vars.offset_elastic = std::nullopt;
    }
}

void from_json(const json& js, ReferencePointBladeDb& ref_point, const GlobalVariablesBladeDb& global_vars) {
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

    ref_point.damping_flapwise = read_value(js, "damping_flapwise", global_vars.damping_flapwise);
    ref_point.damping_edgewise = read_value(js, "damping_edgewise", global_vars.damping_edgewise);
    ref_point.damping_axial = read_value(js, "damping_axial", global_vars.damping_axial);
    ref_point.damping_torsion = read_value(js, "damping_torsion", global_vars.damping_torsion);
    ref_point.damping_mass = read_value(js, "damping_mass", global_vars.damping_mass);

    if (js.contains("offset_elastic"))
        ref_point.offset_elastic = Eigen::Vector2d(js.at("offset_elastic")[0], js.at("offset_elastic")[1]);
    else if (global_vars.offset_elastic.has_value())
        ref_point.offset_elastic = global_vars.offset_elastic.value();
    else
        throw std::runtime_error(
            "offset_elastic is not defined in the JSON file and no default value is provided in the global variables.");
    if (js.contains("offset_gravity"))
        ref_point.offset_gravity = Eigen::Vector2d(js.at("offset_gravity")[0], js.at("offset_gravity")[1]);
    else if (global_vars.offset_gravity.has_value())
        ref_point.offset_gravity = global_vars.offset_gravity.value();
    else
        throw std::runtime_error(
            "offset_gravity is not defined in the JSON file and no default value is provided in the global variables.");
}

void from_json(const json& js, BladeDb& blade_db) {
    if (js.contains("global_variables")) {
        blade_db.global_variables = js.at("global_variables").get<GlobalVariablesBladeDb>();
    }
    for (auto& ref_point_json : js.at("reference_points")) {
        ReferencePointBladeDb ref_point;
        from_json(ref_point_json, ref_point, blade_db.global_variables);
        blade_db.reference_points.push_back(ref_point);
    }
}

void from_json(const json& js, AirfoilDb& airfoil_db) {
    airfoil_db.reynolds_number = js.at("reynolds_number").get<double>();
    airfoil_db.header = js.at("header").get<std::vector<std::string>>();
    airfoil_db.coefficients = js.at("coefficients").get<std::vector<std::vector<double>>>();
}

std::vector<AirfoilDb> read_airfoil_json(const std::string& filepath) {
    AirfoilDb airfoil_db;
    json json_db = get_json(filepath);
    return json_db.get<std::vector<AirfoilDb>>();
}

BladeDb read_blade_json(const std::string& filepath) {
    BladeDb blade_db;
    json json_db = get_json(filepath);
    from_json(json_db, blade_db);

    auto main_directory = fs::path(filepath).parent_path();

    for (auto& ref_point : blade_db.reference_points) {
        if (!ref_point.airfoil_file.empty()) {
            auto airfoil_filepath = main_directory / ref_point.airfoil_file;
            ref_point.airfoil_db_list = read_airfoil_json(airfoil_filepath.u8string());
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
    nacelle.inertia = get_matrix_from_vector_of_vectors(inertia);
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
    hub.inertia = get_matrix_from_vector_of_vectors(inertia);
}

void from_json(const json& js, RnaDb& rna) {
    rna.shaft = js.at("shaft").get<ShaftDb>();
    rna.nacelle = js.at("nacelle").get<NacelleDb>();
    rna.drivetrain = js.at("drivetrain").get<DrivetrainDb>();
    rna.hub = js.at("hub").get<HubDb>();
}

RnaDb read_rna_json(const std::string& filepath) {
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
    wind.type = to_lowercase(js.at("type").get<std::string>());
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
    options.type = to_lowercase(js.at("type").get<std::string>());
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
    options.type = to_lowercase(js.at("type").get<std::string>());
    options.direction = Eigen::Vector3d(js.at("direction")[0], js.at("direction")[1], js.at("direction")[2]);
    options.velocity_surface = js.at("velocity_surface").get<double>();
    options.velocity_seabed = js.at("velocity_seabed").get<double>();
}

void from_json(const json& js, SeaDb& sea) {
    sea.type = to_lowercase(js.at("type").get<std::string>());
    sea.water_density = js.at("water_density").get<double>();
    sea.mean_water_level = js.at("mean_water_level").get<double>();
    sea.water_depth = js.at("water_depth").get<double>();
    if (sea.type == "hydrochrono") {
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
    soil.type = to_lowercase(js.at("type").get<std::string>());
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

EnvironmentDb read_environment_json(const std::string& filepath) {
    EnvironmentDb env_db;
    json json_db = get_json(filepath);
    from_json(json_db, env_db);
    auto main_directory = fs::path(filepath).parent_path();
    if (env_db.wind.type == "inflowwind") {
        auto inflowwind_filepath = main_directory / env_db.wind.options.file_inflowwind;
        env_db.wind.options.file_inflowwind_path = inflowwind_filepath;
        if (!fs::exists(inflowwind_filepath)) {
            throw std::runtime_error("InflowWind file not found: " + inflowwind_filepath.u8string());
        }
    }
    return env_db;
}

void from_json(const json& js, AeroOptionsTurbineDb& options, const std::string& type) {
    if (type == "bemt") {
        options.hub_loss = js.at("hub_loss").get<bool>();
        options.tip_loss = js.at("tip_loss").get<bool>();
        options.tower_shadow = js.at("tower_shadow").get<bool>();
    } else if (type == "disk") {
        options.performance_file = js.at("performance_file").get<std::string>();
    } else if (type == "aerodyn") {
        options.file_aerodyn = js.at("file_aerodyn").get<std::string>();
        options.file_inflowwind = js.at("file_inflowwind").get<std::string>();
    }
}

void from_json(const json& js, AeroTurbineDb& aero) {
    aero.solver = to_lowercase(js.at("solver").get<std::string>());
    from_json(js.at("options"), aero.options, aero.solver);
}

void from_json(const json& js, BladeTurbineDb& blade) {
    blade.file = js.at("file").get<std::string>();
    blade.initial_pitch = js.at("initial_pitch").get<double>();
    blade.precone = js.at("precone").get<double>();
}

void from_json(const json& js, RotorOptionsTurbineDb& options) {
    if (!js.contains("inertia_blades") || js.at("inertia_blades").is_null()) {
        throw std::runtime_error("The \"inertia_blades\" key (rotor options) must be provided for rigid/disk rotors.");
    }
    options.inertia_blades = js.at("inertia_blades").get<double>();
    if (!js.contains("mass_blades") || js.at("mass_blades").is_null()) {
        throw std::runtime_error("The \"mass_blades\" key (rotor options) must be given for rigid/disk rotors.");
    }
    options.mass_blades = js.at("mass_blades").get<double>();
    if (!js.contains("radius") || js.at("radius").is_null()) {
        throw std::runtime_error(
            "The \"radius\" key (rotor options) must be given for rotors using actuator disk theory.");
    }
    options.radius = js.at("radius").get<double>();
}

void from_json(const json& js, DiscretizationRotorTurbineDb& discretization) {
    discretization.elasto = js.at("elasto").get<std::vector<double>>();
    discretization.aero = js.at("aero").get<std::vector<double>>();
}

void from_json(const json& js, RotorTurbineDb& rotor) {
    rotor.type = to_lowercase(js.at("type").get<std::string>());
    if (rotor.type == "disk") {
        rotor.option = js.at("options").get<RotorOptionsTurbineDb>();
    } else {
        rotor.discretization = js.at("discretization").get<DiscretizationRotorTurbineDb>();
        rotor.pitch_actuator_dynamics = js.at("pitch_actuator_dynamics").get<bool>();
        rotor.blades = js.at("blades").get<std::vector<BladeTurbineDb>>();
    }
}

void from_json(const json& js, RNATurbineDb& rna) {
    rna.file = js.at("file").get<std::string>();
    rna.initial_yaw = js.at("initial_yaw").get<double>();
    rna.yaw_actuator_dynamics = js.at("yaw_actuator_dynamics").get<bool>();
}

void from_json(const json& js, TowerOptionsTurbineDb& options) {
    if (js.contains("use_MacCamyFuchs_correction"))
        options.use_MacCamyFuchs_correction = js.at("use_MacCamyFuchs_correction").get<bool>();
    else
        spdlog::warn("MacCamyFuchs correction for tower/pile not defined in turbine.json, it is by default set to {}.",
                     0.0);

    if (js.contains("use_Cd_correction"))
        options.use_Cd_correction = js.at("use_Cd_correction").get<bool>();
    else
        spdlog::warn(
            "Drag Coefficient correction for tower/pile not defined in turbine.json, it is by default set to {}.", 0.0);
}

void from_json(const json& js, DiscretizationTowerTurbineDb& discretization) {
    discretization.elasto = js.at("elasto").get<std::vector<double>>();
    discretization.aero = js.at("aero").get<std::vector<double>>();
}

void from_json(const json& js, TowerTurbineDb& tower) {
    tower.discretization = js.at("discretization").get<DiscretizationTowerTurbineDb>();
    tower.file = js.at("file").get<std::string>();

    if (js.contains("options") && !js["options"].is_null()) {
        tower.options = js.at("options").get<TowerOptionsTurbineDb>();
    } else {
        tower.options = std::nullopt;
    }
}

void from_json(const json& js, ControllerOptionsTurbineDb& options, const std::string& type) {
    if (type == "discon") {
        options.infile = js.at("infile").get<std::string>();
        if (js.contains("libfile") && !js["libfile"].is_null())
            options.libfile = js.at("libfile").get<std::string>();
        else
            throw std::runtime_error("Need to define path to libfile for DISCON routine.");
    } else if (type == "rpm") {
        if (js.contains("target_rpm") && !js["target_rpm"].is_null())
            options.target_rpm = js.at("target_rpm").get<double>();
        else
            throw std::runtime_error("Need to define target RPM for RPM controller (target_rpm).");
    }
}

void from_json(const json& js, ControllerTurbineDb& controller) {
    controller.type = to_lowercase(js.at("type").get<std::string>());
    from_json(js.at("options"), controller.options, controller.type);
}

void from_json(const json& js, DiscretizationFoundationTurbineDb& discretization) {
    discretization.elasto = js.at("elasto").get<std::vector<double>>();
    discretization.hydro = js.at("hydro").get<std::vector<double>>();
}

void from_json(const json& js, FoundationTurbineDb& foundation) {
    foundation.type = to_lowercase(js.at("type").get<std::string>());
    if (js.contains("file") && !js["file"].is_null()) {
        foundation.file = js.at("file").get<std::string>();
    } else {
        foundation.file = std::nullopt;
    }

    if (foundation.type != "floater")
        foundation.discretization = js.at("discretization").get<DiscretizationFoundationTurbineDb>();

    if (js.contains("options") && !js["options"].is_null())
        foundation.options = js.at("options").get<TowerOptionsTurbineDb>();
    else
        foundation.options = std::nullopt;
}

void from_json(const json& js, TurbineDb& turbine) {
    turbine.aero = js.at("aero").get<AeroTurbineDb>();
    turbine.rotor = js.at("rotor").get<RotorTurbineDb>();
    turbine.rna = js.at("rna").get<RNATurbineDb>();
    turbine.tower = js.at("tower").get<TowerTurbineDb>();
    turbine.controller = js.at("controller").get<ControllerTurbineDb>();
    if (js.contains("foundation") && !js["foundation"].is_null()) {
        turbine.foundation = js.at("foundation").get<FoundationTurbineDb>();
    } else {
        turbine.foundation = std::nullopt;
    }
}

TurbineDb read_turbine_json(const std::string& filepath) {
    TurbineDb turbine_db;
    json json_db = get_json(filepath);
    from_json(json_db, turbine_db);

    // Finalize
    auto main_directory = fs::path(filepath).parent_path();
    if (turbine_db.aero.solver == "aerodyn") {
        auto inflowwind_filepath = main_directory / turbine_db.aero.options.file_inflowwind;
        turbine_db.aero.options.file_inflowwind_path = inflowwind_filepath;
        if (!fs::exists(inflowwind_filepath)) {
            throw std::runtime_error("InflowWind file not found: " + inflowwind_filepath.u8string());
        }
        auto aerodyn_filepath = main_directory / turbine_db.aero.options.file_aerodyn;
        turbine_db.aero.options.file_aerodyn_path = aerodyn_filepath;
        if (!fs::exists(aerodyn_filepath)) {
            throw std::runtime_error("AeroDyn file not found: " + aerodyn_filepath.u8string());
        }
    }
    if (turbine_db.aero.solver == "disk") {
        auto performance_filepath = main_directory / turbine_db.aero.options.performance_file;
        turbine_db.aero.options.performance_file_path = performance_filepath;
        if (!fs::exists(performance_filepath)) {
            throw std::runtime_error("Performance file not found: " + performance_filepath.u8string());
        }
    }
    // Finalize the rna database
    auto rna_filepath = main_directory / turbine_db.rna.file;
    turbine_db.rna.data = read_rna_json(rna_filepath.generic_string());
    // Read the rotor database
    for (auto& blade : turbine_db.rotor.blades) {
        auto blade_filepath = main_directory / blade.file;
        blade.data = read_blade_json(blade_filepath.generic_string());
    }
    // Finalize the tower database
    auto tower_filepath = main_directory / turbine_db.tower.file;
    turbine_db.tower.data = read_tower_json(tower_filepath.generic_string());

    if (turbine_db.foundation.has_value()) {
        auto& foundation_db = turbine_db.foundation.value();
        if (foundation_db.file.has_value()) {
            foundation_db.file_path = main_directory / foundation_db.file.value();
            if (foundation_db.type == "floater") {
                foundation_db.data_floater = read_floater_json(foundation_db.file_path.generic_string());
            } else if (foundation_db.type == "monopile") {
                foundation_db.data_tower = read_tower_json(foundation_db.file_path.generic_string());
            }
        }
        if (foundation_db.type == "floater") {
            foundation_db.data_floater.options_file_path = main_directory / foundation_db.data_floater.options_file;
            if (foundation_db.file.has_value()) {
                foundation_db.file_path = main_directory / foundation_db.file.value();
            }
            for (auto& mooring : foundation_db.data_floater.moorings) {
                auto line_properties_filepath = main_directory / mooring.line_properties;
                mooring.properties = read_mooring_properties_json(line_properties_filepath.generic_string());
            }
        }
    }
    // Finalize the controller database
    auto discon_filepath = main_directory / turbine_db.controller.options.infile;
    turbine_db.controller.options.infile_path = discon_filepath;
    if (!fs::exists(discon_filepath)) {
        throw std::runtime_error("DISCON file not found: " + discon_filepath.u8string());
    }
    auto lib_filepath = main_directory / turbine_db.controller.options.libfile;
    turbine_db.controller.options.libfile_path = lib_filepath;
    if (!fs::exists(lib_filepath)) {
        throw std::runtime_error("DISCON library file not found: " + lib_filepath.u8string());
    }

    return turbine_db;
}

void from_json(const json& js, DiscretizationFloaterdb& discretization) {
    discretization.elasto = js.at("elasto").get<std::vector<double>>();
    discretization.hydro = js.at("hydro").get<std::vector<double>>();
}

void from_json(const json& js, MooringFloaterdb& mooring) {
    mooring.connected_body_name = js.at("connected_body_name").get<std::string>();
    mooring.line_properties = js.at("line_properties").get<std::string>();
    mooring.length = js.at("length").get<double>();
    mooring.discretization = js.at("discretization").get<DiscretizationFloaterdb>();
    mooring.relative_fairlead = js.at("relative_fairlead").get<bool>();
    mooring.relative_anchor = js.at("relative_anchor").get<bool>();
    mooring.fairlead_position =
        Eigen::Vector3d(js.at("fairlead_position")[0], js.at("fairlead_position")[1], js.at("fairlead_position")[2]);
    mooring.anchor_position =
        Eigen::Vector3d(js.at("anchor_position")[0], js.at("anchor_position")[1], js.at("anchor_position")[2]);
    mooring.rotation_axis =
        Eigen::Vector3d(js.at("rotation_axis")[0], js.at("rotation_axis")[1], js.at("rotation_axis")[2]);
    mooring.rotation_angle = js.at("rotation_angle").get<double>();
}

void from_json(const json& js, BodyFloaterdb& body) {
    body.name = js.at("name").get<std::string>();
    if (js.at("position").size() != 3) {
        throw std::runtime_error("Position of body should be a vector of length 3.");
    }
    body.position = Eigen::Vector3d(js.at("position")[0], js.at("position")[1], js.at("position")[2]);
    body.mass = js.at("mass").get<double>();

    const auto& inertia = js.at("inertia");
    if (inertia.size() != 3 || inertia[0].size() != 3) {
        throw std::runtime_error("Inertia matrix should be a 3x3 matrix.");
    }
    body.inertia = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < 3; ++i) {
        if (inertia[i].size() != 3) {
            throw std::runtime_error("Inertia matrix of body should be 3x3.");
        }
        for (size_t j = 0; j < 3; ++j) {
            body.inertia(i, j) = inertia[i][j].get<double>();
        }
    }
}

void from_json(const json& js, Floaterdb& floater) {
    floater.type = to_lowercase(js.at("type").get<std::string>());
    floater.options_file = js.at("options").at("file").get<std::string>();
    if (js.at("position").size() != 3) {
        throw std::runtime_error("Position of body should be a vector of length 3.");
    }
    floater.position = Eigen::Vector3d(js.at("position")[0], js.at("position")[1], js.at("position")[2]);
    floater.mass = js.at("mass").get<double>();

    const auto& inertia = js.at("inertia");
    if (inertia.size() != 3 || inertia[0].size() != 3) {
        throw std::runtime_error("Inertia matrix should be a 3x3 matrix.");
    }
    floater.inertia = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < 3; ++i) {
        if (inertia[i].size() != 3) {
            throw std::runtime_error("Inertia matrix of body should be 3x3.");
        }
        for (size_t j = 0; j < 3; ++j) {
            floater.inertia(i, j) = inertia[i][j].get<double>();
        }
    }

    const auto& damping = js.at("damping_matrix");
    if (damping.size() != 6) {
        throw std::runtime_error("Viscous damping matrix for floater body has to be defined as 6x6 matrices.");
    }
    floater.damping_matrix = Eigen::MatrixXd(damping.size(), damping[0].size());
    for (size_t i = 0; i < damping.size(); ++i) {
        if (damping[i].size() != 6) {
            throw std::runtime_error("Viscous damping matrix for floater body has to be defined as 6x6 matrices.");
        }
        for (size_t j = 0; j < damping[i].size(); ++j) {
            floater.damping_matrix(i, j) = damping[i][j].get<double>();
        }
    }

    floater.bodies = js.at("bodies").get<std::vector<BodyFloaterdb>>();
    floater.moorings = js.at("moorings").get<std::vector<MooringFloaterdb>>();
}

Floaterdb read_floater_json(const std::string& filepath) {
    Floaterdb floater_db;
    json json_db = get_json(filepath);
    from_json(json_db, floater_db);
    return floater_db;
}

void from_json(const json& js, MooringPropertiesDb& mooring_props) {
    mooring_props.diameter = js.at("diameter").get<double>();
    mooring_props.stiffness_axial = js.at("stiffness_axial").get<double>();
    mooring_props.stiffness_bending = js.at("stiffness_bending").get<double>();
    mooring_props.density_linear = js.at("density_linear").get<double>();
    mooring_props.drag_coefficient_normal = js.at("drag_coefficient_normal").get<double>();
    mooring_props.drag_coefficient_axial = js.at("drag_coefficient_axial").get<double>();
    mooring_props.added_mass_coefficient_normal = js.at("added_mass_coefficient_normal").get<double>();
    mooring_props.added_mass_coefficient_axial = js.at("added_mass_coefficient_axial").get<double>();
}

MooringPropertiesDb read_mooring_properties_json(const std::string& filepath) {
    MooringPropertiesDb mooring_props;
    json json_db = get_json(filepath);
    from_json(json_db, mooring_props);
    return mooring_props;
}

void from_json(const json& js, StaticsMainDb& statics) {
    statics.linear_step = js.at("linear_step").get<bool>();
    statics.nonlinear_steps = js.at("nonlinear_steps").get<int>();
}

void from_json(const json& js, PresimulationMainDb& presimulation) {
    presimulation.dt = js.at("dt").get<double>();
    presimulation.duration = js.at("duration").get<double>();
    presimulation.presetup = js.at("presetup").get<bool>();
    presimulation.fix_towers = js.at("fix_towers").get<bool>();
}

void from_json(const json& js, NumericsMainDb& numerics) {
    numerics.dt = js.at("dt").get<double>();
    numerics.duration = js.at("duration").get<double>();
    numerics.statics = js.at("statics").get<StaticsMainDb>();
    numerics.presimulation = js.at("presimulation").get<PresimulationMainDb>();
}

void from_json(const json& js, OutputsMainDb& outputs) {
    outputs.dt = js.at("dt").get<double>();
    if (js.contains("folder") && !js["folder"].is_null()) {
        outputs.folder = js.at("folder").get<std::string>();
    } else {
        outputs.folder = std::nullopt;
    }
    outputs.vtk = js.at("vtk").get<bool>();
    outputs.log_level = js.at("log_level").get<std::string>();
    outputs.gui = js.at("gui").get<bool>();
}

void from_json(const json& js, EnvironmentMainDb& environment) {
    environment.file = js.at("file").get<std::string>();
}

void from_json(const json& js, TurbineMainDb& turbine) {
    turbine.file = js.at("file").get<std::string>();
    turbine.translation = Eigen::Vector3d(js.at("translation")[0], js.at("translation")[1], js.at("translation")[2]);
    turbine.rotation = js.at("rotation").get<double>();
}

void from_json(const json& js, MainDb& config) {
    config.numerics = js.at("numerics").get<NumericsMainDb>();
    config.outputs = js.at("outputs").get<OutputsMainDb>();
    config.environment = js.at("environment").get<EnvironmentMainDb>();
    config.turbines = js.at("turbines").get<std::vector<TurbineMainDb>>();
}

MainDb read_main_json(const std::string& filepath) {
    MainDb main_db;
    json json_db = get_json(filepath);
    from_json(json_db, main_db);
    auto main_directory = fs::path(filepath).parent_path();
    for (auto& turbine : main_db.turbines) {
        if (!turbine.file.empty()) {
            auto turbine_filepath = main_directory / turbine.file;
            turbine.data = read_turbine_json(turbine_filepath.generic_string());
        }
    }

    auto env_filepath = main_directory / main_db.environment.file;
    main_db.environment.data = read_environment_json(env_filepath.generic_string());
    return main_db;
}

}  // namespace io
}  // namespace seahowl
