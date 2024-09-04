#include "seahowl/io/read_json.h"
#include "seahowl/io/utils_io.h"
#include "seahowl/io/read_rotor_perf.h"

#include "seahowl/commons/utils.h"
#include "seahowl/core/blade.h"
#include "seahowl/core/rotor.h"
#include "seahowl/core/tower.h"
#include "seahowl/core/turbine.h"
#include "seahowl/core/system.h"
#include "seahowl/elasto/tower_elasto.h"
#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/reference_point_elasto.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/servo/controller_discon.h"
#include "seahowl/commons/numerics.h"
#include "seahowl/env/fluid_models.h"
#include "seahowl/env/wind_models.h"
#include "seahowl/env/wave_models.h"
#include "seahowl/env/soil_models.h"
#include "seahowl/env/combined_models.h"
#ifdef HAVE_INFLOWWIND
    #include "seahowl/env/inflowwind_adapter.h"
#endif
#include "seahowl/aero/airfoil.h"
#include "seahowl/aero/blade_aero.h"
#include "seahowl/aero/rotor_aero.h"
#include "seahowl/aero/turbine_aero.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/hydro/mooring_hydro.h"
#ifdef HAVE_HYDROCHRONO
    #include "seahowl/hydro/hydrochrono_adapter.h"
    #include "seahowl/elasto/chrono_adapters.h"
    #include <hydroc/hydro_forces.h>
#endif
#ifdef HAVE_AERODYN
    #include "seahowl/aero/aerodyn_adapter.h"
#endif

#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <typeinfo>
#include <filesystem>
#include <spdlog/spdlog.h>
namespace fs = std::filesystem;
using std::filesystem::path;
using std::filesystem::absolute;

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using seahowl::Vector3d;
using seahowl::Vector2d;
using seahowl::PI;

json get_json_from_file(const std::string& filepath) {
    check_file_exists(filepath);
    std::ifstream json_file(filepath);
    json json_obj;
    json_file >> json_obj;
    json_file.close();
    return json_obj;
}

class InputData {
  public:
    int nrows = 0;
    virtual void open(const std::string& filepath) = 0;
    virtual double get(const std::string& key, int row) = 0;
};

class InputDataCSV : public InputData {
    std::map<std::string, std::vector<double>> csv_data;

  public:
    void open(const std::string& filepath) override {
        check_file_exists(filepath);
        // open file
        std::ifstream csv_file;
        csv_file.open(filepath);

        // headers
        std::map<int, std::string> csv_headers;
        std::string line, word;
        std::getline(csv_file, line);
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        std::stringstream line_ss(line);
        int idx_header = 0;
        while (std::getline(line_ss, word, ',')) {
            csv_data[word] = {};
            csv_headers[idx_header] = word;
            idx_header += 1;
        }

        // data
        int idx_line = 1;
        while (std::getline(csv_file, line)) {
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
            std::stringstream line_ss(line);
            int idx_word = 0;
            while (std::getline(line_ss, word, ',')) {
                csv_data[csv_headers[idx_word]].push_back(std::stod(word));
                idx_word += 1;
            }
            idx_line += 1;
            if (idx_word != idx_header) {
                throw std::runtime_error("File " + filepath + " at line " + std::to_string(idx_line) +
                                         ": number of columns (" + std::to_string(idx_word) +
                                         ") different from number of headers (" + std::to_string(idx_header) + ").");
            }
        }
        nrows = idx_line - 1;

        csv_file.close();
    }

    double get(const std::string& key, int row) override {
        if (csv_data.count(key) > 0) {
            return csv_data[key][row];
        } else {
            throw std::runtime_error("Header " + key + " does not exist in CSV file.");
        }
    }
};

class InputDataJSON : public InputData {
    json json_data;

  public:
    void open(const std::string& filepath) override {
        check_file_exists(filepath);
        std::ifstream json_file(filepath);
        json_file >> json_data;
        json_file.close();
        nrows = json_data.size();
    }

    double get(const std::string& key, int row) override { return json_data[row].at(key).get<double>(); }
};

std::shared_ptr<InputData> get_input_data(const std::string& filepath) {
    // READ INPUT DATA
    std::shared_ptr<InputData> input_data;
    auto file_extension = filepath.substr(filepath.find_last_of(".") + 1);
    if (file_extension == "csv") {
        input_data = std::make_shared<InputDataCSV>();
    } else if (file_extension == "json") {
        input_data = std::make_shared<InputDataJSON>();
    }
    input_data->open(filepath);

    return input_data;
}

std::vector<seahowl::elasto::BladeReferencePointElasto> get_blade_elasto_reference_points_from_json(
    const std::string& filepath) {
    auto json_obj = get_json_from_file(filepath);

    // EXTRACT INFO
    std::vector<seahowl::elasto::BladeReferencePointElasto> reference_points;

    auto points = json_obj.at("reference_points").get<json>();
    auto damping_coefficients = json_obj.at("damping_coefficients").get<std::vector<double>>();
    if (damping_coefficients.size() != 4) {
        throw std::runtime_error("Damping coefficients of blade has to be vector of length 4.");
    }

    double blade_length = points[points.size() - 1]["coordinates"][2];
    for (size_t ii = 0; ii < points.size(); ii++) {
        auto& point = points[ii];
        auto reference_point = seahowl::elasto::BladeReferencePointElasto();

        auto coords = point.at("coordinates").get<std::vector<double>>();
        if (coords.size() != 3) {
            throw std::runtime_error("Coordinates along blade have to be vectors of length 3.");
        }
        reference_point.fraction = coords[2] / blade_length;
        reference_point.coordinates = Vector3d(coords[0], coords[1], coords[2]);
        if (point.contains("offset_gravity")) {
            auto og = point.at("offsets_gravity").get<std::vector<double>>();
            reference_point.offset_gravity = Vector2d(og[0], og[1]);
        }
        if (point.contains("offset_elastic")) {
            auto oe = point.at("offsets_elastic").get<std::vector<double>>();
            reference_point.offset_elastic = Vector2d(oe[0], oe[1]);
        }

        auto sm = point.at("stiffness_matrix").get<std::vector<std::vector<double>>>();
        auto mm = point.at("mass_matrix").get<std::vector<std::vector<double>>>();
        if (sm.size() != 6 || mm.size() != 6) {
            throw std::runtime_error("Mass and stiffness matrices along blade have to be defined as 6x6 matrices.");
        }
        for (int irow = 0; irow < 6; irow++) {
            if (sm[irow].size() != 6 || mm[irow].size() != 6) {
                throw std::runtime_error("Mass and stiffness matrices along blade have to be defined as 6x6 matrices.");
            }
            for (int icol = 0; icol < 6; icol++) {
                reference_point.mass_matrix(irow, icol) = mm[irow][icol];
                reference_point.stiffness_matrix(irow, icol) = sm[irow][icol];
            }
        }
        reference_point.structural_twist = point.at("twist").get<double>() * PI / 180.0;
        reference_point.damping_coefficients[0] = damping_coefficients[0];
        reference_point.damping_coefficients[1] = damping_coefficients[1];
        reference_point.damping_coefficients[2] = damping_coefficients[2];
        reference_point.damping_coefficients[3] = damping_coefficients[3];

        reference_points.push_back(reference_point);
    }

    return reference_points;
}

std::vector<seahowl::aero::BladeReferencePointAero> get_blade_aero_reference_points_from_json(
    const std::string& filepath) {
    auto json_obj = get_json_from_file(filepath);

    // EXTRACT INFO
    std::vector<seahowl::aero::BladeReferencePointAero> reference_points;

    auto points = json_obj.at("reference_points").get<json>();

    double blade_length = points[points.size() - 1]["coordinates"][2];
    for (size_t ii = 0; ii < points.size(); ii++) {
        auto& point = points[ii];
        auto reference_point = seahowl::aero::BladeReferencePointAero();

        auto coords = point.at("coordinates").get<std::vector<double>>();
        if (coords.size() != 3) {
            throw std::runtime_error("Coordinates along blade have to be vectors of length 3.");
        }
        reference_point.fraction = coords[2] / blade_length;
        reference_point.coordinates = Vector3d(coords[0], coords[1], coords[2]);
        reference_point.structural_twist = point.at("twist").get<double>() * PI / 180.0;
        if (point.contains("offset_aero")) {
            auto oa = point.at("offset_aero").get<std::vector<double>>();
            // remove structural twist (offset aero is expressed with twist already applied)
            auto offset3D = seahowl::AngleAxisd(reference_point.structural_twist, Vector3d(0.0, 0.0, 1.0)) *
                            Vector3d(oa[0], oa[1], 0.0);
            reference_point.offset_aero = Vector2d(offset3D[0], offset3D[1]);
        }

        if (point.contains("chord")) {
            reference_point.chord = point["chord"];
        }

        // populate json object
        if (point.contains("airfoil_file") && !point["airfoil_file"].get<std::string>().empty()) {
            auto main_directory = fs::path(filepath).parent_path();
            auto airfoil_filename = point.at("airfoil_file").get<std::string>();
            auto airfoil_filepath = main_directory / airfoil_filename;

            auto json_airfoil = get_json_from_file(airfoil_filepath.u8string());

            const auto nreynolds = json_airfoil.size();
            for (int jj = 0; jj < nreynolds; jj++) {
                auto airfoil_properties = json_airfoil[jj];
                auto coeffs = airfoil_properties.at("coefficients").get<std::vector<std::vector<double>>>();

                std::vector<seahowl::aero::AirfoilCoefficients> coefficients_list;

                for (int kk = 0; kk < coeffs.size(); kk++) {
                    if (coeffs[kk].size() != 4) {
                        throw std::runtime_error("Airfoil coefficients has to be vectors of length 4.");
                    }
                    seahowl::aero::AirfoilCoefficients coefficients;
                    coefficients.alpha = coeffs[kk][0];
                    coefficients.lift = coeffs[kk][1];
                    coefficients.drag = coeffs[kk][2];
                    coefficients.moment = coeffs[kk][3];
                    coefficients_list.push_back(coefficients);
                }
                seahowl::aero::AirfoilProperties airfoil;
                airfoil_properties.at("reynolds_number").get_to(airfoil.reynolds_number);
                airfoil.coefficients_list = coefficients_list;
                reference_point.airfoil_properties.push_back(airfoil);
            }

            // push only if airfoil file present
            // @todo make it possible to push without airfoil file
            reference_points.push_back(reference_point);
        }
    }

    return reference_points;
}

void populate_blade_elasto_from_json(const std::string& filepath, seahowl::elasto::BladeElasto& blade) {
    blade.reference_points = get_blade_elasto_reference_points_from_json(filepath);
}

void populate_blade_aero_from_json(const std::string& filepath, seahowl::aero::BladeAero& blade) {
    blade.reference_points = get_blade_aero_reference_points_from_json(filepath);
}

void populate_blade_from_json(const std::string& filepath, seahowl::core::Blade& blade) {
    spdlog::debug("Populating blade from " + filepath + " file (absolute: " + absolute(path(filepath)).string() + ").");
    populate_blade_elasto_from_json(filepath, blade.elasto);
    populate_blade_aero_from_json(filepath, blade.aero);
}

std::vector<seahowl::elasto::TowerReferencePointElasto> get_tower_elasto_reference_points_from_json(
    const std::string& filepath) {
    auto input_data = get_input_data(filepath);

    // EXTRACT INFO
    std::vector<seahowl::elasto::TowerReferencePointElasto> reference_points;
    Vector3d pos0 = Vector3d(input_data->get("x", 0), input_data->get("y", 0), input_data->get("z", 0));
    Vector3d pos1 = Vector3d(input_data->get("x", input_data->nrows - 1), input_data->get("y", input_data->nrows - 1),
                             input_data->get("z", input_data->nrows - 1));
    double length = (pos1 - pos0).norm();

    // MAKE TOWER REFERENCE POINTS
    for (int ii = 0; ii < input_data->nrows; ii++) {
        auto reference_point = seahowl::elasto::TowerReferencePointElasto();
        reference_point.coordinates =
            Vector3d(input_data->get("x", ii), input_data->get("y", ii), input_data->get("z", ii));
        reference_point.fraction = (reference_point.coordinates - pos0).norm() / length;

        // general properties
        auto density = input_data->get("density", ii);
        auto young_modulus = input_data->get("young_modulus", ii);
        auto poisson_ratio = input_data->get("poisson_ratio", ii);
        auto shear_modulus = 0.5 * young_modulus / (1.0 + poisson_ratio);

        // point-specific properties
        auto diameter = input_data->get("diameter", ii);
        auto thickness = input_data->get("thickness", ii);

        // geometry info
        auto d1 = diameter;
        auto d2 = diameter - 2.0 * thickness;
        auto area = PI * (pow(d1, 2) - pow(d2, 2)) / 4.0;
        // linear density
        auto density_linear = density * area;
        // stiffnesses
        auto EI = young_modulus * PI * (pow(d1, 4) - pow(d2, 4)) / 64.;  // bending
        auto EA = young_modulus * area;                                  // axial
        auto kt = shear_modulus * PI * (pow(d1, 4) - pow(d2, 4)) / 32.;  // torsion

        // populate reference point
        reference_point.stiffness_foreaft = EI;
        reference_point.stiffness_sideside = EI;
        reference_point.stiffness_axial = EA;
        reference_point.stiffness_torsion = kt;
        reference_point.density = density_linear;
        reference_point.inertia_foreaft = EI / young_modulus * density_linear;
        reference_point.inertia_sideside = EI / young_modulus * density_linear;

        reference_point.damping_coefficients[0] = input_data->get("damping_z", ii);
        reference_point.damping_coefficients[1] = input_data->get("damping_y", ii);
        reference_point.damping_coefficients[2] = input_data->get("damping_x", ii);
        reference_point.damping_coefficients[3] = input_data->get("damping_t", ii);

        reference_points.push_back(reference_point);
    }

    return reference_points;
}

std::vector<seahowl::aero::TowerReferencePointAero> get_tower_aero_reference_points_from_json(
    const std::string& filepath) {
    auto input_data = get_input_data(filepath);

    // EXTRACT INFO
    Vector3d pos0 = Vector3d(input_data->get("x", 0), input_data->get("y", 0), input_data->get("z", 0));
    Vector3d pos1 = Vector3d(input_data->get("x", input_data->nrows - 1), input_data->get("y", input_data->nrows - 1),
                             input_data->get("z", input_data->nrows - 1));
    double length = (pos1 - pos0).norm();

    // MAKE TOWER REFERENCE POINTS
    std::vector<seahowl::aero::TowerReferencePointAero> reference_points;
    for (int ii = 0; ii < input_data->nrows; ii++) {
        auto reference_point = seahowl::aero::TowerReferencePointAero();
        reference_point.coordinates =
            Vector3d(input_data->get("x", ii), input_data->get("y", ii), input_data->get("z", ii));
        reference_point.fraction = (reference_point.coordinates - pos0).norm() / length;

        reference_point.diameter = input_data->get("diameter", ii);
        reference_point.coefficients.drag_normal = input_data->get("drag_coefficient_normal", ii);
        reference_point.coefficients.drag_axial = input_data->get("drag_coefficient_axial", ii);
        reference_point.coefficients.added_mass_normal = input_data->get("added_mass_coefficient_normal", ii);
        reference_point.coefficients.added_mass_axial = input_data->get("added_mass_coefficient_axial", ii);
        reference_point.coefficients.buoyancy_factor = input_data->get("buoyancy_factor", ii);

        reference_points.push_back(reference_point);
    }

    return reference_points;
}

void populate_tower_elasto_from_json(const std::string& filepath, seahowl::elasto::TowerElasto& tower) {
    tower.reference_points = get_tower_elasto_reference_points_from_json(filepath);
    tower.height = tower.reference_points.back().coordinates.z();
    tower.base_height = tower.reference_points.front().coordinates.z();
}

void populate_tower_aero_from_json(const std::string& filepath, seahowl::aero::TowerAero& tower) {
    check_file_exists(filepath);
    tower.reference_points = get_tower_aero_reference_points_from_json(filepath);
}

void populate_tower_from_json(const std::string& filepath, seahowl::core::Tower& tower) {
    spdlog::debug("Populating tower from " + filepath + " file (absolute: " + absolute(path(filepath)).string() + ").");
    populate_tower_elasto_from_json(filepath, tower.elasto);
    populate_tower_aero_from_json(filepath, tower.aero);
}

void populate_rna_elasto_from_json(const std::string& filepath, seahowl::elasto::RotorNacelleAssemblyElasto& rna) {
    auto json_obj = get_json_from_file(filepath);

    // EXTRACT INFO
    //
    // hub
    auto hub = json_obj.at("hub");
    hub.at("CM").get_to(rna.rotor->hub.center_of_mass);
    hub.at("mass").get_to(rna.rotor->hub.mass);
    hub.at("inertia").get_to(rna.rotor->hub.inertia);
    hub.at("overhang").get_to(rna.rotor->hub.overhang);
    hub.at("radius").get_to(rna.rotor->hub.radius);
    // nacelle
    auto nacelle = json_obj.at("nacelle");
    auto cm = nacelle.at("CM").get<std::vector<double>>();
    if (cm.size() != 3) {
        throw std::runtime_error("Center of mass of nacelle has to be vector of length 3.");
    }
    rna.nacelle.center_of_mass = Vector3d(cm[0], cm[1], cm[2]);
    nacelle.at("mass").get_to(rna.nacelle.mass);
    nacelle.at("inertia").get_to(rna.nacelle.inertia);
    nacelle.at("yaw_bearing_mass").get_to(rna.nacelle.yaw_bearing_mass);
    // shaft
    auto shaft = json_obj.at("shaft");
    shaft.at("distance_from_towertop").get_to(rna.shaft.distance_from_towertop);
    shaft.at("tilt").get_to(rna.shaft.tilt);
    // convert to radians
    rna.shaft.tilt *= PI / 180.0;
}

void populate_rna_aero_from_json(const std::string& filepath, seahowl::aero::RotorNacelleAssemblyAero& rna) {
    auto json_obj = get_json_from_file(filepath);

    // EXTRACT INFO
    //
    // hub
    auto hub = json_obj.at("hub");
    hub.at("radius").get_to(rna.rotor->hub_radius);
}

void populate_rna_from_json(const std::string& filepath, seahowl::core::RotorNacelleAssembly& rna) {
    spdlog::debug("Populating RNA from " + filepath + " file (absolute: " + absolute(path(filepath)).string() + ").");
    populate_rna_elasto_from_json(filepath, rna.elasto);
    populate_rna_aero_from_json(filepath, rna.aero);
}

void add_turbine_to_system_from_json(const std::string& filepath,
                                     seahowl::core::System& system_core,
                                     const std::string& output_folder) {
    spdlog::debug("Adding turbine to system from " + filepath + " file.");
    auto json_obj = get_json_from_file(filepath);

    // make turbine aero
    if (json_obj.at("aero").at("solver") == "aerodyn" || json_obj.at("aero").at("solver") == "AeroDyn") {
#ifdef HAVE_AERODYN
        auto turbine_aero = std::make_shared<seahowl::aero::TurbineAeroDyn>();
        system_core.aero.turbines.push_back(turbine_aero);
#endif
    } else {
        auto turbine_aero = std::make_shared<seahowl::aero::TurbineAero>();
        system_core.aero.turbines.push_back(turbine_aero);
    }

    // make turbine elasto
    auto turbine_elasto = std::make_shared<seahowl::elasto::TurbineElasto>();
    system_core.elasto.turbines.push_back(turbine_elasto);

    // add turbine to system
    std::shared_ptr<seahowl::core::Turbine> turbine;
    turbine = std::make_shared<seahowl::core::Turbine>(*system_core.elasto.turbines.back(),
                                                       *system_core.aero.turbines.back());
    system_core.turbines.push_back(turbine);

    // populate turbine
    populate_turbine_from_json(filepath, *turbine, output_folder);
    turbine->build();
}

auto populate_body_from_json(const json& body_json, seahowl::elasto::BodyElasto& body) {
    body.set_mass(body_json.at("mass").get<double>());
    auto body_position = body_json.at("position").get<std::vector<double>>();
    if (body_position.size() != 3) {
        throw std::runtime_error("Position of body should be a vector of length 3.");
    }
    body.set_position(Vector3d(body_position[0], body_position[1], body_position[2]));
    // inertia body
    auto body_inertia = body_json.at("inertia").get<std::vector<std::vector<double>>>();
    if (body_inertia.size() != 3) {
        throw std::runtime_error("Inertia matrix of body should be 3x3.");
    }
    Eigen::Matrix<double, 3, 3> body_inertia_matrix;
    for (int row = 0; row < 3; row++) {
        if (body_inertia[row].size() != 3) {
            throw std::runtime_error("Inertia matrix of body should be 3x3.");
        }
        for (int col = 0; col < 3; col++) {
            body_inertia_matrix(row, col) = body_inertia[row][col];
        }
    }
    body.set_inertia_matrix(body_inertia_matrix);
}

void populate_turbine_from_json(const std::string& filepath,
                                seahowl::core::Turbine& turbine,
                                const std::string& output_folder) {
    spdlog::debug("Populating turbine from " + filepath + " file (absolute: " + absolute(path(filepath)).string() +
                  ").");
    auto json_obj = get_json_from_file(filepath);

    auto DATADIR = path(filepath).parent_path();

    auto rotor_json = json_obj.at("rotor");
    auto tower_json = json_obj.at("tower");
    auto rna_json = json_obj.at("rna");
    auto controller_json = json_obj.at("controller");
    auto aero_json = json_obj.at("aero");

    if (aero_json.at("solver").get<std::string>() == "bemt" || aero_json.at("solver").get<std::string>() == "BEMT") {
        spdlog::info("Aerodynamic model: Blade Element Momentum Theory (BEMT).");
        auto rotor_aero = std::make_shared<seahowl::aero::RotorAeroBEMT>(turbine.aero.tower);
        turbine.rna.aero.rotor = rotor_aero;
        auto aero_options = aero_json.at("options");
        aero_options.at("hub_loss").get_to(rotor_aero->has_hub_loss);
        aero_options.at("tip_loss").get_to(rotor_aero->has_tip_loss);
        aero_options.at("tower_shadow").get_to(rotor_aero->has_tower_shadow);

    } else if (aero_json.at("solver").get<std::string>() == "disk") {
        spdlog::info("Aerodynamic model: Actuator Disk Theory.");
        if (rotor_json.at("type").get<std::string>() != "disk") {
            throw std::runtime_error("Only rotor type \"disk\" can be used with aero solver \"disk\".");
        }
        auto rotor_disk = std::make_shared<seahowl::aero::RotorAeroDisk>();
        turbine.rna.aero.rotor = rotor_disk;
        auto aero_options = aero_json.at("options");
        // get rotor performance from table
        auto perf_filepath = (DATADIR / aero_options.at("performance_file").get<std::string>()).generic_string();
        get_disk_perf_from_table(perf_filepath, *rotor_disk);
    } else if (aero_json.at("solver").get<std::string>() == "aerodyn" ||
               aero_json.at("solver").get<std::string>() == "AeroDyn") {
        spdlog::info("Aerodynamic model: AeroDyn.");
#ifdef HAVE_AERODYN
        // check that right turbine type was defined for AeroDyn
        try {
            auto& turbine_aero = dynamic_cast<seahowl::aero::TurbineAeroDyn&>(turbine.aero);
        } catch (const std::exception& e) {
            throw std::runtime_error("Wrong aero turbine type to use AeroDyn solver (needs to be TurbineAeroDyn).");
        }
        auto& turbine_aero = dynamic_cast<seahowl::aero::TurbineAeroDyn&>(turbine.aero);
        auto aero_options = aero_json.at("options");
        std::string inflowwind_filepath;
        std::string aerodyn_filepath;
        if (aero_options.contains("file_aerodyn")) {
            aerodyn_filepath = (DATADIR / aero_options.at("file_aerodyn").get<std::string>()).generic_string();
        } else {
            throw std::runtime_error("Turbine set to use aerodyn but AeroDyn file path not defined.");
        }
        if (aero_options.contains("file_inflowwind")) {
            inflowwind_filepath = (DATADIR / aero_options.at("file_inflowwind").get<std::string>()).generic_string();
        } else {
            throw std::runtime_error("Turbine set to use aerodyn but InflowWind file not defined.");
        }
        turbine_aero.aerodyn.set_infiles(aerodyn_filepath, inflowwind_filepath);
        turbine_aero.rna.rotor = std::make_shared<seahowl::aero::RotorAeroDyn>(turbine.aero.tower);
#else
        throw std::runtime_error("Trying to use AeroDyn for turbine but the code was not compiled for using AeroDyn.");
#endif
    } else {
        throw std::runtime_error("Unknown aero solver \"" + aero_json.at("solver").get<std::string>() + "\".");
    }

    // check rotor type
    if (rotor_json.at("type") == "fea") {
        auto rotor_options = rotor_json.at("options");
        if (rotor_options.at("fpm") == true) {
            spdlog::info("Rotor type: finite element blades (FPM).");
        } else {
            spdlog::info("Rotor type: finite element blades.");
        }
    } else if (rotor_json.at("type") == "rigid") {
        spdlog::info("Rotor type: rigid.");
    } else if (rotor_json.at("type") == "disk") {
        spdlog::info("Rotor type: disk");
    } else {
        throw std::runtime_error("Rotor type does not exist: try \"fea\" or \"rigid\" or \"disk\".");
    }

    // blades
    // only make blades if rotor type is not disk
    if (rotor_json.at("type").get<std::string>() != "disk") {
        std::vector<std::shared_ptr<seahowl::core::Blade>> blades;
        std::vector<std::shared_ptr<seahowl::elasto::BladeElasto>> blades_elasto;
        std::vector<std::shared_ptr<seahowl::aero::BladeAero>> blades_aero;
        auto blades_json = rotor_json.at("blades");
        for (auto& blade_json : blades_json) {
            auto filepath_blade = (DATADIR / blade_json.at("file").get<std::string>()).generic_string();
            std::shared_ptr<seahowl::elasto::BladeElasto> blade_elasto;
            if (rotor_json.at("type").get<std::string>() == "fea") {
                blade_elasto = std::make_shared<seahowl::elasto::BladeElastoFEA>();
                auto& blade_elasto_fea = dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade_elasto);
                rotor_json.at("discretization").at("elasto").get_to(blade_elasto_fea.discretization_fractions);
                auto rotor_options = rotor_json.at("options");
                rotor_options.at("fpm").get_to(blade_elasto_fea.fpm_mode);
            } else if (rotor_json.at("type").get<std::string>() == "rigid") {
                blade_elasto = std::make_shared<seahowl::elasto::BladeElastoRigid>();
            }
            auto blade_aero = std::make_shared<seahowl::aero::BladeAero>();
            auto blade = std::make_shared<seahowl::core::Blade>(*blade_elasto, *blade_aero);
            populate_blade_from_json(filepath_blade, *blade);
            rotor_json.at("discretization").at("aero").get_to(blade->aero.discretization_fractions);
            blade_json.at("initial_pitch").get_to(blade->elasto.pitch);
            blade_elasto->precone = blade_json.at("precone").get<double>() * PI / 180.0;
            // no precone if blade is rigid (assumed that blade is on rotor disc)
            if (rotor_json.at("type").get<std::string>() == "rigid") {
                blade_elasto->precone = 0.0;
            }
            blades_elasto.push_back(blade_elasto);
            blades_aero.push_back(blade_aero);
            blades.push_back(blade);
        }
        turbine.elasto.rna.rotor->blades = blades_elasto;
        turbine.aero.rna.rotor->blades = blades_aero;
        turbine.rna.blades = blades;
    }

    // RNA
    auto filepath_rna = (DATADIR / rna_json.at("file").get<std::string>()).generic_string();
    populate_rna_from_json(filepath_rna, turbine.rna);
    rna_json.at("initial_pitch_collective").get_to(turbine.elasto.rna.rotor->pitch_collective);

    // update info if rotor is disk
    if (rotor_json.at("type").get<std::string>() == "disk") {
        auto rotor_options = rotor_json.at("options");
        if (!rotor_options.contains("inertia_blades")) {
            throw std::runtime_error(
                "The \"inertia_blades\" key (rotor options) must be provided for rigid/disk rotors.");
        }
        if (!rotor_options.contains("mass_blades")) {
            throw std::runtime_error("The \"mass_blades\" key (rotor options) must be given for rigid/disk rotors.");
        }
        auto blades_inertia = rotor_options.at("inertia_blades").get<double>();
        turbine.rna.elasto.rotor->hub.inertia += blades_inertia;
        auto blades_mass = rotor_options.at("mass_blades").get<double>();
        turbine.rna.elasto.rotor->hub.mass += blades_mass;
        if (!rotor_options.contains("radius")) {
            throw std::runtime_error(
                "The \"radius\" key (rotor options) must be given for rotors using actuator disk theory.");
        } else {
            rotor_options.at("radius").get_to(turbine.rna.aero.rotor->radius);
        }
    }

    // tower
    auto filepath_tower = (DATADIR / tower_json.at("file").get<std::string>()).generic_string();
    populate_tower_from_json(filepath_tower, turbine.tower);
    tower_json.at("discretization").at("elasto").get_to(turbine.elasto.tower.discretization_fractions);
    tower_json.at("discretization").at("aero").get_to(turbine.aero.tower.discretization_fractions);

    // controller
    if (controller_json.at("type").get<std::string>() == "DISCON") {
        if (!controller_json.at("options").contains("libfile")) {
            throw std::runtime_error("Need to define path to libfile for DISCON routine.");
        }

        auto libfilepath = controller_json.at("options").at("libfile").get<std::string>();
        if (libfilepath != "") {
            // path
            libfilepath = path(DATADIR / libfilepath).generic_string();
            // copy libdiscon to temporary folder with new name in case there are several turbines
            check_file_exists(libfilepath);
            // first copy libdiscon to tmp folder
            libfilepath = copy_file_and_increment(libfilepath, output_folder + "/tmp");
        }
        auto infilepath = controller_json.at("options").at("infile").get<std::string>();
        if (infilepath != "") {
            infilepath = (DATADIR / infilepath).generic_string();
        }
        // instantiate controller
        auto controller = std::make_shared<seahowl::servo::ControllerDISCON>(infilepath, libfilepath);
        turbine.controller = controller;

    } else if (controller_json.at("type").get<std::string>() == "RPM") {
        if (!controller_json.at("options").contains("target_rpm")) {
            throw std::runtime_error("Need to define target RPM for RPM controller (target_rpm).");
        } else {
            auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
            controller_json.at("options").at("target_rpm").get_to(controller->target_rpm);
            turbine.controller = controller;
        }
    }

    // get extra drivetrain info
    auto json_obj2 = get_json_from_file(filepath_rna);
    auto drivetrain = json_obj2.at("drivetrain");
    // gearbox
    drivetrain.at("gearbox_ratio").get_to(turbine.gearbox_ratio);
    drivetrain.at("gearbox_efficiency").get_to(turbine.gearbox_efficiency);
    turbine.gearbox_efficiency /= 100.0;
    // generator
    drivetrain.at("generator_efficiency").get_to(turbine.generator_efficiency);
    turbine.generator_efficiency /= 100.0;
    // add inertia of generator to hub directly
    double drivetrain_inertia;
    drivetrain.at("generator_inertia").get_to(drivetrain_inertia);
    turbine.rna.elasto.rotor->hub.inertia += drivetrain_inertia;

    if (json_obj.contains("floater")) {
        auto& turbine_elasto = turbine.elasto;

        auto floater_json = json_obj.at("floater");

        auto floater_hydro_ptr = std::make_shared<seahowl::hydro::FloaterHydro>();
        turbine.aero.foundation = floater_hydro_ptr;
        auto& floater_hydro = *floater_hydro_ptr;

        if (floater_json.contains("file")) {
            auto json_obj_floater =
                get_json_from_file((DATADIR / floater_json.at("file").get<std::string>()).generic_string());

            auto floater_type = json_obj_floater.at("type").get<std::string>();
            if (floater_type == "HydroChrono") {
                spdlog::info("Hydrodynamic model: HydroChrono.");
#ifdef HAVE_HYDROCHRONO
                // make floater
                auto floater_elasto_ptr = std::make_shared<seahowl::hydro::FloaterHydroChrono>();
                turbine_elasto.foundation = floater_elasto_ptr;
                // add h5file path
                auto floater_options = json_obj_floater.at("options");
                floater_elasto_ptr->set_h5_filepath(
                    (DATADIR / floater_options.at("file").get<std::string>()).generic_string());
#else
                throw std::runtime_error("Trying to use HydroChrono but did not compile with HydroChrono dependency.");
#endif
            } else {
                auto floater_elasto_ptr = std::make_shared<seahowl::elasto::FloaterElasto>();
                turbine_elasto.foundation = floater_elasto_ptr;
            }

            auto& floater_elasto = dynamic_cast<seahowl::elasto::FloaterElasto&>(*turbine_elasto.foundation);
            // get main body info
            auto& body = *floater_elasto.body_main;
            populate_body_from_json(json_obj_floater, body);
            // get other bodies info
            auto bodies_json = json_obj_floater.at("bodies");
            for (auto& body_json : bodies_json) {
                auto body_name = body_json.at("name").get<std::string>();
                floater_elasto.add_body(body_name);
                populate_body_from_json(body_json, floater_elasto.get_body(body_name));
            }

            auto dm = json_obj_floater.at("damping_matrix").get<std::vector<std::vector<double>>>();
            if (dm.size() != 6) {
                throw std::runtime_error("Viscous damping matrix for floater body has to be defined as 6x6 matrices.");
            }
            for (int irow = 0; irow < 6; irow++) {
                if (dm[irow].size() != 6) {
                    throw std::runtime_error(
                        "Viscous damping matrix for floater body has to be defined as 6x6 matrices.");
                }
                for (int icol = 0; icol < 6; icol++) {
                    floater_elasto.damping_matrix(irow, icol) = dm[irow][icol];
                }
            }

            // core floater
            auto floater_core_ptr = std::make_shared<seahowl::core::Floater>(floater_elasto, floater_hydro);
            turbine.foundation = floater_core_ptr;
            auto& floater_core = *floater_core_ptr;

            auto moorings_json = json_obj_floater.at("moorings");
            for (auto& mooring_json : moorings_json) {
                auto axis = mooring_json.at("rotation_axis").get<std::vector<double>>();
                auto rotation_axis = seahowl::Vector3d(axis[0], axis[1], axis[2]);
                auto rotation_angle = mooring_json.at("rotation_angle").get<double>() * seahowl::PI / 180.0;
                auto rotation = seahowl::AngleAxisd(rotation_angle, rotation_axis);

                // fairlead
                auto body_name = mooring_json.at("connected_body_name").get<std::string>();
                auto fpos = mooring_json.at("fairlead_position").get<std::vector<double>>();
                auto fairlead_relative_position = seahowl::Vector3d(fpos[0], fpos[1], fpos[2]);
                seahowl::Vector3d fairlead_position = rotation * fairlead_relative_position;
                if (mooring_json.at("relative_fairlead").get<bool>()) {
                    fairlead_position += floater_elasto.body_main->get_position();
                }
                floater_elasto.add_fairlead(fairlead_position, body_name);
                auto& fairlead_body =
                    floater_elasto.get_fairlead_body(body_name, floater_elasto.get_fairlead_count(body_name) - 1);

                // anchor
                auto apos = mooring_json.at("anchor_position").get<std::vector<double>>();
                auto anchor_relative_position = seahowl::Vector3d(apos[0], apos[1], apos[2]);
                seahowl::Vector3d anchor_position = rotation * anchor_relative_position;
                if (mooring_json.at("relative_anchor").get<bool>()) {
                    anchor_position += floater_elasto.body_main->get_position();
                }
                floater_elasto.mooring_system->anchors.push_back(std::make_shared<seahowl::elasto::BodyElastoChrono>());
                auto& anchor_body = *floater_elasto.mooring_system->anchors.back();
                anchor_body.set_position(anchor_position);
                anchor_body.set_fixed(true);

                auto mooring_properties_json = get_json_from_file(
                    (DATADIR / mooring_json.at("line_properties").get<std::string>()).generic_string());

                // elasto
                floater_elasto.mooring_system->moorings.push_back(
                    std::make_shared<seahowl::elasto::MooringElastoFEA>(fairlead_body, anchor_body));
                auto& mooring_elasto =
                    dynamic_cast<seahowl::elasto::MooringElastoFEA&>(*floater_elasto.mooring_system->moorings.back());
                mooring_json.at("length").get_to(mooring_elasto.length);
                mooring_properties_json.at("diameter").get_to(mooring_elasto.diameter);
                mooring_json.at("discretization_elasto").get_to(mooring_elasto.discretization_fractions);
                mooring_properties_json.at("stiffness_axial").get_to(mooring_elasto.stiffness_axial);
                mooring_properties_json.at("stiffness_bending").get_to(mooring_elasto.stiffness_bending);
                mooring_properties_json.at("density_linear").get_to(mooring_elasto.density_linear);

                // hydro
                floater_hydro.mooring_system->moorings.push_back(std::make_shared<seahowl::hydro::MooringHydro>());
                auto& mooring_hydro = *floater_hydro.mooring_system->moorings.back();
                mooring_json.at("length").get_to(mooring_hydro.length);
                mooring_properties_json.at("diameter").get_to(mooring_hydro.diameter);
                mooring_json.at("discretization_hydro").get_to(mooring_hydro.discretization_fractions);
                mooring_properties_json.at("drag_coefficient_normal").get_to(mooring_hydro.coefficients.drag_normal);
                mooring_properties_json.at("drag_coefficient_axial").get_to(mooring_hydro.coefficients.drag_axial);
                mooring_properties_json.at("added_mass_coefficient_normal")
                    .get_to(mooring_hydro.coefficients.added_mass_normal);
                mooring_properties_json.at("added_mass_coefficient_axial")
                    .get_to(mooring_hydro.coefficients.added_mass_axial);

                floater_core.mooring_system->moorings.push_back(
                    std::make_shared<seahowl::core::Mooring>(mooring_elasto, mooring_hydro));
            }
        } else {
            spdlog::warn("Turbine has floater key but no floater file was defined.");

            auto floater_elasto_ptr = std::make_shared<seahowl::elasto::FloaterElasto>();
            turbine_elasto.foundation = floater_elasto_ptr;
            auto& floater_elasto = *floater_elasto_ptr;
            auto floater_core = std::make_shared<seahowl::core::Floater>(floater_elasto, floater_hydro);
            turbine.foundation = floater_core;
        }
    }
}

std::shared_ptr<seahowl::env::FluidSoilModel> get_environmental_model_from_json(const std::string& filepath) {
    spdlog::debug("Getting environmental conditions from " + filepath + " file.");
    auto environment_json = get_json_from_file(filepath);

    auto DATADIR = path(filepath).parent_path();
    // gravity
    auto gravity = environment_json.at("gravity").get<std::vector<double>>();
    auto gravity_vector = seahowl::Vector3d(gravity[0], gravity[1], gravity[2]);
    auto gravity_direction = gravity_vector / gravity_vector.norm();

    // environment
    auto env_model = std::make_shared<seahowl::env::FluidSoilModel>();

    // sea
    bool has_sea = false;
    if (environment_json.contains("sea")) {
        has_sea = true;
        auto sea_json = environment_json.at("sea");
        env_model->fluid_model = std::make_shared<seahowl::env::WaveWindModel>();
        auto& fluid_model = dynamic_cast<seahowl::env::WaveWindModel&>(*env_model->fluid_model);

        // specific model options
        auto sea_type = sea_json.at("type").get<std::string>();
        if (sea_type == "still") {
            spdlog::info("Sea conditions: still water.");
            fluid_model.wave_model = std::make_unique<seahowl::env::StillWater>();
        } else if (sea_type == "HydroChrono" || sea_type == "hydrochrono") {
            spdlog::info("Sea conditions: HydroChrono.");
#ifdef HAVE_HYDROCHRONO
            fluid_model.wave_model = std::make_unique<seahowl::env::WaveModelHydroChrono>();
            auto& wave_model = dynamic_cast<seahowl::env::WaveModelHydroChrono&>(*fluid_model.wave_model);
            auto sea_options = sea_json.at("options");
            auto wave_type = sea_options.at("type").get<std::string>();
            if (wave_type == "still") {
                wave_model.waves = std::make_shared<NoWave>();
            } else if (wave_type == "regular") {
                auto hydrochrono_waves = std::make_shared<RegularWave>();
                wave_model.waves = hydrochrono_waves;
                hydrochrono_waves->regular_wave_amplitude_ = sea_options.at("wave_height").get<double>() / 2.0;
                hydrochrono_waves->regular_wave_omega_ = 2 * PI / sea_options.at("wave_period").get<double>();
            } else if (wave_type == "irregular") {
                auto params = IrregularWaveParams();
                sea_options.at("num_bodies").get_to(params.num_bodies_);
                sea_options.at("wave_height").get_to(params.wave_height_);
                sea_options.at("wave_period").get_to(params.wave_period_);
                sea_options.at("frequency_min").get_to(params.frequency_min_);
                sea_options.at("frequency_max").get_to(params.frequency_max_);
                sea_options.at("nfrequencies").get_to(params.nfrequencies_);
                sea_options.at("wave_period").get_to(params.wave_period_);
                sea_options.at("peak_enhancement_factor").get_to(params.peak_enhancement_factor_);
                sea_options.at("is_normalized").get_to(params.is_normalized_);
                sea_options.at("seed").get_to(params.seed_);
                sea_options.at("dt").get_to(params.simulation_dt_);
                sea_options.at("duration").get_to(params.simulation_duration_);
                sea_options.at("wave_stretching").get_to(params.wave_stretching_);
                params.ramp_duration_ = 0.0;
                params.num_bodies_ = 1;
                wave_model.waves = std::make_shared<IrregularWaves>(params);
            } else {
                throw std::runtime_error("Unrecognized wave type \"" + wave_type + "\" for HydroChrono.");
            }
            sea_json.at("mean_water_level").get_to(wave_model.waves->mwl_);
            sea_json.at("water_depth").get_to(wave_model.waves->water_depth_);
            wave_model.waves->g_ = gravity_vector.norm();
            if (wave_type == "irregular") {
                dynamic_cast<IrregularWaves&>(*wave_model.waves).CreateSpectrum();
            }
            if (wave_type == "regular") {
                dynamic_cast<RegularWave&>(*wave_model.waves).Initialize();
            }
#else
            throw std::runtime_error("Must compile and enable HydroChrono dependency to use HydroChrono waves.");
#endif
        } else if (sea_type == "current") {
            spdlog::info("Sea conditions: current.");
            fluid_model.wave_model = std::make_unique<seahowl::env::CurrentConstant>();
            auto& wave_model = dynamic_cast<seahowl::env::CurrentConstant&>(*fluid_model.wave_model);
            auto sea_options = sea_json.at("options");
            auto sea_direction = sea_options.at("direction").get<std::vector<double>>();
            wave_model.direction = Vector3d(sea_direction[0], sea_direction[1], 0.0);
            sea_options.at("velocity_surface").get_to(wave_model.velocity_surface);
            sea_options.at("velocity_seabed").get_to(wave_model.velocity_seabed);
        } else {
            throw std::runtime_error(
                "The input sea type is unknown. Please use the existing current types: still, current, HydroChrono.");
        }

        // general wave model options
        auto& wave_model = dynamic_cast<seahowl::env::WaveModel&>(*fluid_model.wave_model);
        sea_json.at("water_density").get_to(wave_model.density);
        sea_json.at("mean_water_level").get_to(wave_model.mean_water_level);
        sea_json.at("water_depth").get_to(wave_model.water_depth);
        wave_model.surface_normal = -gravity_direction;
    } else {
        spdlog::warn("Sea conditions were not defined.");
    }

    // wind
    auto wind_json = environment_json.at("wind");
    std::shared_ptr<seahowl::env::WindModel> wind_model_ptr;
    if (wind_json.at("type").get<std::string>() == "ramp") {
        spdlog::info("Wind conditions: wind ramp.");
        wind_model_ptr = std::make_shared<seahowl::env::WindRamp>();
        auto& wind_model = dynamic_cast<seahowl::env::WindRamp&>(*wind_model_ptr);
        auto wind_options = wind_json.at("options");
        auto v0 = wind_options.at("velocity_start").get<std::vector<double>>();
        auto v1 = wind_options.at("velocity_end").get<std::vector<double>>();
        wind_model.set_wind_ramp(Vector3d(v0[0], v0[1], v0[2]), wind_options.at("time_start").get<double>(),
                                 Vector3d(v1[0], v1[1], v1[2]), wind_options.at("time_end").get<double>());
        wind_model.direction_gravity = gravity_vector.normalized();
        wind_model.reference_height = wind_options.at("reference_height").get<double>();
        wind_model.shear_coefficient = wind_options.at("shear_coefficient").get<double>();
        wind_model.density = wind_json.at("air_density").get<double>();
    } else if (wind_json.at("type").get<std::string>() == "inflowwind") {
        spdlog::info("Wind conditions: InflowWind.");
#ifdef HAVE_INFLOWWIND
        std::string inflowwind_filepath;
        auto wind_options = wind_json.at("options");
        if (wind_options.contains("file_inflowwind")) {
            inflowwind_filepath = (DATADIR / wind_options.at("file_inflowwind")).generic_string();
        } else {
            throw std::runtime_error("InflowWind file not defined.");
        }
        wind_model_ptr = std::make_shared<seahowl::env::InflowWindAdapter>(inflowwind_filepath);
#else
        throw std::runtime_error(
            "InflowWind module in CMAKE options should be enabled if wind type 'inflowwind' selected.");
#endif
    } else {
        throw std::runtime_error(
            "The input wind type is unknown. Please use the existing wind types: ramp or inflowwind.");
    }
    if (has_sea) {
        auto& fluid_model = dynamic_cast<seahowl::env::WaveWindModel&>(*env_model->fluid_model);
        fluid_model.wind_model = std::move(wind_model_ptr);
    } else {
        env_model->fluid_model = std::move(wind_model_ptr);
    }

    // soil
    if (environment_json.contains("soil")) {
        auto soil_json = environment_json.at("soil");
        if (soil_json.at("type") == "linear") {
            spdlog::info("Soil conditions: linear.");
            auto soil_model_shared = std::make_shared<seahowl::env::LinearSoilModel>();
            auto& soil_model = *soil_model_shared;
            auto soil_options = soil_json.at("options");
            soil_options.at("soil_position").get_to(soil_model.soil_position);
            soil_options.at("stiffness_normal").get_to(soil_model.stiffness_normal);
            soil_options.at("stiffness_shear").get_to(soil_model.stiffness_shear);
            soil_model.soil_normal = -gravity_direction;
            env_model->soil_model = std::move(soil_model_shared);
        } else {
            throw std::runtime_error("The input soil type is unknown. Please use the existing soil types: linear.");
        }
    } else {
        spdlog::warn("Soil conditions were not defined.");
    }

    return env_model;
}

void populate_environmental_conditions_from_json(const std::string& filepath, seahowl::core::System& system_core) {
    spdlog::debug("Populating environmental conditions from " + filepath +
                  " file (absolute: " + absolute(path(filepath)).string() + ").");
    auto environment_json = get_json_from_file(filepath);

    auto DATADIR = path(filepath).parent_path();

    // gravity
    auto gravity = environment_json.at("gravity").get<std::vector<double>>();
    system_core.elasto.set_gravitational_acceleration(Vector3d(gravity[0], gravity[1], gravity[2]));

    auto env_model = get_environmental_model_from_json(filepath);

    if (env_model->fluid_model) {
        system_core.fluid_model = env_model->fluid_model;
    }
    if (env_model->soil_model) {
        system_core.soil_model = env_model->soil_model;
    }

    // environment
#ifdef HAVE_HYDROCHRONO
    if (environment_json.contains("sea")) {
        auto sea_json = environment_json.at("sea");
        auto sea_type = sea_json.at("type").get<std::string>();
        if (sea_type == "HydroChrono" || sea_type == "hydrochrono") {
            for (auto turbine : system_core.turbines) {
                if (turbine->elasto.foundation) {
                    try {
                        auto& floater = dynamic_cast<seahowl::hydro::FloaterHydroChrono&>(*turbine->elasto.foundation);
                        try {
                            auto& fluid_model = dynamic_cast<seahowl::env::WaveWindModel&>(*system_core.fluid_model);
                            auto& waves_model =
                                dynamic_cast<seahowl::env::WaveModelHydroChrono&>(*fluid_model.wave_model);
                            floater.set_waves_hydrochrono(waves_model.waves);
                        } catch (const std::bad_cast& e) {
                            throw std::runtime_error("Must use HydroChrono wave model when using HydroChrono floater.");
                        }
                    } catch (const std::bad_cast& e) {
                    }
                }
            }
        }
    }
#endif
}

void populate_system_from_json(const std::string& filepath, seahowl::core::System& system_core) {
    auto DATADIR = path(filepath).parent_path();

    auto json_obj = get_json_from_file(filepath);

    // outputs
    auto outputs_json = json_obj.at("outputs");
    std::string output_folder = "./output";
    if (outputs_json.contains("folder")) {
        output_folder = outputs_json.at("folder").get<std::string>();
    }
    // logging
    auto log_level = outputs_json.at("log_level").get<std::string>();
    seahowl::set_log_level_global(log_level);
    spdlog::debug("Populating system from " + filepath + " file (absolute: " + absolute(path(filepath)).string() +
                  ").");

    // environmental info
    auto filepath_environment = (DATADIR / json_obj.at("environment").at("file").get<std::string>()).generic_string();
    populate_environmental_conditions_from_json(filepath_environment, system_core);

    // turbines
    auto turbines_json = json_obj.at("turbines");
    for (int ii = 0; ii < turbines_json.size(); ii++) {
        // add turbine to system
        auto turbine_json = turbines_json[ii];
        auto filepath_turbine = (DATADIR / turbine_json.at("file").get<std::string>()).generic_string();
        add_turbine_to_system_from_json(filepath_turbine, system_core, output_folder);

        // get ref to turbine added last
        auto& turbine = *system_core.turbines.back();

#ifdef HAVE_HYDROCHRONO
        if (turbine.elasto.foundation) {
            try {
                auto& floater = dynamic_cast<seahowl::hydro::FloaterHydroChrono&>(*turbine.elasto.foundation);
                try {
                    auto& fluid_model = dynamic_cast<seahowl::env::WaveWindModel&>(*system_core.fluid_model);
                    auto& waves_model = dynamic_cast<seahowl::env::WaveModelHydroChrono&>(*fluid_model.wave_model);
                    floater.set_waves_hydrochrono(waves_model.waves);
                } catch (const std::bad_cast& e) {
                    throw std::runtime_error("Must use HydroChrono wave model when using HydroChrono floater.");
                }
            } catch (const std::bad_cast& e) {
            }
        }
#endif

#ifdef HAVE_AERODYN
        try {
            // set VTK options if using AeroDyn
            auto& turbine_aero = dynamic_cast<seahowl::aero::TurbineAeroDyn&>(turbine.aero);
            if (outputs_json.at("VTK").get<bool>()) {
                turbine_aero.WrVTK = 2;
                turbine_aero.WrVTK_dt = outputs_json.at("dt").get<double>();
            }
        } catch (const std::bad_cast& e) {
            // do nothing if not using AeroDyn
        }
#endif

        // rotate turbine to align tower with gravity vector
        auto v1 = Vector3d(-system_core.elasto.get_gravitational_acceleration()).normalized();
        auto v2 = (turbine.tower.elasto.nodes[1]->get_position() - turbine.tower.elasto.nodes[0]->get_position())
                      .normalized();
        auto rot_axis = v2.cross(v1);
        auto rot_angle = acos(v1.dot(v2));
        turbine.elasto.rotate(rot_angle, rot_axis);
        // rotation around axis opposite to gravity (yaw)
        turbine.elasto.rotate(turbine_json.at("rotation").get<double>() * PI / 180.0,
                              Vector3d(-system_core.elasto.get_gravitational_acceleration()).normalized());
        // translate turbine
        auto trans = turbine_json.at("translation").get<std::vector<double>>();
        turbine.elasto.translate(Vector3d(trans[0], trans[1], trans[2]));

        if (turbine.elasto.foundation) {
            // if has foundation: do not fix tower
            turbine.elasto.tower.nodes.front()->set_fixed(false);
        } else {
            turbine.tower.elasto.nodes.front()->set_fixed(true);
        }
    }
}

void initialize_system_from_json(const std::string& filepath, seahowl::core::System& system_core) {
    auto json_obj = get_json_from_file(filepath);

    // assemble system if it was not already
    if (!system_core.elasto.is_assembled) {
        system_core.elasto.assemble();
    }

    // NUMERICS options
    auto num_json = json_obj.at("numerics");
    // intialization
    auto statics_json = num_json.at("statics");
    // timestepping
    double dt = num_json.at("dt").get<double>();

    // initialization
    // statics
    auto linear_step = statics_json.at("linear_step").get<bool>();
    auto nonlinear_steps = statics_json.at("nonlinear_steps").get<int>();
    if (linear_step && nonlinear_steps > 0) {
        system_core.elasto.do_statics(linear_step, nonlinear_steps);
    }

    system_core.initialize(system_core.get_time(), dt);

    // apply presetup
    if (num_json.contains("presetup")) {
        auto presetup_json = num_json.at("presetup");
        auto presetup_duration = presetup_json.at("duration").get<double>();
        auto presetup_dt = presetup_json.at("dt").get<double>();
        system_core.run_presimulation(presetup_duration, presetup_dt, true, true);
    } else {
        spdlog::debug("No presetup defined in json.");
    }

    if (num_json.contains("presim")) {
        auto presim_json = num_json.at("presim");
        auto presim_duration = presim_json.at("duration").get<double>();
        auto presim_dt = presim_json.at("dt").get<double>();
        auto fix_towers = presim_json.at("fix_towers").get<bool>();
        system_core.run_presimulation(presim_duration, presim_dt, true, false);
    } else {
        spdlog::debug("No presim defined in json.");
    }
}
