#include "seahowl/io/read_json.h"

#include <seahowl/core/utils.h>
#include <seahowl/core/blade.h>
#include <seahowl/core/rotor.h>
#include <seahowl/core/tower.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/core/turbine.h>
#include <seahowl/elasto/blade_elasto.h>

#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;

#include <nlohmann/json.hpp>
using json = nlohmann::json;

std::vector<seahowl::core::BladeReferencePoint> get_blade_reference_points_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    // EXTRACT INFO
    std::vector<seahowl::core::BladeReferencePoint> reference_points;

    auto points = json_obj.at("reference_points").get<json>();
    auto damping_coefficients = json_obj.at("damping_coefficients").get<std::vector<double>>();
    if (damping_coefficients.size() != 4) {
        throw std::runtime_error("Damping coefficients has to be vector of length 4.");
    }

    double blade_length = points[points.size() - 1]["coordinates"][2];
    for (size_t ii = 0; ii < points.size(); ii++) {
        auto& point = points[ii];
        auto reference_point = seahowl::core::BladeReferencePoint();

        auto coords = point.at("coordinates").get<std::vector<double>>();
        if (coords.size() != 3) {
            throw std::runtime_error("Coordinates has to be vector of length 3.");
        }
        reference_point.fraction = coords[2] / blade_length;
        reference_point.m_coordinates = chrono::ChVector<double>(coords[0], coords[1], coords[2]);
        if (point.contains("m_offset_gravity")) {
            auto og = point.at("offsets_gravity").get<std::vector<double>>();
            reference_point.m_offset_gravity = chrono::ChVector2<double>(og[0], og[1]);
        }
        if (point.contains("offset_elastic")) {
            auto oe = point.at("offsets_elastic").get<std::vector<double>>();
            reference_point.m_offset_elastic = chrono::ChVector2<double>(oe[0], oe[1]);
        }

        auto sm = point.at("stiffness_matrix").get<std::vector<std::vector<double>>>();
        auto mm = point.at("mass_matrix").get<std::vector<std::vector<double>>>();
        if (sm.size() != 6 || mm.size() != 6) {
            throw std::runtime_error("Mass and stiffness matrices hqve to be defined as 6x6 matrices.");
        }
        int jjo, kko;
        // apply offsets to indices to switch from IEC standard to Chrono standard
        for (int jj = 0; jj < 6; jj++) {
            if (sm[jj].size() != 6 || mm[jj].size() != 6) {
                throw std::runtime_error("Mass and stiffness matrices hqve to be defined as 6x6 matrices.");
            }

            if (jj == 2 || jj == 5) {
                jjo = -2;
            }
            if (jj == 1 || jj == 4) {
                jjo = +0;
            }
            if (jj == 0 || jj == 3) {
                jjo = +2;
            }
            for (int kk = 0; kk < 6; kk++) {
                if (kk == 2 || kk == 5) {
                    kko = -2;
                }
                if (kk == 1 || kk == 4) {
                    kko = +0;
                }
                if (kk == 0 || kk == 3) {
                    kko = +2;
                }
                reference_point.stiffness_matrix(jj + jjo, kk + kko) = sm[jj][kk];
                reference_point.mass_matrix(jj + jjo, kk + kko) = mm[jj][kk];
            }
        }
        reference_point.structural_twist = point.at("twist").get<double>() * chrono::CH_C_PI / 180.0;
        reference_point.damping_coefficients.bx = damping_coefficients[0];
        reference_point.damping_coefficients.by = damping_coefficients[1];
        reference_point.damping_coefficients.bz = damping_coefficients[2];
        reference_point.damping_coefficients.bt = damping_coefficients[3];

        if (point.contains("chord")) {
            reference_point.chord = point["chord"];
        }

        // populate json object
        if (point.contains("airfoil_file")) {
            auto main_directory = fs::path(filepath).parent_path();
            auto airfoil_filename = point.at("airfoil_file").get<std::string>();
            auto airfoil_filepath = main_directory / airfoil_filename;
            std::ifstream airfoil_file(airfoil_filepath.u8string());
            json json_airfoil;
            airfoil_file >> json_airfoil;

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
                    coefficients.added_mass = coeffs[kk][3];
                    coefficients_list.push_back(coefficients);
                }
                seahowl::aero::AirfoilProperties airfoil;
                airfoil_properties.at("reynolds_number").get_to(airfoil.reynolds_number);
                airfoil.coefficients_list = coefficients_list;
                reference_point.airfoil_properties.push_back(airfoil);
            }
        }

        reference_points.push_back(reference_point);
    }

    return reference_points;
}

seahowl::core::Blade get_blade_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    seahowl::core::Blade blade{};
    blade.elasto->fpm_mode = json_obj.at("fpm_mode").get<bool>();
    blade.reference_points = get_blade_reference_points_from_json(filepath);
    if (json_obj.contains("discretization_elasto")) {
        auto discretization_elasto = json_obj.at("discretization_elasto").get<std::vector<double>>();
        blade.set_discretization_elasto(discretization_elasto);
    }
    if (json_obj.contains("discretization_aero")) {
        auto discretization_aero = json_obj.at("discretization_aero").get<std::vector<double>>();
        blade.set_discretization_aero(discretization_aero);
    }

    return blade;
}

std::vector<seahowl::core::TowerReferencePoint> get_tower_reference_points_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    // EXTRACT INFO
    double height = json_obj.at("height").get<double>();
    double base_height = json_obj.at("base_height").get<double>();
    auto damping_coefficients = json_obj.at("damping_coefficients").get<std::vector<double>>();

    // MAKE TOWER REFERENCE POINTS
    std::vector<seahowl::core::TowerReferencePoint> reference_points;
    auto points = json_obj.at("reference_points").get<json>();
    for (int ii = 0; ii < points.size(); ii++) {
        auto& point = points[ii];
        auto reference_point = seahowl::core::TowerReferencePoint();
        point.at("fraction").get_to(reference_point.fraction);
        reference_point.coordinates =
            chrono::ChVector<double>(0.0, 0.0, (height - base_height) * reference_point.fraction + base_height);
        point.at("stiffness_sideside").get_to(reference_point.stiffness_sideside);
        point.at("stiffness_foreaft").get_to(reference_point.stiffness_foreaft);
        point.at("density").get_to(reference_point.density);
        point.at("diameter").get_to(reference_point.diameter);
        point.at("drag_coefficient").get_to(reference_point.drag_coefficient);
        reference_point.damping_coefficients.bx = damping_coefficients[0];
        reference_point.damping_coefficients.by = damping_coefficients[1];
        reference_point.damping_coefficients.bz = damping_coefficients[2];
        reference_point.damping_coefficients.bt = damping_coefficients[3];

        ///@todo change to actual values
        reference_point.stiffness_axial = 210e9;
        reference_point.stiffness_torsion = 1e11;

        reference_points.push_back(reference_point);
    }

    return reference_points;
}

seahowl::core::Tower get_tower_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    seahowl::core::Tower tower{};
    tower.elasto.height = json_obj.at("height").get<double>();
    tower.elasto.base_height = json_obj.at("base_height").get<double>();
    tower.reference_points = get_tower_reference_points_from_json(filepath);
    if (json_obj.contains("discretization_elasto")) {
        auto discretization_elasto = json_obj.at("discretization_elasto").get<std::vector<double>>();
        tower.set_discretization_elasto(discretization_elasto);
    }
    if (json_obj.contains("discretization_aero")) {
        auto discretization_aero = json_obj.at("discretization_aero").get<std::vector<double>>();
        tower.set_discretization_aero(discretization_aero);
    }

    return tower;
}

seahowl::core::Rotor get_rotor_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    // EXTRACT INFO
    //
    seahowl::core::Rotor rotor;
    // blades
    json_obj.at("precones").get_to(rotor.elasto.blade_precones);
    for (int ii = 0; ii < rotor.elasto.blade_precones.size(); ii++) {
        // convert to radians
        rotor.elasto.blade_precones[ii] *= chrono::CH_C_PI / 180.0;
    }
    // hub
    auto hub = json_obj.at("hub");
    hub.at("CM").get_to(rotor.elasto.hub.center_of_mass);
    hub.at("mass").get_to(rotor.elasto.hub.mass);
    hub.at("inertia").get_to(rotor.elasto.hub.inertia);
    hub.at("overhang").get_to(rotor.elasto.hub.overhang);
    hub.at("radius").get_to(rotor.elasto.hub.radius);
    hub.at("radius").get_to(rotor.aero.hub_radius);
    // nacelle
    auto nacelle = json_obj.at("nacelle");
    auto cm = nacelle.at("CM").get<std::vector<double>>();
    if (cm.size() != 3) {
        throw std::runtime_error("Center of mass of nacelle has to be vector of length 3.");
    }
    rotor.elasto.nacelle.center_of_mass = chrono::ChVector<double>(cm[0], cm[1], cm[2]);
    nacelle.at("mass").get_to(rotor.elasto.nacelle.mass);
    nacelle.at("inertia").get_to(rotor.elasto.nacelle.inertia);
    nacelle.at("yaw_bearing_mass").get_to(rotor.elasto.nacelle.yaw_bearing_mass);
    // shaft
    auto shaft = json_obj.at("shaft");
    shaft.at("distance_from_towertop").get_to(rotor.elasto.shaft.distance_from_towertop);
    shaft.at("tilt").get_to(rotor.elasto.shaft.tilt);
    // convert to radians
    rotor.elasto.shaft.tilt *= chrono::CH_C_PI / 180.0;

    return rotor;
}

seahowl::core::Turbine get_turbine_from_json(std::vector<std::string> filepaths_blades,
                                             std::string filepath_rotor,
                                             std::string filepath_tower) {
    std::vector<std::shared_ptr<seahowl::core::Blade>> blades;
    for (auto& fpath : filepaths_blades) {
        blades.push_back(std::make_shared<seahowl::core::Blade>(get_blade_from_json(fpath)));
    }

    auto rotor = get_rotor_from_json(filepath_rotor);

    auto tower = get_tower_from_json(filepath_tower);

    auto turbine = seahowl::core::Turbine();
    turbine.rotor = rotor;
    turbine.tower = tower;
    turbine.blades = blades;

    // get extra drivetrain info
    std::ifstream json_file(filepath_rotor);
    // populate json object
    json json_obj;
    json_file >> json_obj;
    auto drivetrain = json_obj.at("drivetrain");
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
    turbine.rotor.elasto.hub.inertia += drivetrain_inertia;

    return turbine;
}
