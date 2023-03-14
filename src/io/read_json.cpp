#include "seahowl/io/read_json.h"

#include <seahowl/core/utils.h>
#include <seahowl/core/blade.h>
#include <seahowl/core/rotor.h>
#include <seahowl/core/tower.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/core/turbine.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/servo/controller_discon.h>
#include <seahowl/core/system.h>
#include <seahowl/commons.h>

#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <iostream>
#include <typeinfo>
#include <filesystem>
namespace fs = std::filesystem;
using std::filesystem::path;
using std::filesystem::absolute;

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using seahowl::Vector3d;
using seahowl::Vector2d;
using seahowl::PI;

/**@brief Copy file to destination dir, increment file name if already exists, and return path to new copid file.
 */
std::string copy_file_and_increment(std::string filepath, std::string destination_dir) {
    if (!fs::exists(destination_dir)) {
        fs::create_directories(destination_dir);
    }
    path pfilepath = fs::path(filepath);
    path filecopypath;
    if (fs::exists(destination_dir / pfilepath.filename())) {
        auto filename = pfilepath.stem().generic_string();
        auto fileext = pfilepath.extension().generic_string();
        bool copied = false;
        int file_idx = 0;
        while (!copied) {
            filecopypath = fs::path(destination_dir) / (filename + std::to_string(file_idx) + fileext);
            if (!fs::exists(filecopypath)) {
                fs::copy(pfilepath, filecopypath);
                pfilepath = filecopypath;
                copied = true;
            } else {
                file_idx += 1;
            }
        }
    } else {
        filecopypath = destination_dir / pfilepath.filename();
        fs::copy(pfilepath, filecopypath);
    }
    return filecopypath.generic_string();
}

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
        reference_point.coordinates = Vector3d(coords[0], coords[1], coords[2]);
        if (point.contains("offset_gravity")) {
            auto og = point.at("offsets_gravity").get<std::vector<double>>();
            reference_point.offset_gravity = Vector2d(og[0], og[1]);
        }
        if (point.contains("offset_elastic")) {
            auto oe = point.at("offsets_elastic").get<std::vector<double>>();
            reference_point.offset_elastic = Vector2d(oe[0], oe[1]);
        }
        if (point.contains("offset_aero")) {
            auto oa = point.at("offset_aero").get<std::vector<double>>();
            reference_point.offset_aero = Vector2d(oa[0], oa[1]);
        }

        auto sm = point.at("stiffness_matrix").get<std::vector<std::vector<double>>>();
        auto mm = point.at("mass_matrix").get<std::vector<std::vector<double>>>();
        if (sm.size() != 6 || mm.size() != 6) {
            throw std::runtime_error("Mass and stiffness matrices have to be defined as 6x6 matrices.");
        }
        for (int irow = 0; irow < 6; irow++) {
            if (sm[irow].size() != 6 || mm[irow].size() != 6) {
                throw std::runtime_error("Mass and stiffness matrices have to be defined as 6x6 matrices.");
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

        if (point.contains("chord")) {
            reference_point.chord = point["chord"];
        }

        // populate json object
        if (point.contains("airfoil_file") && !point["airfoil_file"].get<std::string>().empty()) {
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
    blade.reference_points = get_blade_reference_points_from_json(filepath);

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
            Vector3d(0.0, 0.0, (height - base_height) * reference_point.fraction + base_height);
        point.at("stiffness_sideside").get_to(reference_point.stiffness_sideside);
        point.at("stiffness_foreaft").get_to(reference_point.stiffness_foreaft);
        point.at("density").get_to(reference_point.density);
        point.at("diameter").get_to(reference_point.diameter);
        point.at("drag_coefficient").get_to(reference_point.drag_coefficient);
        reference_point.damping_coefficients[0] = damping_coefficients[0];
        reference_point.damping_coefficients[1] = damping_coefficients[1];
        reference_point.damping_coefficients[2] = damping_coefficients[2];
        reference_point.damping_coefficients[3] = damping_coefficients[3];

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
        rotor.elasto.blade_precones[ii] *= PI / 180.0;
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
    rotor.elasto.nacelle.center_of_mass = Vector3d(cm[0], cm[1], cm[2]);
    nacelle.at("mass").get_to(rotor.elasto.nacelle.mass);
    nacelle.at("inertia").get_to(rotor.elasto.nacelle.inertia);
    nacelle.at("yaw_bearing_mass").get_to(rotor.elasto.nacelle.yaw_bearing_mass);
    // shaft
    auto shaft = json_obj.at("shaft");
    shaft.at("distance_from_towertop").get_to(rotor.elasto.shaft.distance_from_towertop);
    shaft.at("tilt").get_to(rotor.elasto.shaft.tilt);
    // convert to radians
    rotor.elasto.shaft.tilt *= PI / 180.0;

    return rotor;
}

seahowl::core::Turbine get_turbine_from_json_files(std::vector<std::string> filepaths_blades,
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

seahowl::core::Turbine get_turbine_from_json(std::string filepath_turbine) {
    // get turbine info
    std::ifstream json_file(filepath_turbine);
    // populate json object
    json json_obj;
    json_file >> json_obj;

    auto DATADIR = absolute(path(filepath_turbine).parent_path());

    auto blades_json = json_obj.at("blades");
    auto tower_json = json_obj.at("tower");
    auto rna_json = json_obj.at("rna");
    auto controller_json = json_obj.at("controller");

    // blades
    std::vector<std::shared_ptr<seahowl::core::Blade>> blades;
    auto blades_json2 = blades_json.at("blades");
    for (auto& blade_json : blades_json2) {
        auto filepath_blade = (DATADIR / blade_json.at("file").get<std::string>()).generic_string();
        auto blade = std::make_shared<seahowl::core::Blade>(get_blade_from_json(filepath_blade));
        blades_json.at("discretization").at("elasto").get_to(blade->elasto->discretization_fractions);
        blades_json.at("discretization").at("aero").get_to(blade->aero->discretization_fractions);
        blades_json.at("fpm").get_to(blade->elasto->fpm_mode);
        blade_json.at("initial_pitch").get_to(blade->elasto->pitch);
        blades.push_back(blade);
    }

    // RNA
    auto filepath_rotor = (DATADIR / rna_json.at("file").get<std::string>()).generic_string();
    auto rotor = get_rotor_from_json(filepath_rotor);
    rna_json.at("initial_pitch_collective").get_to(rotor.elasto.pitch_collective);

    // tower
    auto filepath_tower = (DATADIR / tower_json.at("file").get<std::string>()).generic_string();
    auto tower = get_tower_from_json(filepath_tower);
    tower_json.at("discretization").at("elasto").get_to(tower.elasto.discretization_fractions);
    tower_json.at("discretization").at("aero").get_to(tower.aero.discretization_fractions);

    auto turbine = seahowl::core::Turbine();
    turbine.rotor = rotor;
    turbine.tower = tower;
    turbine.blades = blades;

    // controller
    if (controller_json.at("type").get<std::string>() == "DISCON") {
        auto OUTPUT_CONTROLLER_DIR = path("./output/dynlib_copies");
        auto libfilepath = path(DATADIR / controller_json.at("options").at("libfile"));
        if (!std::filesystem::exists(libfilepath)) {
            throw std::invalid_argument(
                "Dynamic library path for DISCON routine does not exist: " + libfilepath.generic_string() + ".");
        }
        auto copyfilepath =
            copy_file_and_increment(libfilepath.generic_string(), OUTPUT_CONTROLLER_DIR.generic_string());
        turbine.controller = std::make_shared<seahowl::servo::ControllerDISCON>(
            (DATADIR / controller_json.at("options").at("infile")).generic_string(), copyfilepath);
    }

    // get extra drivetrain info
    std::ifstream json_file2(filepath_rotor);
    // populate json object
    json json_obj2;
    json_file2 >> json_obj2;
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
    turbine.rotor.elasto.hub.inertia += drivetrain_inertia;

    return turbine;
}

seahowl::core::System get_system_from_json(std::string filepath_main,
                                           std::shared_ptr<seahowl::elasto::SystemElasto> system_elasto,
                                           std::shared_ptr<seahowl::elasto::MeshElasto> mesh_elasto) {
    auto DATADIR = absolute(path(filepath_main)).parent_path();

    // get main info
    std::ifstream json_file(filepath_main);
    // populate json object
    json json_obj;
    json_file >> json_obj;

    // environmental info
    auto environment_json = json_obj.at("environment");
    // gravity
    auto gravity = environment_json.at("gravity").get<std::vector<double>>();
    system_elasto->set_gravitational_acceleration(Vector3d(gravity[0], gravity[1], gravity[2]));

    // system
    auto seahowl_system = seahowl::core::System();
    auto wind_json = environment_json.at("wind");
    if (wind_json.at("type").get<std::string>() == "ramp") {
        seahowl_system.wind_model = std::make_shared<seahowl::aero::WindRamp>();
        auto wind_options = wind_json.at("options");
        auto wind_model = std::dynamic_pointer_cast<seahowl::aero::WindRamp>(seahowl_system.wind_model);
        auto v0 = wind_options.at("velocity_start").get<std::vector<double>>();
        wind_model->wind_velocity_start = Vector3d(v0[0], v0[1], v0[2]);
        auto v1 = wind_options.at("velocity_stop").get<std::vector<double>>();
        wind_model->wind_velocity_stop = Vector3d(v1[0], v1[1], v1[2]);
        wind_model->direction_gravity = Vector3d(system_elasto->get_gravitational_acceleration()).normalized();
        wind_model->reference_height = wind_options.at("reference_height").get<double>();
        wind_model->time_start = wind_options.at("time_start").get<double>();
        wind_model->time_stop = wind_options.at("time_stop").get<double>();
        wind_model->shear_coefficient = wind_options.at("shear_coefficient").get<double>();
        wind_model->density = environment_json.at("air_density").get<double>();
    } else {
        throw std::runtime_error("Only wind ramp is allowed as input.");
    }

    auto turbines_json = json_obj.at("turbines");
    for (int ii = 0; ii < turbines_json.size(); ii++) {
        auto turbine_json = turbines_json[ii];
        auto filepath_turbine = (DATADIR / turbine_json.at("file").get<std::string>()).generic_string();
        seahowl_system.turbines.push_back(seahowl::core::Turbine(get_turbine_from_json(filepath_turbine)));
        auto& turbine = seahowl_system.turbines.back();
        // aerodyn option
#ifdef HAVE_AERODYN
        turbine.use_aerodyn = turbine_json.at("use_aerodyn").get<bool>();
        bool output_vtk = json_obj.at("outputs").at("VTK").get<bool>();
        if (output_vtk) {
            turbine.WrVTK = 2;
        }
        turbine.WrVTK_dt = json_obj.at("outputs").at("dt").get<double>();
        if (turbine.use_aerodyn) {
            std::string inflowwind_filepath;
            std::string aerodyn_filepath;
            if (turbine_json.contains("file_aerodyn")) {
                aerodyn_filepath = (DATADIR / turbine_json.at("file_aerodyn")).generic_string();
            } else {
                throw std::runtime_error("Turbine set to use aerodyn but AeroDyn file path not defined.");
            }
            if (turbine_json.contains("file_inflowwind")) {
                inflowwind_filepath = (DATADIR / turbine_json.at("file_inflowwind")).generic_string();
            } else {
                throw std::runtime_error("Turbine set to use aerodyn but InflowWind file not defined.");
            }
            turbine.aerodyn = std::make_shared<seahowl::aero::AeroDynAdapter>(aerodyn_filepath, inflowwind_filepath);
        }
#endif
        // build turbine (Chrono)
        turbine.build();
        turbine.assemble(system_elasto, mesh_elasto);
        // rotate turbine to align tower with gravity vector
        auto v1 = Vector3d(-system_elasto->get_gravitational_acceleration()).normalized();
        auto v2 = (turbine.tower.elasto.nodes[1]->get_position() - turbine.tower.elasto.nodes[0]->get_position())
                      .normalized();
        auto rot_axis = v2.cross(v1);
        auto rot_angle = acos(v1.dot(v2));
        turbine.rotate(rot_angle, rot_axis);
        // rotation around axis opposite to gravity (yaw)
        turbine.rotate(turbine_json.at("rotation").get<double>(),
                       Vector3d(-system_elasto->get_gravitational_acceleration()).normalized());
        // translate turbine
        auto trans = turbine_json.at("translation").get<std::vector<double>>();
        turbine.translate(Vector3d(trans[0], trans[1], trans[2]));

        // apply initial pitches
        for (auto blade : turbine.blades) {
            auto pitch0 = blade->elasto->pitch;
            blade->elasto->apply_pitch_increment(pitch0);
            blade->elasto->pitch = pitch0;
        }
        auto rotor_pitch0 = turbine.rotor.elasto.pitch_collective;
        turbine.rotor.elasto.apply_collective_pitch_increment(rotor_pitch0);
        turbine.rotor.elasto.pitch_collective = rotor_pitch0;
    }

    return seahowl_system;
}
