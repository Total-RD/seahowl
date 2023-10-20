#include "seahowl/io/read_json.h"
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
#include "seahowl/servo/controller_discon.h"
#include <seahowl/commons/numerics.h>
#include "seahowl/aero/inflowwind_adapter.h"
#include "seahowl/aero/airfoil.h"
#include "seahowl/aero/blade_aero.h"
#include "seahowl/aero/rotor_aero.h"
#include "seahowl/aero/turbine_aero.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/core/turbine_floating.h"
#ifdef HAVE_HYDROCHRONO
    #include "seahowl/hydro/hydrochrono_adapter.h"
    #include "seahowl/elasto/chrono_adapters.h"
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

void check_file_exists(const std::string& filepath) {
    if (!fs::exists(filepath)) {
        spdlog::critical("File {} does not exist.", filepath);
        exit(1);
    }
}

json get_json_from_file(const std::string& filepath) {
    check_file_exists(filepath);
    std::ifstream json_file(filepath);
    json json_obj;
    json_file >> json_obj;
    return json_obj;
}

std::vector<seahowl::elasto::BladeReferencePointElasto> get_blade_elasto_reference_points_from_json(
    std::string filepath) {
    auto json_obj = get_json_from_file(filepath);

    // EXTRACT INFO
    std::vector<seahowl::elasto::BladeReferencePointElasto> reference_points;

    auto points = json_obj.at("reference_points").get<json>();
    auto damping_coefficients = json_obj.at("damping_coefficients").get<std::vector<double>>();
    if (damping_coefficients.size() != 4) {
        spdlog::critical("Damping coefficients of blade has to be vector of length 4.");
        exit(1);
    }

    double blade_length = points[points.size() - 1]["coordinates"][2];
    for (size_t ii = 0; ii < points.size(); ii++) {
        auto& point = points[ii];
        auto reference_point = seahowl::elasto::BladeReferencePointElasto();

        auto coords = point.at("coordinates").get<std::vector<double>>();
        if (coords.size() != 3) {
            spdlog::critical("Coordinates along blade have to be vectors of length 3.");
            exit(1);
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
            spdlog::critical("Mass and stiffness matrices along blade have to be defined as 6x6 matrices.");
            exit(1);
        }
        for (int irow = 0; irow < 6; irow++) {
            if (sm[irow].size() != 6 || mm[irow].size() != 6) {
                spdlog::critical("Mass and stiffness matrices along blade have to be defined as 6x6 matrices.");
                exit(1);
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

std::vector<seahowl::aero::BladeReferencePointAero> get_blade_aero_reference_points_from_json(std::string filepath) {
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
            spdlog::critical("Coordinates along blade have to be vectors of length 3.");
            exit(1);
        }
        reference_point.fraction = coords[2] / blade_length;
        reference_point.coordinates = Vector3d(coords[0], coords[1], coords[2]);
        if (point.contains("offset_aero")) {
            auto oa = point.at("offset_aero").get<std::vector<double>>();
            reference_point.offset_aero = Vector2d(oa[0], oa[1]);
        }
        reference_point.structural_twist = point.at("twist").get<double>() * PI / 180.0;

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
                        spdlog::critical("Airfoil coefficients has to be vectors of length 4.");
                        exit(1);
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

            // push only if airfoil file present
            // @todo make it possible to push without airfoil file
            reference_points.push_back(reference_point);
        }
    }

    return reference_points;
}

void populate_blade_elasto_from_json(std::string filepath, seahowl::elasto::BladeElasto& blade) {
    blade.reference_points = get_blade_elasto_reference_points_from_json(filepath);
}

void populate_blade_aero_from_json(std::string filepath, seahowl::aero::BladeAero& blade) {
    blade.reference_points = get_blade_aero_reference_points_from_json(filepath);
}

void populate_blade_from_json(std::string filepath, seahowl::core::Blade& blade) {
    spdlog::debug("Populating blade from " + filepath + " file.");
    populate_blade_elasto_from_json(filepath, blade.elasto);
    populate_blade_aero_from_json(filepath, blade.aero);
}

std::vector<seahowl::elasto::TowerReferencePointElasto> get_tower_elasto_reference_points_from_json(
    std::string filepath) {
    auto json_obj = get_json_from_file(filepath);

    // EXTRACT INFO
    double height = json_obj.at("height").get<double>();
    double base_height = json_obj.at("base_height").get<double>();
    auto damping_coefficients = json_obj.at("damping_coefficients").get<std::vector<double>>();

    // MAKE TOWER REFERENCE POINTS
    std::vector<seahowl::elasto::TowerReferencePointElasto> reference_points;
    auto points = json_obj.at("reference_points").get<json>();
    for (int ii = 0; ii < points.size(); ii++) {
        auto& point = points[ii];
        auto reference_point = seahowl::elasto::TowerReferencePointElasto();
        point.at("fraction").get_to(reference_point.fraction);
        reference_point.coordinates =
            Vector3d(0.0, 0.0, (height - base_height) * reference_point.fraction + base_height);
        point.at("stiffness_sideside").get_to(reference_point.stiffness_sideside);
        point.at("stiffness_foreaft").get_to(reference_point.stiffness_foreaft);
        point.at("density").get_to(reference_point.density);
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

std::vector<seahowl::aero::TowerReferencePointAero> get_tower_aero_reference_points_from_json(std::string filepath) {
    auto json_obj = get_json_from_file(filepath);

    // EXTRACT INFO
    double height = json_obj.at("height").get<double>();
    double base_height = json_obj.at("base_height").get<double>();
    auto damping_coefficients = json_obj.at("damping_coefficients").get<std::vector<double>>();

    // MAKE TOWER REFERENCE POINTS
    std::vector<seahowl::aero::TowerReferencePointAero> reference_points;
    auto points = json_obj.at("reference_points").get<json>();
    for (int ii = 0; ii < points.size(); ii++) {
        auto& point = points[ii];
        auto reference_point = seahowl::aero::TowerReferencePointAero();
        point.at("fraction").get_to(reference_point.fraction);
        reference_point.coordinates =
            Vector3d(0.0, 0.0, (height - base_height) * reference_point.fraction + base_height);
        point.at("diameter").get_to(reference_point.diameter);
        point.at("drag_coefficient").get_to(reference_point.drag_coefficient);

        reference_points.push_back(reference_point);
    }

    return reference_points;
}

void populate_tower_elasto_from_json(std::string filepath, seahowl::elasto::TowerElasto& tower) {
    auto json_obj = get_json_from_file(filepath);

    tower.reference_points = get_tower_elasto_reference_points_from_json(filepath);
    tower.height = json_obj.at("height").get<double>();
    tower.base_height = json_obj.at("base_height").get<double>();
}

void populate_tower_aero_from_json(std::string filepath, seahowl::aero::TowerAero& tower) {
    check_file_exists(filepath);
    tower.reference_points = get_tower_aero_reference_points_from_json(filepath);
}

void populate_tower_from_json(std::string filepath, seahowl::core::Tower& tower) {
    spdlog::debug("Populating tower from " + filepath + " file.");
    populate_tower_elasto_from_json(filepath, tower.elasto);
    populate_tower_aero_from_json(filepath, tower.aero);
}

void populate_rna_elasto_from_json(std::string filepath, seahowl::elasto::RotorNacelleAssemblyElasto& rna) {
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
        spdlog::critical("Center of mass of nacelle has to be vector of length 3.");
        exit(1);
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

void populate_rna_aero_from_json(std::string filepath, seahowl::aero::RotorNacelleAssemblyAero& rna) {
    auto json_obj = get_json_from_file(filepath);

    // EXTRACT INFO
    //
    // hub
    auto hub = json_obj.at("hub");
    hub.at("radius").get_to(rna.hub_radius);
}

void populate_rna_from_json(std::string filepath, seahowl::core::RotorNacelleAssembly& rna) {
    spdlog::info("Populating RNA from " + filepath + " file.");
    populate_rna_elasto_from_json(filepath, rna.elasto);
    populate_rna_aero_from_json(filepath, rna.aero);
}

void populate_turbine_from_json(std::string filepath, seahowl::core::Turbine& turbine) {
    spdlog::debug("Populating turbine from " + filepath + " file.");
    auto json_obj = get_json_from_file(filepath);

    auto DATADIR = absolute(path(filepath).parent_path());

    auto rotor_json = json_obj.at("rotor");
    auto tower_json = json_obj.at("tower");
    auto rna_json = json_obj.at("rna");
    auto controller_json = json_obj.at("controller");

    if (rotor_json.at("type").get<std::string>() == "disk") {
        turbine.aero.use_disktheory = true;
        spdlog::info("Aerodynamic model: Actuator Disk Theory");
        // get rotor performance from table
        if (!rotor_json.contains("performance_file")) {
            spdlog::critical("The \"performance_file\" key must be given for actuator disk rotor.");
            exit(1);
        }
        get_disk_perf_from_table((DATADIR / rotor_json.at("performance_file")).generic_string(), turbine.rna.aero);
        if (turbine.aero.use_aerodyn == true) {
            spdlog::critical("When Disk Theory is activated, you can't ask for AeroDyn module.");
            exit(1);
        }
    } else
        spdlog::info("Aerodynamic model: Blade Element Momentum Theory");

    // blades
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
            rotor_json.at("fpm").get_to(blade_elasto_fea.fpm_mode);
        } else if (rotor_json.at("type").get<std::string>() == "rigid") {
            blade_elasto = std::make_shared<seahowl::elasto::BladeElastoRigid>();
        } else if (rotor_json.at("type").get<std::string>() == "disk") {
            blade_elasto = std::make_shared<seahowl::elasto::BladeElastoRigid>();
        } else {
            spdlog::critical("Wrong blade type: try \"fea\" or \"rigid\" or \"disk\".");
            exit(1);
        }
        auto blade_aero = std::make_shared<seahowl::aero::BladeAero>();
        auto blade = std::make_shared<seahowl::core::Blade>(*blade_elasto, *blade_aero);
        populate_blade_from_json(filepath_blade, *blade);
        rotor_json.at("discretization").at("aero").get_to(blade->aero.discretization_fractions);
        blade_json.at("initial_pitch").get_to(blade->elasto.pitch);
        blade_elasto->precone = blade_json.at("precone").get<double>() * PI / 180.0;
        // no precone if blade is rigid (assumed that blade is on rotor disc)
        if (rotor_json.at("type").get<std::string>() == "rigid" || rotor_json.at("type").get<std::string>() == "disk") {
            blade_elasto->precone = 0.0;
        }
        blades_elasto.push_back(blade_elasto);
        blades_aero.push_back(blade_aero);
        blades.push_back(blade);
    }

    turbine.elasto.rna.rotor->blades = blades_elasto;
    turbine.aero.rna.blades = blades_aero;
    turbine.rna.blades = blades;

    // RNA
    auto filepath_rna = (DATADIR / rna_json.at("file").get<std::string>()).generic_string();
    populate_rna_from_json(filepath_rna, turbine.rna);
    rna_json.at("initial_pitch_collective").get_to(turbine.elasto.rna.rotor->pitch_collective);

    // update info if rotor is rigid
    if (rotor_json.at("type").get<std::string>() == "rigid" || rotor_json.at("type").get<std::string>() == "disk") {
        if (!rotor_json.contains("inertia_total")) {
            spdlog::critical("The \"inertia_total\" key must be provided for rigid rotors.");
            exit(1);
        }
        if (!rotor_json.contains("mass_blades_total")) {
            spdlog::critical("The \"mass_blades_total\" key must be given for rigid rotors.");
            exit(1);
        }
        auto rotor_inertia = rotor_json.at("inertia_total").get<double>();
        turbine.rna.elasto.rotor->hub.inertia = rotor_inertia;
        auto blades_mass = rotor_json.at("mass_blades_total").get<double>();
        turbine.rna.elasto.rotor->hub.mass += blades_mass;
    }

    // tower
    auto filepath_tower = (DATADIR / tower_json.at("file").get<std::string>()).generic_string();
    populate_tower_from_json(filepath_tower, turbine.tower);
    tower_json.at("discretization").at("elasto").get_to(turbine.elasto.tower.discretization_fractions);
    tower_json.at("discretization").at("aero").get_to(turbine.aero.tower.discretization_fractions);

    // controller
    if (controller_json.at("type").get<std::string>() == "DISCON") {
        auto OUTPUT_CONTROLLER_DIR = path("./output/dynlib_copies");
        if (!controller_json.at("options").contains("libfile")) {
            spdlog::critical("Need to define path to libfile for DISCON routine.");
            exit(1);
        }
        auto libfilepath = path(DATADIR / controller_json.at("options").at("libfile"));
        check_file_exists(libfilepath.generic_string());
        auto copyfilepath =
            copy_file_and_increment(libfilepath.generic_string(), OUTPUT_CONTROLLER_DIR.generic_string());
        turbine.controller = std::make_shared<seahowl::servo::ControllerDISCON>(
            (DATADIR / controller_json.at("options").at("infile")).generic_string(), copyfilepath);
    } else if (controller_json.at("type").get<std::string>() == "RPM") {
        if (!controller_json.at("options").contains("target_rpm")) {
            spdlog::critical("Need to define target RPM for RPM controller (target_rpm).");
            exit(1);
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
        auto floater_json = json_obj.at("floater");
        auto json_obj_floater =
            get_json_from_file((DATADIR / floater_json.at("file").get<std::string>()).generic_string());

        auto floater_type = json_obj_floater.at("type").get<std::string>();
        if (floater_type == "HydroChrono") {
#ifdef HAVE_HYDROCHRONO
            // dynamic cast turbine
            auto& turbine_floating = dynamic_cast<seahowl::elasto::TurbineFloatingElasto&>(turbine.elasto);
            // make floater
            turbine_floating.floater = std::make_unique<seahowl::hydro::FloaterHydroChrono>();
            auto& floater = dynamic_cast<seahowl::hydro::FloaterHydroChrono&>(*turbine_floating.floater);
            auto floater_options = json_obj_floater.at("options");
            // add h5file path
            floater.set_h5_filepath((DATADIR / floater_options.at("file").get<std::string>()).generic_string());
            // make body
            auto body_name = floater_options.at("name").get<std::string>();
            floater.add_body(body_name);
            // position body
            auto& body = floater.get_body(body_name);
            body.set_mass(json_obj_floater.at("mass").get<double>());
            auto body_position = json_obj_floater.at("cog").get<std::vector<double>>();
            if (body_position.size() != 3) {
                throw std::runtime_error("COG of floater should be a vector of length 3.");
            }
            body.set_position(Vector3d(body_position[0], body_position[1], body_position[2]));
            // inertia body
            auto body_inertia = json_obj_floater.at("inertia").get<std::vector<std::vector<double>>>();
            if (body_inertia.size() != 3) {
                throw std::runtime_error("Inertia matrix of floater should be 3x3.");
            }
            Eigen::Matrix<double, 3, 3> body_inertia_matrix;
            for (int row = 0; row < 3; row++) {
                if (body_inertia[row].size() != 3) {
                    throw std::runtime_error("Inertia matrix of floater should be 3x3.");
                }
                for (int col = 0; col < 3; col++) {
                    body_inertia_matrix(row, col) = body_inertia[row][col];
                }
            }
            body.set_inertia_matrix(body_inertia_matrix);
#endif
        } else {
            throw std::runtime_error("Type of floater defined in turbine json file does not exist.");
        }
    }
}

void populate_system_from_json(std::string filepath, seahowl::core::System& system_core) {
    auto DATADIR = absolute(path(filepath)).parent_path();

    auto json_obj = get_json_from_file(filepath);

    // outputs
    auto outputs_json = json_obj.at("outputs");
    // logging
    auto log_level = outputs_json.at("log_level").get<std::string>();
    seahowl::set_log_level_global(log_level);
    spdlog::debug("Populating system from " + filepath + " file.");

    // environmental info
    auto environment_json = json_obj.at("environment");
    // gravity
    auto gravity = environment_json.at("gravity").get<std::vector<double>>();
    system_core.system_elasto->set_gravitational_acceleration(Vector3d(gravity[0], gravity[1], gravity[2]));
    // wind
    auto wind_json = environment_json.at("wind");
    if (wind_json.at("type").get<std::string>() == "ramp") {
        system_core.wind_model = std::make_shared<seahowl::aero::WindRamp>();
        auto wind_options = wind_json.at("options");
        auto wind_model = std::dynamic_pointer_cast<seahowl::aero::WindRamp>(system_core.wind_model);
        auto v0 = wind_options.at("velocity_start").get<std::vector<double>>();
        auto v1 = wind_options.at("velocity_end").get<std::vector<double>>();
        wind_model->set_wind_ramp(Vector3d(v0[0], v0[1], v0[2]), wind_options.at("time_start").get<double>(),
                                  Vector3d(v1[0], v1[1], v1[2]), wind_options.at("time_end").get<double>());
        wind_model->direction_gravity =
            Vector3d(system_core.system_elasto->get_gravitational_acceleration()).normalized();
        wind_model->reference_height = wind_options.at("reference_height").get<double>();
        wind_model->shear_coefficient = wind_options.at("shear_coefficient").get<double>();
        wind_model->density = environment_json.at("air_density").get<double>();
    } else if (wind_json.at("type").get<std::string>() == "inflowwind") {
#ifdef HAVE_INFLOWWIND
        std::string inflowwind_filepath;
        std::string windwnd_filepath;
        auto wind_options = wind_json.at("options");
        if (wind_options.contains("file_inflowwind")) {
            inflowwind_filepath = (DATADIR / wind_options.at("file_inflowwind")).generic_string();
        } else {
            spdlog::critical("InflowWind file not defined.");
            exit(1);
        }
        if (wind_options.contains("file_windwnd")) {
            windwnd_filepath = (DATADIR / wind_options.at("file_windwnd")).generic_string();
        } else {
            spdlog::critical("InflowWind input file (.wnd) not defined.");
            exit(1);
        }
        system_core.wind_model =
            std::make_shared<seahowl::aero::InflowWindAdapter>(inflowwind_filepath, windwnd_filepath);
        auto wind_model = std::dynamic_pointer_cast<seahowl::aero::InflowWindAdapter>(system_core.wind_model);
        double dt = json_obj.at("numerics").at("dt").get<double>();
        wind_model->init(dt);
#else
        spdlog::critical("InflowWind module in CMAKE options should be enabled if wind type 'inflowwind' selected.");
        exit(1);
#endif
    } else {
        spdlog::critical("The input wind type is unknown. Please use the existing wind types: ramp or inflowwind.");
        exit(1);
    }

    // turbines
    auto turbines_json = json_obj.at("turbines");
    for (int ii = 0; ii < turbines_json.size(); ii++) {
        auto turbine_json = turbines_json[ii];
        auto filepath_turbine = (DATADIR / turbine_json.at("file").get<std::string>()).generic_string();
        // make turbine aero
        system_core.system_aero->turbines.push_back(seahowl::aero::TurbineAero());
        // check if floater defined

        auto json_obj_turbine = get_json_from_file(filepath_turbine);
        bool is_floating = false;
        if (json_obj_turbine.contains("floater")) {
            // floating turbine if floater is defined
            auto floating_turbine_elasto = std::make_shared<seahowl::elasto::TurbineFloatingElasto>();
            system_core.system_elasto->turbines.push_back(floating_turbine_elasto);
            is_floating = true;
            system_core.turbines.push_back(std::move(std::make_shared<seahowl::core::TurbineFloating>(
                *floating_turbine_elasto, system_core.system_aero->turbines[ii])));
        } else {
            system_core.system_elasto->turbines.push_back(std::make_shared<seahowl::elasto::TurbineElasto>());
            // simple turbine if floater is not defined
            system_core.turbines.push_back(std::move(std::make_shared<seahowl::core::Turbine>(
                *system_core.system_elasto->turbines[ii], system_core.system_aero->turbines[ii])));
        }
        // get reference to turbine object
        auto& turbine = *system_core.turbines.back();
        populate_turbine_from_json(filepath_turbine, turbine);

        // aerodyn option
#ifdef HAVE_AERODYN
        turbine.aero.use_aerodyn = turbine_json.at("use_aerodyn").get<bool>();
        bool output_vtk = json_obj.at("outputs").at("VTK").get<bool>();
        if (output_vtk) {
            turbine.aero.WrVTK = 2;
        }
        turbine.aero.WrVTK_dt = json_obj.at("outputs").at("dt").get<double>();
        if (turbine.aero.use_aerodyn) {
            std::string inflowwind_filepath;
            std::string aerodyn_filepath;
            if (turbine_json.contains("file_aerodyn")) {
                aerodyn_filepath = (DATADIR / turbine_json.at("file_aerodyn")).generic_string();
            } else {
                spdlog::critical("Turbine set to use aerodyn but AeroDyn file path not defined.");
                exit(1);
            }
            if (wind_json.at("options").contains("file_inflowwind")) {
                inflowwind_filepath = (DATADIR / wind_json.at("options").at("file_inflowwind")).generic_string();
            } else {
                spdlog::critical("Turbine set to use aerodyn but InflowWind file not defined.");
                exit(1);
            }
            turbine.aero.aerodyn =
                std::make_shared<seahowl::aero::AeroDynAdapter>(aerodyn_filepath, inflowwind_filepath);
        }
#endif

        // build turbine
        turbine.build();
        // rotate turbine to align tower with gravity vector
        auto v1 = Vector3d(-system_core.system_elasto->get_gravitational_acceleration()).normalized();
        auto v2 = (turbine.tower.elasto.nodes[1]->get_position() - turbine.tower.elasto.nodes[0]->get_position())
                      .normalized();
        auto rot_axis = v2.cross(v1);
        auto rot_angle = acos(v1.dot(v2));
        turbine.rotate(rot_angle, rot_axis);
        // rotation around axis opposite to gravity (yaw)
        turbine.rotate(turbine_json.at("rotation").get<double>(),
                       Vector3d(-system_core.system_elasto->get_gravitational_acceleration()).normalized());
        // translate turbine
        auto trans = turbine_json.at("translation").get<std::vector<double>>();
        turbine.translate(Vector3d(trans[0], trans[1], trans[2]));

        // apply initial pitches
        for (auto& blade : turbine.rna.blades) {
            auto pitch0 = blade->elasto.pitch;
            blade->elasto.apply_pitch_increment(pitch0);
            blade->elasto.pitch = pitch0;
        }
        auto rotor_pitch0 = turbine.rna.elasto.rotor->pitch_collective;
        turbine.rna.elasto.rotor->apply_collective_pitch_increment(rotor_pitch0);
        turbine.rna.elasto.rotor->pitch_collective = rotor_pitch0;

        if (is_floating) {
            turbine.tower.elasto.nodes.front()->set_fixed(false);
        } else {
            turbine.tower.elasto.nodes.front()->set_fixed(true);
        }
    }

    // assemble whole system (Chrono)
    system_core.assemble();
}
