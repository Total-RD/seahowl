#include "seahowl/io/read_json.h"
#include "seahowl/io/utils_io.h"
#include "seahowl/io/read_rotor_perf.h"

#include "seahowl/commons/utils.h"
#include "seahowl/core/blade.h"
#include "seahowl/core/rotor.h"
#include "seahowl/core/tower.h"
#include "seahowl/core/monopile.h"
#include "seahowl/core/turbine.h"
#include "seahowl/core/system.h"
#include "seahowl/elasto/tower_elasto.h"
#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/reference_point_elasto.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/elasto/monopile_elasto.h"
#include "seahowl/servo/controller_discon.h"
#include "seahowl/commons/numerics.h"
#include "seahowl/env/fluid_models.h"
#include "seahowl/env/wind_models.h"
#include "seahowl/env/wave_models.h"
#include "seahowl/env/soil_models.h"
#include "seahowl/env/combined_models.h"
#include "seahowl/hydro/morison.h"
#ifdef HAVE_INFLOWWIND
    #include "seahowl/env/inflowwind_adapter.h"
#endif
#include "seahowl/aero/airfoil.h"
#include "seahowl/aero/blade_aero.h"
#include "seahowl/aero/rotor_aero.h"
#include "seahowl/aero/turbine_aero.h"
#include "seahowl/hydro/monopile_hydro.h"
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

#include "seahowl/io/json_db.h"
#include "seahowl/io/store_db_models.h"

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

namespace seahowl {
namespace io {

json get_json_from_file(const std::string& filepath) {
    utils::check_file_exists(filepath);
    std::ifstream json_file(filepath);
    json json_obj;
    json_file >> json_obj;
    json_file.close();
    return json_obj;
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
            mat(irow, icol) = matvec[irow][icol];
        }
    }
    return mat;
}

class InputData {
  public:
    int nrows = 0;
    std::string filepath;
    virtual void open(const std::string& filepath) = 0;
    virtual double get(const std::string& key, int row) = 0;
    virtual Vector3d get_vector3(const std::string& key, int row) = 0;
    virtual Vector2d get_vector2(const std::string& key, int row) = 0;
    virtual std::vector<double> get_vector_std(size_t length, const std::string& key, int row) = 0;
    virtual std::string get_string(const std::string& key, int row) = 0;
    virtual Eigen::MatrixX<double> get_matrix(const std::string& key, int row) = 0;
};

class InputDataCSV : public InputData {
    std::map<std::string, std::vector<std::string>> csv_data;

  public:
    void open(const std::string& filepath) override {
        this->filepath = filepath;
        utils::check_file_exists(filepath);
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
                csv_data[csv_headers[idx_word]].push_back(word);
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
            return std::stod(csv_data[key][row]);
        } else {
            throw std::runtime_error("Header " + key + " does not exist in CSV file " + filepath + ".");
        }
    }

    Vector3d get_vector3(const std::string& key, int row) override {
        std::vector<double> vec;
        std::vector<std::string> dim_keys = {key + "_x", key + "_y", key + "_z"};
        for (auto& dim_key : dim_keys) {
            if (csv_data.count(dim_key) > 0) {
                vec.push_back(std::stod(csv_data[dim_key][row]));
            } else {
                throw std::runtime_error("Could not find '" + dim_key + "' in CSV file " + filepath + ".");
            }
        }
        return Vector3d(vec[0], vec[1], vec[2]);
    }

    Vector2d get_vector2(const std::string& key, int row) override {
        std::vector<double> vec;
        std::vector<std::string> dim_keys = {key + "_x", key + "_y"};
        for (auto& dim_key : dim_keys) {
            if (csv_data.count(dim_key) > 0) {
                vec.push_back(std::stod(csv_data[dim_key][row]));
            } else {
                throw std::runtime_error("Could not find '" + dim_key + "' in CSV file " + filepath + ".");
            }
        }
        return Vector2d(vec[0], vec[1]);
    }

    std::vector<double> get_vector_std(size_t length, const std::string& key, int row) override {
        std::vector<double> vec;
        std::vector<std::string> dim_keys;
        for (size_t ii = 0; ii < length; ii++) {
            dim_keys.push_back(key + "_dim" + std::to_string(ii + 1));
        }
        for (auto& dim_key : dim_keys) {
            if (csv_data.count(dim_key) > 0) {
                vec.push_back(std::stod(csv_data[dim_key][row]));
            } else {
                throw std::runtime_error("Could not find '" + dim_key + "' in CSV file " + filepath + ".");
            }
        }
        return vec;
    }

    std::string get_string(const std::string& key, int row) override { return csv_data[key][row]; }

    Eigen::MatrixX<double> get_matrix(const std::string& key, int row) override {
        throw std::runtime_error("Cannot get matrix '" + key + "' from CSV file.");
    }
};

class InputDataJSON : public InputData {
    json json_rows;
    json json_globals;

  public:
    void open(const std::string& filepath) override {
        this->filepath = filepath;
        utils::check_file_exists(filepath);
        std::ifstream json_file(filepath);
        json json_data;
        json_file >> json_data;
        json_file.close();
        json_globals = json_data["global_variables"];
        json_rows = json_data["reference_points"];
        nrows = json_rows.size();
    }

    double get(const std::string& key, int row) override {
        if (json_rows[row].contains(key)) {
            return json_rows[row].at(key).get<double>();
        } else if (json_globals.contains(key)) {
            return json_globals.at(key).get<double>();
        } else {
            throw std::runtime_error("Could not find '" + key + "' in JSON file " + filepath + ".");
        }
    }

    Vector3d get_vector3(const std::string& key, int row) override {
        auto vec = get_vector_std(3, key, row);
        if (vec.size() != 3) {
            throw std::runtime_error("Key '" + key + "' has to be a vector of length 3 in JSON file " + filepath + ".");
        }
        return Vector3d(vec[0], vec[1], vec[2]);
    }

    Vector2d get_vector2(const std::string& key, int row) override {
        auto vec = get_vector_std(2, key, row);
        if (vec.size() != 2) {
            throw std::runtime_error("Key '" + key + "' has to be a vector of length 2 in JSON file " + filepath + ".");
        }
        return Vector2d(vec[0], vec[1]);
    }

    std::vector<double> get_vector_std(size_t length, const std::string& key, int row) override {
        std::vector<double> vec;
        if (json_rows[row].contains(key)) {
            return json_rows[row].at(key).get<std::vector<double>>();
        } else if (json_globals.contains(key)) {
            return json_globals.at(key).get<std::vector<double>>();
        } else {
            throw std::runtime_error("Could not find '" + key + "' in JSON file " + filepath + ".");
        }
    }

    std::string get_string(const std::string& key, int row) override {
        std::string str;
        if (json_rows[row].contains(key)) {
            return json_rows[row].at(key).get<std::string>();
        } else if (json_globals.contains(key)) {
            return json_globals.at(key).get<std::string>();
        } else {
            throw std::runtime_error("Could not find '" + key + "' in JSON file " + filepath + ".");
        }
    }
    Eigen::MatrixX<double> get_matrix(const std::string& key, int row) override {
        std::vector<std::vector<double>> matvec;
        if (json_rows[row].contains(key)) {
            matvec = json_rows[row].at(key).get<std::vector<std::vector<double>>>();
        } else if (json_globals.contains(key)) {
            matvec = json_globals.at(key).get<std::vector<std::vector<double>>>();
        } else {
            throw std::runtime_error("Could not find '" + key + "' in JSON file " + filepath + ".");
        }

        return get_matrix_from_vector_of_vectors(matvec);
    }
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
    BladeDb blade_db = read_blade_db(filepath);
    return get_blade_elasto_reference_points_from_db(blade_db);
}

std::vector<seahowl::elasto::BladeReferencePointElasto> get_blade_elasto_reference_points_from_db(
    const BladeDb& blade_db) {
    // EXTRACT INFO
    std::vector<seahowl::elasto::BladeReferencePointElasto> reference_points;

    double blade_length = blade_db.reference_points.back().coordinates.z();

    for (auto& ref_point : blade_db.reference_points) {
        auto reference_point = seahowl::elasto::BladeReferencePointElasto();
        reference_point.coordinates = ref_point.coordinates;
        reference_point.fraction = reference_point.coordinates.z() / blade_length;
        reference_point.offset_gravity = blade_db.global_variables.offset_gravity;
        reference_point.offset_elastic = blade_db.global_variables.offset_elastic;
        reference_point.structural_twist = ref_point.twist * PI / 180.0;
        reference_point.mass_matrix = ref_point.mass_matrix;
        reference_point.stiffness_matrix = ref_point.stiffness_matrix;
        reference_point.damping_flapwise = blade_db.global_variables.damping_flapwise;
        reference_point.damping_edgewise = blade_db.global_variables.damping_edgewise;
        reference_point.damping_axial = blade_db.global_variables.damping_axial;
        reference_point.damping_torsion = blade_db.global_variables.damping_torsion;
        reference_points.push_back(reference_point);
    }
    return reference_points;
}

std::vector<seahowl::aero::BladeReferencePointAero> get_blade_aero_reference_points_from_json(
    const std::string& filepath) {
    BladeDb blade_db = read_blade_db(filepath);
    return get_blade_aero_reference_points_from_db(blade_db);
}

std::vector<seahowl::aero::BladeReferencePointAero> get_blade_aero_reference_points_from_db(const BladeDb& blade_db) {
    // EXTRACT INFO
    std::vector<seahowl::aero::BladeReferencePointAero> reference_points;

    double blade_length = blade_db.reference_points.back().coordinates.z();

    for (auto& ref_point_db : blade_db.reference_points) {
        auto reference_point = seahowl::aero::BladeReferencePointAero();
        reference_point.coordinates = ref_point_db.coordinates;
        reference_point.fraction = reference_point.coordinates.z() / blade_length;
        reference_point.structural_twist = ref_point_db.twist * PI / 180.0;
        reference_point.offset_aero = ref_point_db.offset_aero;
        reference_point.chord = ref_point_db.chord;

        if (!ref_point_db.airfoil_file.empty()) {
            for (const auto& airfoil_db : ref_point_db.airfoil_db_list) {
                std::vector<seahowl::aero::AirfoilCoefficients> coefficients_list;
                for (const auto& coeff_db : airfoil_db.coefficients) {
                    if (coeff_db.size() != 4) {
                        throw std::runtime_error("Airfoil coefficients has to be vectors of length 4.");
                    }
                    seahowl::aero::AirfoilCoefficients coefficients;
                    coefficients.alpha = coeff_db[0];
                    coefficients.lift = coeff_db[1];
                    coefficients.drag = coeff_db[2];
                    coefficients.moment = coeff_db[3];
                    coefficients_list.push_back(coefficients);
                }
                seahowl::aero::AirfoilProperties airfoil;
                airfoil.reynolds_number = airfoil_db.reynolds_number;
                airfoil.coefficients_list = coefficients_list;
                reference_point.airfoil_properties.push_back(airfoil);
            }
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
    BladeDb blade_db = read_blade_db(filepath);
    blade.elasto.reference_points = get_blade_elasto_reference_points_from_db(blade_db);
    blade.aero.reference_points = get_blade_aero_reference_points_from_db(blade_db);
}

std::vector<seahowl::elasto::TowerReferencePointElasto> get_tower_elasto_reference_points(const TowerDb& tower_db) {
    // EXTRACT INFO
    std::vector<seahowl::elasto::TowerReferencePointElasto> reference_points;

    Vector3d pos0 = tower_db.reference_points[0].position;
    Vector3d pos1 = tower_db.reference_points[tower_db.reference_points.size() - 1].position;
    double length = (pos1 - pos0).norm();

    // MAKE TOWER REFERENCE POINTS
    for (auto point_db : tower_db.reference_points) {
        auto reference_point = seahowl::elasto::TowerReferencePointElasto();
        reference_point.coordinates = point_db.position;
        reference_point.fraction = (reference_point.coordinates - pos0).norm() / length;

        // general properties
        // shear set to false as it leads to issues when tower is not finely discretized (wrong nat. freq.)
        // its effect is usually small enough to be neglected here
        reference_point.set_properties_cylinder(point_db.density, point_db.young_modulus, point_db.poisson_ratio,
                                                point_db.diameter, point_db.thickness, false);

        reference_point.damping_foreaft = point_db.damping_foreaft;
        reference_point.damping_sideside = point_db.damping_sideside;
        reference_point.damping_axial = point_db.damping_axial;
        reference_point.damping_torsion = point_db.damping_torsion;
        reference_point.damping_mass = point_db.damping_mass;

        reference_points.push_back(reference_point);
    }

    return reference_points;
}

std::vector<seahowl::aero::TowerReferencePointAero> get_tower_aero_reference_points(const TowerDb& tower_db) {
    // EXTRACT INFO
    Vector3d pos0 = tower_db.reference_points[0].position;
    Vector3d pos1 = tower_db.reference_points[tower_db.reference_points.size() - 1].position;
    double length = (pos1 - pos0).norm();

    // MAKE TOWER REFERENCE POINTS
    std::vector<seahowl::aero::TowerReferencePointAero> reference_points;
    for (auto point_db : tower_db.reference_points) {
        auto reference_point = seahowl::aero::TowerReferencePointAero();
        reference_point.coordinates = point_db.position;
        reference_point.fraction = (reference_point.coordinates - pos0).norm() / length;

        reference_point.diameter = point_db.diameter;
        reference_point.coefficients.drag_normal = point_db.drag_coefficient_normal;
        reference_point.coefficients.drag_axial = point_db.drag_coefficient_axial;
        reference_point.coefficients.added_mass_normal = point_db.added_mass_coefficient_normal;
        reference_point.coefficients.added_mass_axial = point_db.added_mass_coefficient_axial;
        reference_point.coefficients.buoyancy_factor = point_db.buoyancy_factor;

        reference_points.push_back(reference_point);
    }

    return reference_points;
}

void populate_tower_elasto_from_json(const std::string& filepath, seahowl::elasto::TowerElasto& tower) {
    TowerDb tower_db = seahowl::io::read_tower_db(filepath);
    tower.reference_points = get_tower_elasto_reference_points(tower_db);
    tower.height = tower.reference_points.back().coordinates.z();
    tower.base_height = tower.reference_points.front().coordinates.z();
}

void populate_tower_aero_from_json(const std::string& filepath, seahowl::aero::TowerAero& tower) {
    utils::check_file_exists(filepath);
    TowerDb tower_data = seahowl::io::read_tower_db(filepath);
    tower.reference_points = get_tower_aero_reference_points(tower_data);
}

void populate_tower_from_json(const std::string& filepath, seahowl::core::Tower& tower) {
    spdlog::debug("Populating tower from " + filepath + " file (absolute: " + absolute(path(filepath)).string() + ").");
    TowerDb tower_data = seahowl::io::read_tower_db(filepath);
    // elasto
    tower.elasto.reference_points = get_tower_elasto_reference_points(tower_data);
    tower.elasto.height = tower.elasto.reference_points.back().coordinates.z();
    tower.elasto.base_height = tower.elasto.reference_points.front().coordinates.z();
    // aero
    tower.aero.reference_points = get_tower_aero_reference_points(tower_data);
}

void populate_rna_elasto_from_json(const std::string& filepath, seahowl::elasto::RotorNacelleAssemblyElasto& rna) {
    RnaDb rna_db = read_rna_db(filepath);
    populate_rna_elasto_from_db(rna_db, rna);
}

void populate_rna_aero_from_json(const std::string& filepath, seahowl::aero::RotorNacelleAssemblyAero& rna) {
    RnaDb rna_db = read_rna_db(filepath);
    populate_rna_aero_from_db(rna_db, rna);
}

void populate_rna_elasto_from_db(const RnaDb& rna_db, seahowl::elasto::RotorNacelleAssemblyElasto& rna) {
    // EXTRACT INFO
    // rotor
    rna.rotor->hub.position_from_apex = rna_db.hub.position_from_apex;
    rna.rotor->hub.mass = rna_db.hub.mass;
    rna.rotor->hub.inertia = rna_db.hub.inertia;
    rna.rotor->hub.overhang = rna_db.hub.overhang;
    rna.rotor->hub.radius = rna_db.hub.radius;
    // nacelle
    rna.nacelle.position_from_towertop = rna_db.nacelle.position_from_towertop;
    if (rna.nacelle.position_from_towertop.size() != 3) {
        throw std::runtime_error("Center of mass of nacelle has to be vector of length 3.");
    }
    rna.nacelle.mass = rna_db.nacelle.mass;
    rna.nacelle.inertia = rna_db.nacelle.inertia;
    rna.nacelle.yaw_bearing_mass = rna_db.nacelle.yaw_bearing_mass;
    // shaft
    rna.shaft.distance_from_towertop = rna_db.shaft.distance_from_towertop;
    rna.shaft.tilt = rna_db.shaft.tilt;
    // convert to radians
    rna.shaft.tilt *= PI / 180.0;
}

void populate_rna_aero_from_db(const RnaDb& rna_db, seahowl::aero::RotorNacelleAssemblyAero& rna) {
    // EXTRACT INFO
    // hub
    rna.rotor->hub_radius = rna_db.hub.radius;
}

void populate_rna_from_json(const std::string& filepath, seahowl::core::RotorNacelleAssembly& rna) {
    spdlog::debug("Populating RNA from " + filepath + " file (absolute: " + absolute(path(filepath)).string() + ").");
    RnaDb rna_db = read_rna_db(filepath);
    populate_rna_elasto_from_db(rna_db, rna.elasto);
    populate_rna_aero_from_db(rna_db, rna.aero);
}

void populate_rna_from_db(const RnaDb& rna_db, seahowl::core::RotorNacelleAssembly& rna) {
    populate_rna_elasto_from_db(rna_db, rna.elasto);
    populate_rna_aero_from_db(rna_db, rna.aero);
}

void add_turbine_to_system_from_json(const std::string& filepath, seahowl::core::System& system_core) {
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
    populate_turbine_from_json(filepath, *turbine);
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

void populate_turbine_from_json(const std::string& filepath, seahowl::core::Turbine& turbine) {
    spdlog::debug("Populating turbine from " + filepath + " file (absolute: " + absolute(path(filepath)).string() +
                  ").");
    auto json_obj = get_json_from_file(filepath);
    auto DATADIR = path(filepath).parent_path();

    TurbineDb turbine_db = read_turbine_db(filepath);

    if (turbine_db.aero.solver == "bemt") {
        spdlog::info("Aerodynamic model: Blade Element Momentum Theory (BEMT).");
        auto rotor_aero = std::make_shared<seahowl::aero::RotorAeroBEMT>(turbine.aero.tower);
        turbine.rna.aero.rotor = rotor_aero;

        rotor_aero->has_hub_loss = turbine_db.aero.options.hub_loss;
        rotor_aero->has_tip_loss = turbine_db.aero.options.tip_loss;
        rotor_aero->has_tower_shadow = turbine_db.aero.options.tower_shadow;

    } else if (turbine_db.aero.solver == "disk") {
        spdlog::info("Aerodynamic model: Actuator Disk Theory.");
        if (turbine_db.rotor.type != "disk") {
            throw std::runtime_error("Only rotor type \"disk\" can be used with aero solver \"disk\".");
        }
        auto rotor_disk = std::make_shared<seahowl::aero::RotorAeroDisk>();
        turbine.rna.aero.rotor = rotor_disk;
        // get rotor performance from table
        auto perf_filepath = (DATADIR / turbine_db.aero.options.performance_file).generic_string();
        get_disk_perf_from_table(perf_filepath, *rotor_disk);
    } else if (turbine_db.aero.solver == "aerodyn") {
        spdlog::info("Aerodynamic model: AeroDyn.");
#ifdef HAVE_AERODYN
        // check that right turbine type was defined for AeroDyn
        try {
            auto& turbine_aero = dynamic_cast<seahowl::aero::TurbineAeroDyn&>(turbine.aero);
        } catch (const std::exception& e) {
            throw std::runtime_error("Wrong aero turbine type to use AeroDyn solver (needs to be TurbineAeroDyn).");
        }
        auto& turbine_aero = dynamic_cast<seahowl::aero::TurbineAeroDyn&>(turbine.aero);
        std::string inflowwind_filepath;
        std::string aerodyn_filepath;
        if (!turbine_db.aero.options.file_aerodyn.empty()) {
            aerodyn_filepath = (DATADIR / turbine_db.aero.options.file_aerodyn).generic_string();
        } else {
            throw std::runtime_error("Turbine set to use aerodyn but AeroDyn file path not defined.");
        }
        if (!turbine_db.aero.options.file_inflowwind.empty()) {
            inflowwind_filepath = (DATADIR / turbine_db.aero.options.file_inflowwind).generic_string();
        } else {
            throw std::runtime_error("Turbine set to use aerodyn but InflowWind file not defined.");
        }
        turbine_aero.aerodyn.set_infiles(aerodyn_filepath, inflowwind_filepath);
        turbine_aero.rna.rotor = std::make_shared<seahowl::aero::RotorAeroDyn>(turbine.aero.tower);
#else
        throw std::runtime_error("Trying to use AeroDyn for turbine but the code was not compiled for using AeroDyn.");
#endif
    } else {
        throw std::runtime_error("Unknown aero solver \"" + turbine_db.aero.solver + "\".");
    }

    // check rotor type
    if (turbine_db.rotor.type == "fea") {
        spdlog::info("Rotor type: finite element blades.");
    } else if (turbine_db.rotor.type == "fpm") {
        spdlog::info("Rotor type: finite element blades (FPM).");
    } else if (turbine_db.rotor.type == "rigid") {
        spdlog::info("Rotor type: rigid.");
    } else if (turbine_db.rotor.type == "disk") {
        spdlog::info("Rotor type: disk");
    } else {
        throw std::runtime_error("Rotor type does not exist: try \"fea\", \"fpm\", \"rigid\", or \"disk\".");
    }

    // blades
    // only make blades if rotor type is not disk
    if (turbine_db.rotor.type != "disk") {
        // pitch actuator dynamics
        bool has_pitch_actuator_dynamics = turbine_db.rotor.pitch_actuator_dynamics;

        std::vector<std::shared_ptr<seahowl::core::Blade>> blades;
        std::vector<std::shared_ptr<seahowl::elasto::BladeElasto>> blades_elasto;
        std::vector<std::shared_ptr<seahowl::aero::BladeAero>> blades_aero;

        for (auto& blade_db : turbine_db.rotor.blades) {
            auto filepath_blade = (DATADIR / blade_db.file).generic_string();
            std::shared_ptr<seahowl::elasto::BladeElasto> blade_elasto;
            if (turbine_db.rotor.type == "fea" || turbine_db.rotor.type == "fpm") {
                blade_elasto = std::make_shared<seahowl::elasto::BladeElastoFEA>();
                auto& blade_elasto_fea = dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade_elasto);
                blade_elasto_fea.discretization_fractions = turbine_db.rotor.discretization.elasto;
                if (turbine_db.rotor.type == "fpm") {
                    blade_elasto_fea.fpm_mode = true;
                } else if (turbine_db.rotor.type == "fea") {
                    blade_elasto_fea.fpm_mode = false;
                }
            } else if (turbine_db.rotor.type == "rigid") {
                blade_elasto = std::make_shared<seahowl::elasto::BladeElastoRigid>();
            }
            auto blade_aero = std::make_shared<seahowl::aero::BladeAero>();
            auto blade = std::make_shared<seahowl::core::Blade>(*blade_elasto, *blade_aero);
            populate_blade_from_json(filepath_blade, *blade);
            blade->aero.discretization_fractions = turbine_db.rotor.discretization.aero;
            blade_elasto->pitch0 = blade_db.initial_pitch * PI / 180.0;
            blade_elasto->precone = blade_db.precone * PI / 180.0;
            // no precone if blade is rigid (assumed that blade is on rotor disc)
            if (turbine_db.rotor.type == "rigid") {
                blade_elasto->precone = 0.0;
            }

            blade_elasto->actuator_pitch->set_fixed_actuator(!has_pitch_actuator_dynamics);
            blades_elasto.push_back(blade_elasto);
            blades_aero.push_back(blade_aero);
            blades.push_back(blade);
        }
        turbine.elasto.rna.rotor->blades = blades_elasto;
        turbine.aero.rna.rotor->blades = blades_aero;
        turbine.rna.blades = blades;
    }

    // RNA
    auto filepath_rna = (DATADIR / turbine_db.rna.file).generic_string();
    spdlog::debug("Populating RNA from " + filepath_rna + " file (absolute: " + absolute(path(filepath_rna)).string() +
                  ").");
    RnaDb rna_db = read_rna_db(filepath_rna);
    populate_rna_from_db(rna_db, turbine.rna);
    auto yaw_rna = turbine_db.rna.initial_yaw * PI / 180.0;
    turbine.rna.elasto.yaw0 = yaw_rna;
    bool has_yaw_actuator_dynamics = turbine_db.rna.yaw_actuator_dynamics;
    turbine.rna.elasto.actuator_yaw->set_fixed_actuator(!has_yaw_actuator_dynamics);

    // update info if rotor is disk
    if (turbine_db.rotor.type == "disk") {
        turbine.rna.elasto.rotor->hub.inertia(0, 0) += turbine_db.rotor.option.inertia_blades;
        turbine.rna.elasto.rotor->hub.mass += turbine_db.rotor.option.mass_blades;
        turbine.rna.aero.rotor->radius = turbine_db.rotor.option.radius;
    }

    // tower
    auto filepath_tower = (DATADIR / turbine_db.tower.file).generic_string();
    populate_tower_from_json(filepath_tower, turbine.tower);
    turbine.elasto.tower.discretization_fractions = turbine_db.tower.discretization.elasto;
    turbine.aero.tower.discretization_fractions = turbine_db.tower.discretization.aero;
    if (turbine_db.tower.options.has_value()) {
        if (turbine_db.tower.options.value().use_MacCamyFuchs_correction.has_value()) {
            turbine.aero.tower.use_MacCamyFuchs_correction =
                turbine_db.tower.options.value().use_MacCamyFuchs_correction.value();
        }

        if (turbine_db.tower.options.value().use_Cd_correction.has_value()) {
            turbine.aero.tower.use_Cd_correction = turbine_db.tower.options.value().use_Cd_correction.value();
        }
    }
    // controller
    if (turbine_db.controller.type == "DISCON") {
        auto libfilepath = turbine_db.controller.options.libfile;
        if (libfilepath != "") {
            // path
            libfilepath = path(DATADIR / libfilepath).generic_string();
        }
        auto infilepath = turbine_db.controller.options.infile;
        if (infilepath != "") {
            infilepath = (DATADIR / infilepath).generic_string();
        }
        // instantiate controller
        auto controller = std::make_shared<seahowl::servo::ControllerDISCON>(infilepath, libfilepath);
        turbine.controller = controller;

    } else if (turbine_db.controller.type == "RPM") {
        auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
        controller->target_rpm = turbine_db.controller.options.target_rpm;
        turbine.controller = controller;
    }

    // get extra drivetrain info
    // gearbox
    turbine.gearbox_ratio = rna_db.drivetrain.gearbox_ratio;
    turbine.gearbox_efficiency = rna_db.drivetrain.gearbox_efficiency;
    turbine.gearbox_efficiency /= 100.0;
    // generator
    turbine.generator_efficiency = rna_db.drivetrain.generator_efficiency;
    turbine.generator_efficiency /= 100.0;
    // add inertia of generator to hub directly
    double drivetrain_inertia = rna_db.drivetrain.generator_inertia;
    turbine.rna.elasto.rotor->hub.inertia(0, 0) += drivetrain_inertia;

    if (turbine_db.foundation.has_value()) {
        auto& turbine_elasto = turbine.elasto;

        auto& foundation_db = turbine_db.foundation.value();
        if (foundation_db.type == "monopile") {
            // create monopile
            auto monopile_elasto = std::make_shared<seahowl::elasto::MonopileElasto>();
            auto monopile_hydro = std::make_shared<seahowl::hydro::MonopileHydro>();
            auto monopile_core = std::make_shared<seahowl::core::Monopile>(*monopile_elasto, *monopile_hydro);
            turbine.elasto.foundation = monopile_elasto;
            turbine.aero.foundation = monopile_hydro;
            turbine.foundation = monopile_core;

            // populate monopile
            auto filepath_monopile = (DATADIR / foundation_db.file.value()).generic_string();
            populate_tower_from_json(filepath_monopile, *monopile_core);
            monopile_elasto->discretization_fractions = foundation_db.discretization.elasto;
            monopile_hydro->discretization_fractions = foundation_db.discretization.aero;

            if (foundation_db.options.has_value()) {
                auto& options = foundation_db.options.value();
                if (options.use_MacCamyFuchs_correction.has_value()) {
                    monopile_hydro->use_MacCamyFuchs_correction = options.use_MacCamyFuchs_correction.value();
                }
                if (options.use_Cd_correction.has_value()) {
                    monopile_hydro->use_Cd_correction = options.use_Cd_correction.value();
                }
            }

        } else if (foundation_db.type == "floater") {
            auto floater_hydro_ptr = std::make_shared<seahowl::hydro::FloaterHydro>();
            turbine.aero.foundation = floater_hydro_ptr;
            auto& floater_hydro = *floater_hydro_ptr;

            if (foundation_db.file.has_value()) {
                auto json_obj_floater = get_json_from_file((DATADIR / foundation_db.file.value()).generic_string());

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
                    throw std::runtime_error(
                        "Trying to use HydroChrono but did not compile with HydroChrono dependency.");
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
                    throw std::runtime_error(
                        "Viscous damping matrix for floater body has to be defined as 6x6 matrices.");
                }
                Eigen::Matrix<double, 6, 6> damping_matrix;
                for (int irow = 0; irow < 6; irow++) {
                    if (dm[irow].size() != 6) {
                        throw std::runtime_error(
                            "Viscous damping matrix for floater body has to be defined as 6x6 matrices.");
                    }
                    for (int icol = 0; icol < 6; icol++) {
                        damping_matrix(irow, icol) = dm[irow][icol];
                    }
                }
                floater_elasto.body_main->set_damping_matrix(damping_matrix);

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
                    floater_elasto.mooring_system->anchors.push_back(
                        std::make_shared<seahowl::elasto::BodyElastoChrono>());
                    auto& anchor_body = *floater_elasto.mooring_system->anchors.back();
                    anchor_body.set_position(anchor_position);
                    anchor_body.set_mass(0.0);
                    anchor_body.set_inertia_diagonal(Vector3d(0.0, 0.0, 0.0));
                    anchor_body.set_fixed(true);

                    auto mooring_properties_json = get_json_from_file(
                        (DATADIR / mooring_json.at("line_properties").get<std::string>()).generic_string());

                    // elasto
                    floater_elasto.mooring_system->moorings.push_back(
                        std::make_shared<seahowl::elasto::MooringElastoFEA>(fairlead_body, anchor_body));
                    auto& mooring_elasto = dynamic_cast<seahowl::elasto::MooringElastoFEA&>(
                        *floater_elasto.mooring_system->moorings.back());
                    mooring_json.at("length").get_to(mooring_elasto.length);
                    mooring_properties_json.at("diameter").get_to(mooring_elasto.diameter);
                    mooring_json.at("discretization").at("elasto").get_to(mooring_elasto.discretization_fractions);
                    mooring_properties_json.at("stiffness_axial").get_to(mooring_elasto.stiffness_axial);
                    mooring_properties_json.at("stiffness_bending").get_to(mooring_elasto.stiffness_bending);
                    mooring_properties_json.at("density_linear").get_to(mooring_elasto.density_linear);

                    // hydro
                    floater_hydro.mooring_system->moorings.push_back(std::make_shared<seahowl::hydro::MooringHydro>());
                    auto& mooring_hydro = *floater_hydro.mooring_system->moorings.back();
                    mooring_json.at("length").get_to(mooring_hydro.length);
                    mooring_properties_json.at("diameter").get_to(mooring_hydro.diameter);
                    mooring_json.at("discretization").at("hydro").get_to(mooring_hydro.discretization_fractions);
                    mooring_properties_json.at("drag_coefficient_normal")
                        .get_to(mooring_hydro.coefficients.drag_normal);
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
}

std::shared_ptr<seahowl::env::FluidSoilModel> get_environmental_model_from_json(const std::string& filepath) {
    spdlog::debug("Getting environmental conditions from " + filepath + " file.");
    auto DATADIR = path(filepath).parent_path();
    EnvironmentDb environment_db = read_environment_db(filepath);

    return get_environmental_model(environment_db, DATADIR);
}

std::shared_ptr<seahowl::env::FluidSoilModel> get_environmental_model(const EnvironmentDb& environment_db,
                                                                      const fs::path& DATADIR) {
    // gravity
    auto gravity_vector = environment_db.gravity;
    auto gravity_direction = gravity_vector / gravity_vector.norm();

    // environment
    auto env_model = std::make_shared<seahowl::env::FluidSoilModel>();

    // sea
    bool has_sea = false;
    if (environment_db.sea.has_value()) {
        has_sea = true;
        auto sea_db = environment_db.sea.value();
        env_model->fluid_model = std::make_shared<seahowl::env::WaveWindModel>();
        auto& fluid_model = dynamic_cast<seahowl::env::WaveWindModel&>(*env_model->fluid_model);
        // specific model options
        if (sea_db.type == "still") {
            spdlog::info("Sea conditions: still water.");
            fluid_model.wave_model = std::make_unique<seahowl::env::StillWater>();
        } else if (sea_db.type == "HydroChrono" || sea_db.type == "hydrochrono") {
            spdlog::info("Sea conditions: HydroChrono.");
#ifdef HAVE_HYDROCHRONO
            fluid_model.wave_model = std::make_unique<seahowl::env::WaveModelHydroChrono>();
            auto& wave_model = dynamic_cast<seahowl::env::WaveModelHydroChrono&>(*fluid_model.wave_model);

            auto wave_type = sea_db.options.type;
            if (wave_type == "still") {
                wave_model.waves = std::make_shared<NoWave>();
            } else if (wave_type == "regular") {
                auto hydrochrono_waves = std::make_shared<RegularWave>();
                wave_model.waves = hydrochrono_waves;
                hydrochrono_waves->regular_wave_amplitude_ = sea_db.options.wave_height / 2.0;
                hydrochrono_waves->regular_wave_omega_ = 2 * PI / sea_db.options.wave_period;
                seahowl::hydro::myMCFtable = seahowl::hydro::MacCamyFuchsTable();
                seahowl::hydro::myMCFtable.wave_peak_period = sea_db.options.wave_period;
            } else if (wave_type == "irregular") {
                auto params = IrregularWaveParams();
                params.num_bodies_ = sea_db.options.num_bodies;
                params.wave_height_ = sea_db.options.wave_height;
                params.wave_period_ = sea_db.options.wave_period;
                params.frequency_min_ = sea_db.options.frequency_min;
                params.frequency_max_ = sea_db.options.frequency_max;
                params.nfrequencies_ = sea_db.options.nfrequencies;
                params.peak_enhancement_factor_ = sea_db.options.peak_enhancement_factor;
                params.is_normalized_ = sea_db.options.is_normalized;
                params.seed_ = sea_db.options.seed;
                params.simulation_dt_ = sea_db.options.dt;
                params.simulation_duration_ = sea_db.options.duration;
                params.wave_stretching_ = sea_db.options.wave_stretching;
                params.ramp_duration_ = 0.0;
                params.num_bodies_ = 1;
                wave_model.waves = std::make_shared<IrregularWaves>(params);
                seahowl::hydro::myMCFtable = seahowl::hydro::MacCamyFuchsTable();
                seahowl::hydro::myMCFtable.wave_peak_period = sea_db.options.wave_period;
            } else {
                throw std::runtime_error("Unrecognized wave type \"" + wave_type + "\" for HydroChrono.");
            }
            wave_model.waves->mwl_ = sea_db.mean_water_level;
            wave_model.waves->water_depth_ = sea_db.water_depth;
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
        } else if (sea_db.type == "current") {
            spdlog::info("Sea conditions: current.");
            fluid_model.wave_model = std::make_unique<seahowl::env::CurrentConstant>();
            auto& wave_model = dynamic_cast<seahowl::env::CurrentConstant&>(*fluid_model.wave_model);
            auto sea_direction = sea_db.options.direction;
            wave_model.direction = Vector3d(sea_direction[0], sea_direction[1], 0.0);
            wave_model.velocity_surface = sea_db.options.velocity_surface;
            wave_model.velocity_seabed = sea_db.options.velocity_seabed;
        } else {
            throw std::runtime_error(
                "The input sea type is unknown. Please use the existing current types: still, current, HydroChrono.");
        }

        // general wave model options
        auto& wave_model = dynamic_cast<seahowl::env::WaveModel&>(*fluid_model.wave_model);
        wave_model.density = sea_db.water_density;
        wave_model.mean_water_level = sea_db.mean_water_level;
        wave_model.water_depth = sea_db.water_depth;
        wave_model.surface_normal = -gravity_direction;
    } else {
        spdlog::warn("Sea conditions were not defined.");
    }

    // wind
    auto wind_db = environment_db.wind;
    std::shared_ptr<seahowl::env::WindModel> wind_model_ptr;
    if (wind_db.type == "ramp") {
        spdlog::info("Wind conditions: wind ramp.");
        wind_model_ptr = std::make_shared<seahowl::env::WindRamp>();
        auto& wind_model = dynamic_cast<seahowl::env::WindRamp&>(*wind_model_ptr);

        wind_model.set_wind_ramp(wind_db.options.velocity_start, wind_db.options.time_start,
                                 wind_db.options.velocity_end, wind_db.options.time_end);
        wind_model.direction_gravity = gravity_vector.normalized();
        wind_model.reference_height = wind_db.options.reference_height;
        wind_model.shear_coefficient = wind_db.options.shear_coefficient;
        wind_model.density = wind_db.air_density;
    } else if (wind_db.type == "inflowwind") {
        spdlog::info("Wind conditions: InflowWind.");
#ifdef HAVE_INFLOWWIND
        std::string inflowwind_filepath;

        inflowwind_filepath = (DATADIR / wind_db.options.file_inflowwind).generic_string();

        auto ifw_model = std::make_shared<seahowl::env::InflowWindAdapter>(inflowwind_filepath);
        wind_model_ptr = ifw_model;
        ifw_model->zmin = wind_db.options.zmin;
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

    // apply ramp options
    double ramp_start = 0.0;
    double ramp_end = 0.0;
    if (environment_db.ramp_start.has_value()) {
        ramp_start = environment_db.ramp_start.value();
    }
    if (environment_db.ramp_end.has_value()) {
        ramp_end = environment_db.ramp_end.value();
    }
    env_model->fluid_model->ramp_start = ramp_start;
    env_model->fluid_model->ramp_end = ramp_end;

    // soil
    if (environment_db.soil.has_value()) {
        auto soil_db = environment_db.soil.value();

        if (soil_db.type == "linear") {
            spdlog::info("Soil conditions: linear.");
            auto soil_model_shared = std::make_shared<seahowl::env::LinearSoilModel>();
            auto& soil_model = *soil_model_shared;
            soil_model.soil_position = soil_db.options.soil_position;
            soil_model.stiffness_normal = soil_db.options.stiffness_normal;
            soil_model.stiffness_shear = soil_db.options.stiffness_shear;
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

    auto DATADIR = path(filepath).parent_path();

    EnvironmentDb environment_db = read_environment_db(filepath);

    // gravity
    auto gravity = environment_db.gravity;
    system_core.elasto.set_gravitational_acceleration(Vector3d(gravity[0], gravity[1], gravity[2]));

    auto env_model = get_environmental_model(environment_db, DATADIR);

    if (env_model->fluid_model) {
        system_core.fluid_model = env_model->fluid_model;
    }
    if (env_model->soil_model) {
        system_core.soil_model = env_model->soil_model;
    }
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

    // environmental info
    auto filepath_environment = (DATADIR / json_obj.at("environment").at("file").get<std::string>()).generic_string();
    populate_environmental_conditions_from_json(filepath_environment, system_core);

    populate_system(filepath, system_core, output_folder);
}

void populate_system_from_config(const app::ConfigManager& config, seahowl::core::System& system_core) {
    // environmental info
    auto filepath_environment = config.get_string("environment.file");
    populate_environmental_conditions_from_json(filepath_environment, system_core);
    std::string output_folder = config.get_string("outputs.folder");
    populate_system(config.get_json_filepath(), system_core, output_folder);
}

void populate_system(const std::string& filepath, seahowl::core::System& system_core, std::string& output_folder) {
    auto DATADIR = path(filepath).parent_path();
    auto json_obj = get_json_from_file(filepath);

    // turbines
    auto turbines_json = json_obj.at("turbines");
    for (int ii = 0; ii < turbines_json.size(); ii++) {
        // add turbine to system
        auto turbine_json = turbines_json[ii];
        auto filepath_turbine = (DATADIR / turbine_json.at("file").get<std::string>()).generic_string();
        add_turbine_to_system_from_json(filepath_turbine, system_core);

        // get ref to turbine added last
        auto& turbine = *system_core.turbines.back();

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
    }
}

}  // namespace io
}  // namespace seahowl
