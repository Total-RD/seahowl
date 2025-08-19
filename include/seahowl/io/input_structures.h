#pragma once

#include <seahowl/commons/numerics.h>
#include <optional>
#include <filesystem>

namespace fs = std::filesystem;

namespace seahowl {
namespace io {

/**
 * @brief Structure for reference point in tower database
 */
struct ReferencePointTowerDb {
    Eigen::Vector3d position;
    double diameter;
    double thickness;
    double density;
    double young_modulus;
    double poisson_ratio;
    double drag_coefficient_normal;
    double drag_coefficient_axial;
    double added_mass_coefficient_normal;
    double added_mass_coefficient_axial;
    double fill_density;
    double buoyancy_factor;
    double damping_foreaft;
    double damping_sideside;
    double damping_axial;
    double damping_torsion;
    double damping_mass;
};

/**
 * @brief Structure for tower database
 */
struct TowerDb {
    std::vector<ReferencePointTowerDb> reference_points;
};

/**
 * @brief Structure for airfoil database
 */
struct AirfoilDb {
    double reynolds_number;
    std::vector<std::string> header;
    std::vector<std::vector<double>> coefficients;
};

/**
 * * @brief Structure for reference point in blade database
 */
struct ReferencePointBladeDb {
    Eigen::Vector3d coordinates;
    double twist;
    double fraction;
    Eigen::MatrixXd stiffness_matrix;
    Eigen::MatrixXd mass_matrix;
    double chord;
    std::string airfoil_file;
    std::vector<AirfoilDb> airfoil_db_list;
    Eigen::Vector2d offset_aero;
    double damping_flapwise;
    double damping_edgewise;
    double damping_axial;
    double damping_torsion;
    double damping_mass;
    Eigen::Vector2d offset_gravity;
    Eigen::Vector2d offset_elastic;
};

/**
 * * @brief Structure for blade database
 */
struct BladeDb {
    std::vector<ReferencePointBladeDb> reference_points;
};

/**
 * @brief Structure for shaft database
 */
struct ShaftDb {
    double tilt;
    double distance_from_towertop;
};

/**
 * @brief Structure for nacelle database
 */
struct NacelleDb {
    Eigen::Vector3d position_from_towertop;
    double mass;
    Eigen::Matrix<double, 3, 3> inertia;
    double yaw_bearing_mass;
};

/**
 * @brief Structure for drivetrain database
 */
struct DrivetrainDb {
    double generator_inertia;
    double gearbox_efficiency;
    double gearbox_ratio;
    double generator_efficiency;
};

/**
 * @brief Structure for hub database
 */
struct HubDb {
    double radius;
    Eigen::Vector3d position_from_apex;
    double overhang;
    double mass;
    Eigen::Matrix<double, 3, 3> inertia;
};

/**
 * @brief Structure for RNA database
 */
struct RnaDb {
    ShaftDb shaft;
    NacelleDb nacelle;
    DrivetrainDb drivetrain;
    HubDb hub;
};

/**
 * @brief Structure for wind options
 */
struct WindOptionDb {
    // general wind options
    double reference_height;
    double shear_coefficient;

    // wind ramp options
    Eigen::Vector3d velocity_start;
    Eigen::Vector3d velocity_end;
    double time_start;
    double time_end;

    // InflowWind options
    std::string file_inflowwind;    // relative path from input file
    fs::path file_inflowwind_path;  // relative path from driver execution folder
    double zmin;
};

/**
 * @brief Structure for wind database
 */
struct WindDb {
    std::string type;
    double air_density;
    WindOptionDb options;
};

/**
 * @brief Structure for sea options
 */
struct SeaOptionDb {
    // wave options
    std::string type;  // regular / irregular
    double wave_height;
    double wave_period;

    // irregular wave options
    double frequency_min;
    double frequency_max;
    int nfrequencies;
    double duration;
    double dt;
    double peak_enhancement_factor;
    bool is_normalized;
    int seed;
    bool wave_stretching;
    int num_bodies;

    // current options
    Eigen::Vector3d direction;
    double velocity_surface;
    double velocity_seabed;
};

/**
 * @brief Structure for sea database
 */
struct SeaDb {
    std::string type;
    double water_density;
    double mean_water_level;
    double water_depth;
    SeaOptionDb options;
};

// Structure for soil options
/**
 * @brief Structure for soil options
 */
struct SoilOptionDb {
    double stiffness_normal;
    double stiffness_shear;
    double soil_position;
};

/**
 * @brief Structure for soil database
 */
struct SoilDb {
    std::string type;
    SoilOptionDb options;
};

/**
 * @brief Structure for environment database
 */
struct EnvironmentDb {
    Eigen::Vector3d gravity;
    std::optional<double> ramp_start;
    std::optional<double> ramp_end;
    WindDb wind;
    std::optional<SeaDb> sea;
    std::optional<SoilDb> soil;
};

/**
 * @brief Structure for aerodyn options in turbine database
 */
struct AeroOptionsTurbineDb {
    // AeroDyn options
    std::string file_aerodyn;       // relative path from input file
    fs::path file_aerodyn_path;     // relative path from driver execution folder
    std::string file_inflowwind;    // relative path from input file
    fs::path file_inflowwind_path;  // relative path from driver execution folder

    // BEMT options
    bool hub_loss;
    bool tip_loss;
    bool tower_shadow;

    // actuator disk options
    std::string performance_file;    // relative path from input file
    fs::path performance_file_path;  // relative path from driver execution folder
};

/**
 * @brief Structure for aerodyn options in turbine database
 */
struct AeroTurbineDb {
    std::string solver;
    AeroOptionsTurbineDb options;
};

/**
 * @brief Structure for discretization of turbine database
 */
struct DiscretizationTurbineDb {
    std::vector<double> elasto;
    std::vector<double> aero;
};

/**
 * @brief Structure for blade in turbine database
 */
struct BladeTurbineDb {
    std::string file;
    BladeDb data;
    double initial_pitch;
    double precone;
};

/**
 * @brief Structure for rotor options in turbine database
 */
struct RotorOptionsTurbineDb {
    // disk (single body) options
    double inertia_blades;
    double mass_blades;
    double radius;
};

/**
 * @brief Structure for discretization of rotor turbine database
 */
struct DiscretizationRotorTurbineDb {
    std::vector<double> elasto;
    std::vector<double> aero;
};

/**
 * @brief Structure for rotor turbine database
 */
struct RotorTurbineDb {
    std::string type;
    DiscretizationRotorTurbineDb discretization;
    bool pitch_actuator_dynamics;
    std::vector<BladeTurbineDb> blades;
    RotorOptionsTurbineDb option;
};

/**
 * @brief Structure for RNATurbine database
 */
struct RNATurbineDb {
    std::string file;
    RnaDb data;
    double initial_yaw;
    bool yaw_actuator_dynamics;
};

/**
 * @brief Structure for tower options in turbine database
 */
struct TowerOptionsTurbineDb {
    std::optional<bool> use_MacCamyFuchs_correction;
    std::optional<bool> use_Cd_correction;
};

/**
 * @brief Structure for discretization of tower turbine database
 */
struct DiscretizationTowerTurbineDb {
    std::vector<double> elasto;
    std::vector<double> aero;
};

/**
 * @brief Structure for tower turbine database
 */
struct TowerTurbineDb {
    DiscretizationTowerTurbineDb discretization;
    std::string file;
    TowerDb data;
    std::optional<TowerOptionsTurbineDb> options;
};

/**
 * @brief Structure for controller options in turbine database
 */
struct ControllerOptionsTurbineDb {
    std::string infile;     // relative path from input file
    fs::path infile_path;   // relative path from driver execution folder
    std::string libfile;    // relative path from input file
    fs::path libfile_path;  // relative path from driver execution folder

    // options for "RPM" type controller
    double target_rpm;
};

/**
 * @brief Structure for controller turbine database
 */
struct ControllerTurbineDb {
    std::string type;
    ControllerOptionsTurbineDb options;
};

/**
 * @brief Structure for discretization of foundation turbine database
 */
struct DiscretizationFoundationTurbineDb {
    std::vector<double> elasto;
    std::vector<double> hydro;
};

/**
 * @brief Structure for body in floater database
 */
struct BodyFloaterdb {
    std::string name;
    Eigen::Vector3d position;
    double mass;
    Eigen::Matrix3d inertia;
};

/**
 * @brief Structure for discretization of floater database
 */
struct DiscretizationFloaterdb {
    std::vector<double> elasto;
    std::vector<double> hydro;
};

/**
 * @brief Structure for mooring properties in floater database
 */
struct MooringPropertiesDb {
    double diameter;
    double stiffness_axial;
    double stiffness_bending;
    double density_linear;
    double drag_coefficient_normal;
    double drag_coefficient_axial;
    double added_mass_coefficient_normal;
    double added_mass_coefficient_axial;
};

/**
 * @brief Structure for mooring in floater database
 */
struct MooringFloaterdb {
    std::string connected_body_name;
    std::string line_properties;
    double length;
    DiscretizationFloaterdb discretization;
    bool relative_fairlead;
    bool relative_anchor;
    Eigen::Vector3d fairlead_position;
    Eigen::Vector3d anchor_position;
    Eigen::Vector3d rotation_axis;
    double rotation_angle;
    MooringPropertiesDb properties;
};

/**
 * @brief structure for floater database
 */
struct Floaterdb {
    std::string type;
    std::string options_file;    // relative path from input file
    fs::path options_file_path;  // relative path from driver execution folder
    Eigen::Vector3d position;
    double mass;
    Eigen::Matrix3d inertia;
    Eigen::MatrixXd damping_matrix;
    std::vector<BodyFloaterdb> bodies;
    std::vector<MooringFloaterdb> moorings;
};

/**
 * @brief structure for foundation turbine database
 */
struct FoundationTurbineDb {
    std::string type;
    std::optional<std::string> file;  // relative path from input file
    fs::path file_path;               // relative path from driver execution folder
    Floaterdb data_floater;
    TowerDb data_tower;
    DiscretizationFoundationTurbineDb discretization;
    std::optional<TowerOptionsTurbineDb> options;
};

/**
 * @brief structure for turbine database
 */
struct TurbineDb {
    AeroTurbineDb aero;
    RotorTurbineDb rotor;
    RNATurbineDb rna;
    TowerTurbineDb tower;
    ControllerTurbineDb controller;
    std::optional<FoundationTurbineDb> foundation;
};

/**
 * @brief structure for statics in main database
 */
struct StaticsMainDb {
    bool linear_step;
    int nonlinear_steps;
};

/**
 * @brief structure for presimulation in main database
 */
struct PresimulationMainDb {
    double dt;
    double duration;
    bool presetup;
    bool fix_towers;
};

/**
 * @brief structure for numerics in main database
 */
struct NumericsMainDb {
    double dt;
    double duration;
    StaticsMainDb statics;
    PresimulationMainDb presimulation;
};

/**
 * @brief structure for outputs in main database
 */
struct OutputsMainDb {
    double dt;
    std::optional<std::string> folder;
    bool vtk;
    std::string log_level;
    bool gui;
};

/**
 * @brief structure for environment in main database
 */
struct EnvironmentMainDb {
    std::string file;
    EnvironmentDb data;
};

/**
 * @brief structure for turbine in main database
 */
struct TurbineMainDb {
    std::string file;
    TurbineDb data;
    Eigen::Vector3d translation;
    double rotation;
};

/**
 * @brief structure for main database
 */
struct MainDb {
    NumericsMainDb numerics;
    OutputsMainDb outputs;
    EnvironmentMainDb environment;
    std::vector<TurbineMainDb> turbines;
};

}  // namespace io
}  // namespace seahowl
