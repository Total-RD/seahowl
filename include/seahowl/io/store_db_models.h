#pragma once

#include <seahowl/commons/numerics.h>
#include <optional>
#include <filesystem>

namespace fs = std::filesystem;

namespace seahowl {
namespace io {
// Structure pour un point de référence
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
    double buoyancy_factor;
    double damping_foreaft;
    double damping_sideside;
    double damping_axial;
    double damping_torsion;
    double damping_mass;
};

// Structure pour les variables globales
struct GlobalVariablesTowerDb {
    double density;
    double young_modulus;
    double poisson_ratio;
    double drag_coefficient_normal;
    double drag_coefficient_axial;
    double added_mass_coefficient_normal;
    double added_mass_coefficient_axial;
    double buoyancy_factor;
    double damping_foreaft;
    double damping_sideside;
    double damping_axial;
    double damping_torsion;
    double damping_mass;
};

// Structure principale pour englober les données
struct TowerDb {
    GlobalVariablesTowerDb global_variables;
    std::vector<ReferencePointTowerDb> reference_points;
};

struct GlobalVariablesBladeDb {
    double damping_flapwise;
    double damping_edgewise;
    double damping_axial;
    double damping_torsion;
    double damping_mass;
    Eigen::Vector2d offset_gravity;
    Eigen::Vector2d offset_elastic;
};

struct AirfoilDb {
    double reynolds_number;
    std::vector<std::string> header;
    std::vector<std::vector<double>> coefficients;
};

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

struct BladeDb {
    GlobalVariablesBladeDb global_variables;
    std::vector<ReferencePointBladeDb> reference_points;
};

struct ShaftDb {
    double tilt;
    double distance_from_towertop;
};

struct NacelleDb {
    Eigen::Vector3d position_from_towertop;
    double mass;
    Eigen::Matrix<double, 3, 3> inertia;
    double yaw_bearing_mass;
};

struct DrivetrainDb {
    double generator_inertia;
    double gearbox_efficiency;
    double gearbox_ratio;
    double generator_efficiency;
};

struct HubDb {
    double radius;
    Eigen::Vector3d position_from_apex;
    double overhang;
    double mass;
    Eigen::Matrix<double, 3, 3> inertia;
};

struct RnaDb {
    ShaftDb shaft;
    NacelleDb nacelle;
    DrivetrainDb drivetrain;
    HubDb hub;
};

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

struct WindDb {
    std::string type;
    double air_density;
    WindOptionDb options;
};

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

struct SeaDb {
    std::string type;
    double water_density;
    double mean_water_level;
    double water_depth;
    SeaOptionDb options;
};

struct SoilOptionDb {
    double stiffness_normal;
    double stiffness_shear;
    double soil_position;
};

struct SoilDb {
    std::string type;
    SoilOptionDb options;
};

struct EnvironmentDb {
    Eigen::Vector3d gravity;
    std::optional<double> ramp_start;
    std::optional<double> ramp_end;
    WindDb wind;
    std::optional<SeaDb> sea;
    std::optional<SoilDb> soil;
};

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

struct AeroTurbineDb {
    std::string solver;
    AeroOptionsTurbineDb options;
};

struct DiscretizationTurbineDb {
    std::vector<double> elasto;
    std::vector<double> aero;
};

struct BladeTurbineDb {
    std::string file;
    BladeDb data;
    double initial_pitch;
    double precone;
};

struct RotorOptionsTurbineDb {
    // disk (single body) options
    double inertia_blades;
    double mass_blades;
    double radius;
};

struct DiscretizationRotorTurbineDb {
    std::vector<double> elasto;
    std::vector<double> aero;
};
struct RotorTurbineDb {
    std::string type;
    DiscretizationRotorTurbineDb discretization;
    bool pitch_actuator_dynamics;
    std::vector<BladeTurbineDb> blades;
    RotorOptionsTurbineDb option;
};

struct RNATurbineDb {
    std::string file;
    RnaDb data;
    double initial_yaw;
    bool yaw_actuator_dynamics;
};

struct TowerOptionsTurbineDb {
    std::optional<bool> use_MacCamyFuchs_correction;
    std::optional<bool> use_Cd_correction;
};

struct DiscretizationTowerTurbineDb {
    std::vector<double> elasto;
    std::vector<double> aero;
};
struct TowerTurbineDb {
    DiscretizationTowerTurbineDb discretization;
    std::string file;
    TowerDb data;
    std::optional<TowerOptionsTurbineDb> options;
};

struct ControllerOptionsTurbineDb {
    std::string infile;     // relative path from input file
    fs::path infile_path;   // relative path from driver execution folder
    std::string libfile;    // relative path from input file
    fs::path libfile_path;  // relative path from driver execution folder

    // options for "RPM" type controller
    double target_rpm;
};

struct ControllerTurbineDb {
    std::string type;
    ControllerOptionsTurbineDb options;
};

struct DiscretizationFoundationTurbineDb {
    std::vector<double> elasto;
    std::vector<double> hydro;
};

struct BodyFloaterdb {
    std::string name;
    Eigen::Vector3d position;
    double mass;
    Eigen::Matrix3d inertia;
};

struct DiscretizationFloaterdb {
    std::vector<double> elasto;
    std::vector<double> hydro;
};

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
struct FoundationTurbineDb {
    std::string type;
    std::optional<std::string> file;
    Floaterdb data_floater;
    TowerDb data_tower;
    DiscretizationFoundationTurbineDb discretization;
    std::optional<TowerOptionsTurbineDb> options;
};

struct TurbineDb {
    AeroTurbineDb aero;
    RotorTurbineDb rotor;
    RNATurbineDb rna;
    TowerTurbineDb tower;
    ControllerTurbineDb controller;
    std::optional<FoundationTurbineDb> foundation;
};

struct StaticsMainDb {
    bool linear_step;
    int nonlinear_steps;
};

struct PresimulationMainDb {
    double dt;
    double duration;
    bool presetup;
    bool fix_towers;
};

struct NumericsMainDb {
    double dt;
    double duration;
    StaticsMainDb statics;
    PresimulationMainDb presimulation;
};

struct OutputsMainDb {
    double dt;
    std::optional<std::string> folder;
    bool vtk;
    std::string log_level;
    bool gui;
};

struct EnvironmentMainDb {
    std::string file;
    EnvironmentDb data;
};

struct TurbineMainDb {
    std::string file;
    TurbineDb data;
    Eigen::Vector3d translation;
    double rotation;
};

struct MainDb {
    NumericsMainDb numerics;
    OutputsMainDb outputs;
    EnvironmentMainDb environment;
    std::vector<TurbineMainDb> turbines;
};

}  // namespace io
}  // namespace seahowl
