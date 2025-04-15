#pragma once

#include <seahowl/commons/numerics.h>
#include <optional>

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
    double reference_height;
    double shear_coefficient;
    Eigen::Vector3d velocity_start;
    Eigen::Vector3d velocity_end;
    double time_start;
    double time_end;
    std::string file_inflowwind;
    double zmin;
};

struct WindDb {
    std::string type;
    double air_density;
    WindOptionDb options;
};

struct SeaOptionDb {
    std::string type;
    double wave_height;
    double wave_period;
    double frequency_min;
    double frequency_max;
    int nfrequencies;
    double duration;
    double dt;
    double peak_enhancement_factor;
    bool is_normalized;
    int seed;
    int num_bodies;
    bool wave_stretching;
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
    bool hub_loss;
    bool tip_loss;
    bool tower_shadow;
    std::string performance_file;
    std::string file_aerodyn;
    std::string file_inflowwind;
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
    double initial_pitch;
    double precone;
};

struct RotorOptionsTurbineDb {
    double inertia_blades;
    double mass_blades;
    double radius;
};

struct RotorTurbineDb {
    std::string type;
    DiscretizationTurbineDb discretization;
    bool pitch_actuator_dynamics;
    std::vector<BladeTurbineDb> blades;
    RotorOptionsTurbineDb option;
};

struct RNATurbineDb {
    std::string file;
    double initial_yaw;
    bool yaw_actuator_dynamics;
};

struct TowerOptionsTurbineDb {
    std::optional<bool> use_MacCamyFuchs_correction;
    std::optional<bool> use_Cd_correction;
};

struct TowerTurbineDb {
    DiscretizationTurbineDb discretization;
    std::string file;
    std::optional<TowerOptionsTurbineDb> options;
};

struct ControllerOptionsTurbineDb {
    std::string infile;
    std::string libfile;
    double target_rpm;
};

struct ControllerTurbineDb {
    std::string type;
    ControllerOptionsTurbineDb options;
};

struct FoundationTurbineDb {
    std::string type;
    std::optional<std::string> file;
    DiscretizationTurbineDb discretization;
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

}  // namespace io
}  // namespace seahowl
