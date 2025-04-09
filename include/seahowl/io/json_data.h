#pragma once

#include <seahowl/commons/numerics.h>

namespace seahowl {
namespace io {
// Structure pour un point de référence
struct ReferencePointTower {
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
struct GlobalVariablesTower {
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
struct TowerData {
    GlobalVariablesTower global_variables;
    std::vector<ReferencePointTower> reference_points;
};

struct GlobalVariablesBlade {
    double damping_flapwise;
    double damping_edgewise;
    double damping_axial;
    double damping_torsion;
    double damping_mass;
    Eigen::Vector2d offset_gravity;
    Eigen::Vector2d offset_elastic;
};

struct ReferencePointBlade {
    Eigen::Vector3d coordinates;
    double twist;
    double fraction;
    Eigen::MatrixXd stiffness_matrix;
    Eigen::MatrixXd mass_matrix;
    double chord;
    std::string airfoil_file;
    Eigen::Vector2d offset_aero;
};

struct BladeData {
    GlobalVariablesBlade global_variables;
    std::vector<ReferencePointBlade> reference_points;
};

struct AirfoilData {
    double reynolds_number;
    std::vector<std::string> header;
    std::vector<std::vector<double>> coefficients;
};

/**
 * @brief Read tower data from a file.
 * @param filepath Path to the tower data file.
 */
TowerData read_tower(const std::string& filepath);

/**
 * @brief Read airfoil data from a file.
 * @param filepath Path to the airfoil data file.
 */
std::vector<AirfoilData> read_airfoil(const std::string& filepath);

/**
 * @brief Read blade data from a file.
 * @param filepath Path to the blade data file.
 */
BladeData read_blade(const std::string& filepath);

}  // namespace io
}  // namespace seahowl
