#pragma once

#include <seahowl/commons/numerics.h>

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

}  // namespace io
}  // namespace seahowl
