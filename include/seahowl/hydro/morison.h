#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/commons/entities.h"

// forward declarations
namespace seahowl {
namespace env {
class FluidModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {
namespace hydro {

struct HydroCoefficients {
    /** @brief Drag coefficient in normal direction (X-Y plane). */
    double drag_normal = 0.0;
    /** @brief Drag coefficient in axial direction (Z axis). */
    double drag_axial = 0.0;
    /** @brief Added mass coefficient in normal direction (X-Y plane). */
    double added_mass_normal = 0.0;
    /** @brief Added mass coefficient in axial direction (Z axis). */
    double added_mass_axial = 0.0;
    /** @brief Factor for buoyancy (1.0 for fully buoyant). */
    double buoyancy_factor = 1.0;
    /** @brief Factor for nodal acceleration (1.0 for default behavior). */
    double nodal_acceleration_factor = 1.0;
    /** @brief MacCamy and Fuchs Correction for large cylinders, Flag. */
    bool use_MacCamyFuchs_correction = false;
    /** @brief Cd Correction for large cylinders, Flag. */
    bool use_Cd_correction = false;

    HydroCoefficients operator*(const double factor) const;
    HydroCoefficients operator+(const HydroCoefficients& other) const;
};

class MacCamyFuchsTable {
    /** @brief MacCamy and Fuchs Empirical Table for large cylinders */
  public:
    MacCamyFuchsTable();
    /** @brief Wave period */
    double wave_peak_period = 0.0;
    /** @brief The MacCamy and Fuchs Empirical Table for large cylinders */
    std::vector<std::pair<double, double>> MCFTable;
    /** @brief Function to generate the MacCamy and Fuchs Empirical Table for large cylinders */
    void generateMacCamyFuchsTable();
    /** @brief Function to interpolate, per each morison element, the MacCamy and Fuchs Cm coefficient */
    double interpolateCmBinarySearch(double D);
    /** @brief Function to interpolate, per each morison element, the coefficients to get the Cd */
    double interpolate(double x, const std::vector<double>& xData, const std::vector<double>& yData);
    /** @brief Function to interpolate the Cd */
    double getCd(double diameter, double wave_period, double fluid_velocity);
};
extern MacCamyFuchsTable myMCFtable;

/**
 * @brief Morison node.
 */
class MorisonNode : public EntityDynamicEigen {
  public:
    /** @brief Hydrodynamic coefficients. */
    HydroCoefficients coefficients;
    /** @brief Load calculated at node. */
    Vector3d load{0.0, 0.0, 0.0};
    /** @brief Load from without component from structural acceleration. */
    Vector3d load_noacc{0.0, 0.0, 0.0};
    /** @brief Added mass matrix (actually linear density). */
    Eigen::Matrix<double, 6, 6> added_mass_matrix = Eigen::Matrix<double, 6, 6>::Zero();
    /** @brief Diameter at node. */
    double diameter = 0.0;

    /**
     * @brief Constructor.
     */
    MorisonNode();

    void compute_fluid_loads(const env::FluidModel& fluid_model, double time);
};

/**
 * @brief Morison element.
 */
class MorisonElement {
  public:
    /** @brief First node of element. */
    const MorisonNode& node1;
    /** @brief Second node of element. */
    const MorisonNode& node2;
    /** @brief Length of element. */
    double length = 0.0;

    MorisonElement(const MorisonNode& node1, const MorisonNode& node2);

    /**
     * @brief Returns integrated load at center of element.
     */
    Vector3d get_load() const;

    /**
     * @brief Returns integrated load (without component from structural acceleration) at center of element.
     */
    Vector3d get_load_noacc() const;

    /**
     * @brief Returns added mass matrix at center of element.
     */
    Eigen::Matrix<double, 6, 6> get_added_mass_matrix() const;

    /**
     * @brief Get position of center of element.
     */
    Vector3d get_position() const;

    /**
     * @brief Get rotation of center of element.
     */
    Quaternion get_rotation() const;
};

/** Morison plate (with normal along Z-axis). */
class MorisonPlate : public EntityDynamicEigen {
  public:
    /** @brief Diameter at node. */
    double diameter = 0.0;
    /** @brief Drag coefficient. */
    double drag_coefficient = 0.0;
    /** @brief Load calculated at node. */
    Vector3d load{0.0, 0.0, 0.0};
    /** @brief Whether normal direction is along positive or negative Z-axis. */
    bool reverse_direction = false;

    /**
     * @brief Constructor.
     */
    MorisonPlate();

    void compute_fluid_loads(const env::FluidModel& fluid_model, double time);
};

}  // namespace hydro
}  // namespace seahowl
