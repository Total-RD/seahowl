#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/commons/entities.h"

// forward declarations
namespace seahowl {
namespace env {
class EnvModel;
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

    /**
     * @brief Multiplies all coefficients by a scalar factor.
     *
     * @param[in] factor Scalar multiplication factor.
     * @return New HydroCoefficients with scaled values.
     */
    HydroCoefficients operator*(const double factor) const;

    /**
     * @brief Adds two HydroCoefficients element-wise.
     *
     * @param[in] other HydroCoefficients to add.
     * @return New HydroCoefficients with summed values.
     */
    HydroCoefficients operator+(const HydroCoefficients& other) const;
};

/**
 * @brief MacCamy and Fuchs correction table for large cylinder wave loads.
 *
 * Provides corrections to Morison equation coefficients for large diameter
 * cylinders where diffraction effects become significant.
 */
class MacCamyFuchsTable {
  public:
    /**
     * @brief Constructor.
     */
    MacCamyFuchsTable();

    /** @brief Wave peak period [s]. */
    double wave_peak_period = 0.0;
    /** @brief Lookup table mapping diameter to Cm correction factor. */
    std::vector<std::pair<double, double>> MCFTable;

    /**
     * @brief Generates the MacCamy and Fuchs empirical correction table.
     */
    void generateMacCamyFuchsTable();

    /**
     * @brief Interpolates the MacCamy and Fuchs Cm coefficient for a given diameter.
     *
     * @param[in] D Cylinder diameter [m].
     * @return Corrected added mass coefficient Cm.
     */
    double interpolateCmBinarySearch(double D);

    /**
     * @brief Performs linear interpolation on tabulated data.
     *
     * @param[in] x Value at which to interpolate.
     * @param[in] xData Vector of x-coordinates.
     * @param[in] yData Vector of y-coordinates.
     * @return Interpolated y value.
     */
    double interpolate(double x, const std::vector<double>& xData, const std::vector<double>& yData);

    /**
     * @brief Returns corrected drag coefficient for large cylinders.
     *
     * @param[in] diameter Cylinder diameter [m].
     * @param[in] wave_period Wave period [s].
     * @param[in] fluid_velocity Fluid velocity magnitude [m/s].
     * @return Corrected drag coefficient Cd.
     */
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

    /**
     * @brief Computes hydrodynamic loads from environmental model.
     *
     * Calculates Morison equation loads including drag, inertia, and added mass
     * contributions based on the current wave/current conditions.
     *
     * @param[in] env_model Environmental model containing wave and current data.
     * @param[in] time Current simulation time [s].
     */
    void compute_env_loads(const env::EnvModel& env_model, double time);
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

    /**
     * @brief Constructor.
     *
     * @param[in] node1 First node of element.
     * @param[in] node2 Second node of element.
     */
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

    /**
     * @brief Computes hydrodynamic loads on plate from environmental model.
     *
     * @param[in] env_model Environmental model containing wave and current data.
     * @param[in] time Current simulation time [s].
     */
    void compute_env_loads(const env::EnvModel& env_model, double time);
};

}  // namespace hydro
}  // namespace seahowl
