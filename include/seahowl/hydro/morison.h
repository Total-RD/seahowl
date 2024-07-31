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
    bool buoyancy_factor = 1.0;
    /** @brief Factor for inertia (1.0 for default behavior). */
    bool inertia_factor = 1.0;
    /** @brief Factor for acceleration (1.0 for default behavior). */
    bool acceleration_factor = 1.0;

    HydroCoefficients operator*(const double factor) const;
    HydroCoefficients operator+(const HydroCoefficients& other) const;
};

/**
 * @brief Morison node.
 */
class MorisonNode : public EntityDynamicEigen {
  public:
    /** @brief Load calculated at node. */
    Vector3d load{0.0, 0.0, 0.0};
    /** @brief Hydrodynamic coefficients. */
    HydroCoefficients coefficients;
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
