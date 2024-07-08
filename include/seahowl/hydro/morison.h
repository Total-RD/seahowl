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
    /** @brief Drag coefficient in tangent direction (Z axis). */
    double drag_tangent = 0.0;
    /** @brief Added mass coefficient in normal direction (X-Y plane). */
    double added_mass_normal = 0.0;
    /** @brief Added mass coefficient in tangent direction (Z axis). */
    double added_mass_tangent = 0.0;
    /** @brief Whether to compute buoyancy or not. */
    bool has_buoyancy = true;
    /** @brief Whether to compute inertia loads or not. */
    bool has_inertia = true;
};

/**
 * @brief Tower aerodynamic node.
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

    void compute_loads(const env::FluidModel& fluid_model, double time);
};

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

}  // namespace hydro
}  // namespace seahowl
