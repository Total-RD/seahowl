#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/commons/entities.h"

namespace seahowl {
namespace env {

/**
 * @brief Base class for soil models
 */
class SoilModel {
    /**
     * @brief Returns soil penetration load.
     *
     * @param[in] entity Dynamic entity (potentially) penetrating soil.
     * @param[in] contact_area Contact area of entity penetrating soil.
     * @param[in] entity_mass Mass of entity penetrating soil.
     */
    virtual Vector3d get_penetration_load(const EntityDynamic& entity,
                                          double contact_area,
                                          double entity_mass) const = 0;
};

/**
 * @brief Base class for soil models
 */
class LinearSoilModel : public SoilModel {
  public:
    /** @brief Soil position. */
    double soil_position = 0.0;
    /** @brief Soil normal vector. */
    Vector3d soil_normal{0.0, 0.0, 1.0};
    /** @brief Soil normal stiffness. */
    double stiffness_normal = 0.0;
    /** @brief Soil shear stiffness. */
    double stiffness_shear = 0.0;

    /**
     * @brief Constructor.
     */
    LinearSoilModel();

    virtual Vector3d get_penetration_load(const EntityDynamic& entity,
                                          double contact_area,
                                          double entity_mass) const override;
};

}  // namespace env
}  // namespace seahowl
