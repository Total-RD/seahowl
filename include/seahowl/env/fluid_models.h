#pragma once

#include "seahowl/commons/numerics.h"

namespace seahowl {
namespace env {

/**
 * @brief Base class for fluid models
 */
class FluidModel {
  public:
    /**
     * @brief Returns fluid velocity at given coordinates.
     *
     * @param[in] position Position at which fluid velocity is extracted.
     * @param[in] time Time of simulation.
     */
    virtual Vector3d get_fluid_velocity(const Vector3d& position, double time) const = 0;

    /**
     * @brief Returns fluid density at given coordinates.
     *
     * @param[in] position Position at which fluid density is extracted.
     * @param[in] time Time of simulation.
     */
    virtual double get_fluid_density(const Vector3d& position, double time) const = 0;

    virtual Vector3d get_fluid_acceleration(const Vector3d& position, double time) const {
        return Vector3d(0.0, 0.0, 0.0);
    };

    /** @brief wind model from CFD */
    int wind_model_amrwind = 1;

    int get_amrwind_wind_model() { return wind_model_amrwind; }

    void set_amrwind_wind_model(int status) { wind_model_amrwind = status; }

    float* u;
    int u_Len;  // x velocity at interface (seahowl) nodes [m]
    float* v;
    int v_Len;  // y velocity at interface (seahowl) nodes [m]
    float* w;
    int w_Len;  // z velocity at interface (seahowl) nodes [m]

    float* pxVel;
    int pxVel_Len;  // x position of velocity interface (seahowl) nodes [m]
    float* pyVel;
    int pyVel_Len;  // y position of velocity interface (seahowl) nodes [m]
    float* pzVel;
    int pzVel_Len;  // z position of velocity interface (seahowl) nodes [m]

    std::vector<Vector3d> wind_velocities;
    std::vector<Vector3d> wind_positions;
};

}  // namespace env
}  // namespace seahowl
