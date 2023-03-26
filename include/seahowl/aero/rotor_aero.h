#pragma once

#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/commons/numerics.h>

#include <memory>

namespace seahowl {
namespace aero {

/**
 * @brief Rotor-Nacelle Assembly (RNA) of wind turbine as an aero component.
 */
class RotorAero {
  public:
    /** @brief List of blades. */
    std::vector<std::shared_ptr<seahowl::aero::BladeAero>> blades;
    /** @brief Hub. */
    EntityDynamicEigen body_hub;
    /** @brief Hub. */
    EntityDynamicEigen body_nacelle;
    /** @brief Total radius of the rotor (hub + blades). */
    double radius = 0.0;
    /** @brief Radius of hub. */
    double hub_radius = 0.0;
    /** @brief Azimuth of rotor. */
    double azimuth = 0.0;

    /**
     * @brief Constructor.
     */
    RotorAero();

    /**
     * @brief Builds the rotor.
     */
    void build();

    /**
     * @brief Computes chord solidity on all aero nodes of blades.
     */
    void compute_chords_solidity();

    /**
     * @brief Computes distance from hub on all aero nodes of blades.
     */
    void compute_distances_from_hub();

    /**
     * @brief Computes distance from blade tip on all aero nodes of blades.
     */
    void compute_distances_from_tip();

    /**
     * @brief Computes radius on all aero nodes of blades.
     */
    void compute_radii();

    /**
     * @brief Computes wind loads on all aero nodes of blades.
     *
     * @param[in] wind_model Wind model to use for retrieving uninduced wind velocity at nodes.
     * @param[in] time Time of simulation.
     * @param[in] tower_aero Tower reference (for tower shadow effects).
     * @param[in] tower_shadow Whether to take tower shadow effect into account or not.
     * @param[in] tip_loss Whether to take tip loss into account or not.
     * @param[in] hub_loss Whether to take hub loss into account or not.
     */
    void compute_wind_loads_bemt(WindModel& wind_model,
                                 double time,
                                 const TowerAero& tower_aero,
                                 bool tower_shadow = true,
                                 bool tip_loss = true,
                                 bool hub_loss = true);

#ifdef HAVE_AERODYN
    /**
     * @brief Computes wind loads on all aero nodes of blades using AeroDyn.
     *
     * @param[out] LoadAeroDyn Array of loads.
     * @param[in] wind_model Wind model to use for retrieving uninduced wind velocity at nodes.
     * @param[in] time Time of simulation.
     * @param[in] tower_aero Tower reference (for tower shadow effects).
     * @param[in] tower_shadow Whether to take tower shadow effect into account or not.
     * @param[in] tip_loss Whether to take tip loss into account or not.
     * @param[in] hub_loss Whether to take hub loss into account or not.
     */
    void compute_wind_loads_aerodyn(float* LoadAeroDyn,
                                    WindModel& wind_model,
                                    double time,
                                    const TowerAero& tower_aero,
                                    bool tower_shadow = true,
                                    bool tip_loss = true,
                                    bool hub_loss = true);
#endif
};

}  // namespace aero
}  // namespace seahowl
