#pragma once

#include "seahowl/commons/entities.h"
#include "seahowl/commons/numerics.h"

#include <memory>

// forward declarations
namespace seahowl {
namespace aero {
class BladeAero;
class TowerAero;
class WindModel;
}  // namespace aero
}  // namespace seahowl

namespace seahowl {
namespace aero {

/**
 * @brief Rotor-Nacelle Assembly (RNA) of wind turbine as an aero component.
 */
class RotorNacelleAssemblyAero {
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

    /** @brief Collective pitch of blades (in radians). */
    double pitch_collective = 0;

    /**
     * @brief Constructor.
     */
    RotorNacelleAssemblyAero();

    /**
     * @brief Builds the rotor.
     */
    void build();

    /**
     * @brief Initialize rotor related variables with current configuration.
     */
    void initialize();

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
    void compute_aero_loads(const WindModel& wind_model,
                            double time,
                            const TowerAero& tower_aero,
                            bool tower_shadow = true,
                            bool tip_loss = true,
                            bool hub_loss = true);

    /**
     * @brief Computes wind loads on rotor.
     *
     * @param[in] wind_model Wind model to use for retrieving uninduced wind velocity at nodes.
     * @param[in] time Time of simulation.
     * @param[in] pitch collective pitch rotor (for getting performance from table).
     * @param[in] RPM Rotor speed (for getting performance from table).
     *
     */
    void compute_aero_loads_disk(const WindModel& wind_model, double time, double pitch, double RPM);

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
    void compute_aero_loads(float* LoadAeroDyn,
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
