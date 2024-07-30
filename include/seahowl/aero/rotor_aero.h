#pragma once

#include "seahowl/commons/entities.h"
#include "seahowl/commons/numerics.h"
#include "seahowl/commons/component_fluid.h"

#include <memory>

// forward declarations
namespace seahowl {
namespace aero {
class BladeAero;
class TowerAero;
}  // namespace aero
namespace env {
class FluidModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {
namespace aero {

class RotorAero : public ComponentFluid {
  public:
    /** @brief List of blades. */
    std::vector<std::shared_ptr<seahowl::aero::BladeAero>> blades;
    /** @brief Hub. */
    EntityDynamicEigen body_hub;
    /** @brief Radius of hub. */
    double hub_radius = 0.0;
    /** @brief Aerodynamic torque on hub. */
    double hub_torque_aero = 0.0;
    /** @brief Aerodynamic thrust on hub. */
    double hub_thrust_aero = 0.0;
    /** @brief Total radius of the rotor (hub + blades). */
    double radius = 0.0;
    /** @brief Azimuth of rotor. */
    double azimuth = 0.0;
    /** @brief Collective pitch of blades (in radians). */
    double pitch_collective = 0.0;

    virtual void build() = 0;
    virtual void initialize() = 0;
    virtual void compute_fluid_loads(const env::FluidModel& fluid_model, double time) = 0;
};

class RotorAeroBEMT : public RotorAero {
  public:
    /** @brief Reference to tower aero. */
    TowerAero& tower_ref;
    /** Whether to take tip loss into account or not. */
    bool has_tip_loss = true;
    /** Whether to take hub loss into account or not. */
    bool has_hub_loss = true;
    /** Whether to take tower shadow into account or not. */
    bool has_tower_shadow = true;

    RotorAeroBEMT(TowerAero& tower_ref);

    virtual void build() override;
    virtual void initialize() override;
    virtual void compute_fluid_loads(const env::FluidModel& wind_model, double time) override;

    /**
     * @brief Computes radius, distances from tip and hub, and chord solidity on all aero nodes of blades.
     */
    void compute_radii_distances_solidity();
};

// The structure containing the coefficients for the rotor disk
struct DiskCoefficients {
    // Member variables
    Eigen::MatrixXd thrust_coeff;
    Eigen::MatrixXd power_coeff;
    Eigen::VectorXd tsr_list;
    Eigen::VectorXd pitch_list;
    seahowl::Vector2d get_disk_coefficients_from_table(double TSR, double pitch);
};

class RotorAeroDisk : public RotorAero {
  public:
    /** @brief The tables of actuator disk coefficients. */
    DiskCoefficients disk_coefficients;

    virtual void build() override{};
    void initialize() override;
    void compute_fluid_loads(const env::FluidModel& wind_model, double time) override;
};

/**
 * @brief Rotor-Nacelle Assembly (RNA) of wind turbine as an aero component.
 */
class RotorNacelleAssemblyAero : public ComponentFluid {
  public:
    /** @brief Rotor. */
    std::shared_ptr<RotorAero> rotor;
    /** @brief Nacelle. */
    EntityDynamicEigen body_nacelle;

    /**
     * @brief Constructor.
     */
    RotorNacelleAssemblyAero();

    void compute_fluid_loads(const env::FluidModel& wind_model, double time) override;

    /**
     * @brief Builds rotor.
     */
    void build();

    /**
     * @brief Initializes rotor related variables with current configuration.
     */
    void initialize();
};

}  // namespace aero
}  // namespace seahowl
