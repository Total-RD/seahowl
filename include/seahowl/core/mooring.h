#pragma once

#include <memory>
#include <vector>
#include <deque>

#include "seahowl/commons/utils.h"  // for DiscretizationPoint
#include "seahowl/core/component.h"

// forward declarations
namespace seahowl {
namespace elasto {
class MooringElastoFEA;
class MooringSystemElasto;
}  // namespace elasto
namespace hydro {
class MooringHydro;
class MooringSystemHydro;
}  // namespace hydro
}  // namespace seahowl

namespace seahowl {
namespace core {

/**
 * @brief Mooring of wind turbine, with both elasto and hydro components.
 *
 * This class acts as a "mediator" between the elasto and hydro components.
 * Mappings between elasto nodes to hydro domain and hydro nodes to elasto domain are used to ensure communication
 * between the hydro and elasto components. The hydro loads are communicated to the elasto component in the prestep,
 * while the hydro positions are updated using the elasto positions in the poststep.
 */
class Mooring : public ComponentDynamic {
  public:
    /** @brief Elastodynamic model of the mooring. */
    seahowl::elasto::MooringElastoFEA& elasto;
    /** @brief Hydrodynamic model of the mooring. */
    seahowl::hydro::MooringHydro& hydro;
    /** @brief Mapping of hydro nodes into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_hydro2elasto_nodes;
    /** @brief Mapping of hydro elements (central point of elements) into elasto domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_hydro2elasto_elements;
    /** @brief Mapping of elasto nodes into hydro domain. */
    std::vector<seahowl::DiscretizationPoint> mapping_elasto2hydro;

    /**
     * @brief Instantiates mooring for communication between elasto and hydro components.
     *
     * @param[in] elasto Elastodynamic mooring model.
     * @param[in] hydro hydrodynamic mooring model.
     */
    Mooring(seahowl::elasto::MooringElastoFEA& elasto, seahowl::hydro::MooringHydro& hydro);

    /**
     * @brief Sets length of the mooring line.
     *
     * @param[in] length Length of the mooring line.
     */
    void set_length(double length);

    /**
     * @brief Sets diameter of the mooring line.
     *
     * @param[in] diameter Diameter of the mooring line.
     */
    void set_diameter(double diameter);

    /**
     * @brief Prestep for mooring, called before elastodynamic stepping.
     *
     * Updates hydro loads on elasto component.
     *
     * @param[in] time Time of the simulation.
     * @param[in] dt Time step length.
     */
    void prestep(double time, double dt) override;

    /**
     * @brief Poststep for mooring, called afetr elastodynamic stepping.
     *
     * Updates hydro positions from elasto component.
     *
     * @param[in] time Absolute time of the simulation.
     * @param[in] dt Time step length.
     */
    void poststep(double time, double dt) override;

    void apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time) override;
    void apply_soil_model(seahowl::env::SoilModel& soil_model, double time) override;

    /**
     * @brief Builds the mooring (hydro and elasto part).
     *
     * Sets the nodes and elements for elasto and hydro components of the mooring, as well as the hydro->elasto mapping
     * and elasto->hydro mapping.
     */
    void build() override;

    /**
     * @brief Sets the discretization fractions to use when building the elasto part of the mooring.
     *
     * @param[in] fractions Normalized discretization fractions within [0, 1].
     */
    void set_discretization_elasto(std::vector<double> fractions);

    /**
     * @brief Sets the discretization fractions to use when building the hydro part of the mooring.
     *
     * @param[in] fractions Normalized discretization fractions within [0, 1].
     */
    void set_discretization_hydro(std::vector<double> fractions);

    /**
     * @brief Updates hydro positions, rotations, velocities and accelerations from elasto component of the mooring.
     */
    void update_positions_hydro();

    /**
     * @brief Accumulates hydro loads to the elasto component of the mooring.
     */
    void update_loads_elasto();

  private:
    void perform_sanity_check();

    /**
     * @brief Initialize mooring, called before starting the simulation.
     *
     * Runs the preset and poststep once to make elasto and hydro components match.
     *
     * @param[in] time Time of the simulation (usually 0 at init).
     * @param[in] dt Time step length.
     */
    void initialize_this(double time, double dt) override;

    /**
     * @brief Computes the hydro->elasto mapping that is used when accumulating hydro loads on elasto component.
     */
    void compute_mapping_hydro2elasto();

    /**
     * @brief Computes the elasto->hydro mapping.
     */
    void compute_mapping_elasto2hydro();
};

struct MooringSystem : public ComponentDynamic {
  public:
    /** @brief Elastodynamic model of the mooring system. */
    seahowl::elasto::MooringSystemElasto& elasto;
    /** @brief Hydrodynamic model of the mooring system. */
    seahowl::hydro::MooringSystemHydro& hydro;
    /** @brief List of mooring lines. */
    std::deque<std::shared_ptr<Mooring>> moorings;

    /**
     * @brief Constructor.
     *
     * @param[in] elasto Elastodynamic mooring system model.
     * @param[in] hydro hydrodynamic mooring system model.
     */
    MooringSystem(seahowl::elasto::MooringSystemElasto& elasto, seahowl::hydro::MooringSystemHydro& hydro);

    /**
     * @brief Adds mooring to mooring system.
     *
     * @param[in] mooring Mooring to add to mooring system.
     */
    void add_mooring(std::shared_ptr<Mooring> mooring);

    void prestep(double time, double dt) override;
    void poststep(double time, double dt) override;

    void apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time) override;
    void apply_soil_model(seahowl::env::SoilModel& soil_model, double time) override;
    void build() override;

  private:
    void perform_sanity_check();
    void initialize_this(double time, double dt) override;
};

}  // namespace core
}  // namespace seahowl
