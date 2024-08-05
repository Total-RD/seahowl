#pragma once

#include "seahowl/commons/numerics.h"
#include "seahowl/commons/entities.h"
#include "seahowl/hydro/morison.h"
#include "seahowl/commons/component_fluid.h"

#include <vector>
#include <deque>
#include <memory>

// forward declarations
namespace seahowl {
namespace env {
class FluidModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {

/**@brief Hydrodynamic module */
namespace hydro {

/**
 * @brief Mooring of wind turbine as an hydrodynamic component.
 */
class MooringHydro : public ComponentFluid {
  public:
    /** @brief Discretization fractions (normalized abscissa) in the range [0, 1] to discretize the hydro component. */
    std::vector<double> discretization_fractions;
    /** @brief Hydrodynamic coefficients. */
    HydroCoefficients coefficients;
    /** @brief Hydro nodes. */
    std::vector<hydro::MorisonNode> nodes;
    /** @brief Hydro elements. */
    std::vector<hydro::MorisonElement> elements;
    /** @brief Loads at center of mooring elements. */
    std::vector<Vector3d> loads;
    /** @brief Position of the mooring line. */
    double diameter = 0.0;
    /** @brief Length of the mooring line. */
    double length = 0.0;

    /**
     * @brief Constructor.
     */
    MooringHydro();

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
     * @brief Builds the mooring.
     */
    void build();

    /**
     * @brief Compute fluid loads on mooring using Morison's approach on cylindrical elements.
     */
    void compute_fluid_loads(const env::FluidModel& fluid_model, double time) override;
};

/**
 * @brief Mooring system class gathering mooring lines.
 */
struct MooringSystemHydro : public ComponentFluid {
  public:
    /** @brief List of mooring lines. */
    std::deque<std::shared_ptr<MooringHydro>> moorings;

    /**
     * @brief Constructor.
     */
    MooringSystemHydro();

    /**
     * @brief Adds mooring to mooring system.
     *
     * @param[in] mooring Mooring to add to mooring system.
     */
    void add_mooring(std::shared_ptr<MooringHydro> mooring);

    /**
     * @brief Builds the mooring system.
     */
    void build() override;

    /**
     * @brief Compute fluid loads on mooring using Morison's approach on cylindrical elements.
     */
    void compute_fluid_loads(const env::FluidModel& fluid_model, double time) override;
};

}  // namespace hydro
}  // namespace seahowl
