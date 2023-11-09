#pragma once

#include "seahowl/elasto/component_elasto.h"
#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/commons/numerics.h"
#include "seahowl/env/soil_models.h"

#include <deque>

namespace seahowl {
namespace elasto {

/**
 * @brief Mooring base class.
 */
class MooringElasto : public virtual ComponentElasto {
  public:
    /** @brief Fairlead body. */
    BodyElasto& fairlead;
    /** @brief Anchor body. */
    BodyElasto& anchor;

    /**
     * @brief Constructor.
     *
     * @param[in] fairlead Fairlead body to attach to mooring.
     * @param[in] anchor Anchor body to attach to mooring.
     */
    MooringElasto(BodyElasto& fairlead, BodyElasto& anchor);

    /**
     * @brief Prestep of the mooring.
     */
    virtual void prestep(double time, double dt){};

    /**
     * @brief Returns tension at fairlead.
     */
    virtual Vector3d get_tension_fairlead() const = 0;

    /**
     * @brief Returns tension at anchor.
     */
    virtual Vector3d get_tension_anchor() const = 0;
};

/**
 * @brief Mooring system class gathering mooring lines and anchors.
 */
struct MooringSystem : public ComponentElasto {
  public:
    /** @brief List of mooring lines. */
    std::deque<std::shared_ptr<MooringElasto>> moorings;
    /** @brief List of anchors. */
    std::deque<std::shared_ptr<BodyElasto>> anchors;

    /**
     * @brief Constructor.
     */
    MooringSystem();

    virtual void build() override;
    virtual void assemble(SystemElasto& system) override;
    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual double get_mass() const override;
};

/**
 * @brief Mooring as an elastodynamic FEA component.
 *
 * Moorings are discretized into ANCF cable elements.
 */
class MooringElastoFEA : public MooringElasto, public ComponentElastoFEA {
  public:
    /** @brief Link between line and fairlead. */
    std::unique_ptr<seahowl::elasto::Link> fairlead_link;
    /** @brief Link between line and anchor. */
    std::unique_ptr<seahowl::elasto::Link> anchor_link;
    /** @brief Position of the mooring line. */
    double diameter = 0.0;
    /** @brief Axial stiffness of the mooring line. */
    double stiffness_axial = 0.0;
    /** @brief Bending stiffness of the mooring line. */
    double stiffness_bending = 0.0;
    /** @brief Linear density of the mooring line. */
    double density_linear = 0.0;
    /** @brief Unstretched length of the mooring line. */
    double length = 0.0;
    /** @brief Drag coefficient (normal) of the mooring line. */
    double drag_coefficient_normal = 2.0;
    /** @brief Drag coefficient (tangential) of the mooring line. */
    double drag_coefficient_tangential = 1.15;
    /** @brief Added mass coefficient (normal) of the mooring line. */
    double added_mass_coefficient_normal = 1.0;
    /** @brief Added mass coefficient (tangential) of the mooring line. */
    double added_mass_coefficient_tangential = 1.0;

    /**
     * @brief Constructor.
     *
     * @param[in] fairlead Fairlead body to attach to mooring.
     * @param[in] anchor Anchor body to attach to mooring.
     */
    MooringElastoFEA(BodyElasto& fairlead, BodyElasto& anchor);

    /**
     * @brief Builds the mooring (to call before assemble).
     */
    virtual void build() override;

    void build_nodes(const std::vector<ReferencePointElasto>& discretized_points);

    virtual void assemble(SystemElasto& system) override;

    /**
     * @brief Computes hydro loads on cable.
     *
     * param[in] gravitational_acceleration Gravitational acceleration vector.
     * param[in] fluid_density Density of fluid.
     */
    void compute_hydro_loads(const Vector3d& gravitational_acceleration, double fluid_density);

    /**
     * @brief Computes seabed interaction loads on cable.
     *
     * param[in] seabed Seabed model.
     */
    void compute_seabed_loads(const seahowl::env::SoilModel& seabed);

    /**
     * @brief Returns tension at fairlead.
     */
    virtual Vector3d get_tension_fairlead() const override;

    /**
     * @brief Returns tension at anchor.
     */
    virtual Vector3d get_tension_anchor() const override;

  private:
    /**
     * @brief Builds the mooring with ANCF cable elements.
     */
    void build_elements();
};

}  // namespace elasto
}  // namespace seahowl
