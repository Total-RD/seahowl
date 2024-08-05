#pragma once

#include "seahowl/elasto/component_elasto.h"
#include "seahowl/elasto/entities_elasto.h"
#include "seahowl/commons/numerics.h"
#include "seahowl/env/soil_models.h"
#include "seahowl/env/fluid_models.h"

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
     * @brief Sets length of the mooring line.
     *
     * @param[in] length Length of the mooring line.
     */
    virtual void set_length(double length) = 0;

    /**
     * @brief Sets diameter of the mooring line.
     *
     * @param[in] diameter Diameter of the mooring line.
     */
    virtual void set_diameter(double diameter) = 0;

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

    /**
     * @brief Returns total length of mooring.
     */
    virtual double get_length() const = 0;
};

/**
 * @brief Mooring system class gathering mooring lines and anchors.
 */
struct MooringSystemElasto : public ComponentElasto {
  public:
    /** @brief List of mooring lines. */
    std::deque<std::shared_ptr<MooringElasto>> moorings{};
    /** @brief List of anchors. */
    std::deque<std::shared_ptr<BodyElasto>> anchors{};

    /**
     * @brief Constructor.
     */
    MooringSystemElasto();

    /**
     * @brief Adds mooring to mooring system.
     *
     * @param[in] mooring Mooring to add to mooring system.
     */
    void add_mooring(std::shared_ptr<MooringElasto> mooring);

    virtual void build() override;
    virtual void presetup(double fraction) override;
    virtual void prestep(double time, double dt);
    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual double get_mass() const override;

  protected:
    virtual void assemble_this(SystemElasto& system) override;
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

    /**
     * @brief Constructor.
     *
     * @param[in] fairlead Fairlead body to attach to mooring.
     * @param[in] anchor Anchor body to attach to mooring.
     */
    MooringElastoFEA(BodyElasto& fairlead, BodyElasto& anchor);

    /**
     * @brief Sets length of the mooring line.
     *
     * @param[in] length Length of the mooring line.
     */
    virtual void set_length(double length) override;

    /**
     * @brief Sets diameter of the mooring line.
     *
     * @param[in] diameter Diameter of the mooring line.
     */
    virtual void set_diameter(double diameter) override;

    /**
     * @brief Builds the mooring (to call before assemble).
     */
    virtual void build() override;

    /**
     * @brief Presetup of mooring.
     *
     * @param[in] fraction Fraction of presetup phase, starting at 0.0 and ending at 1.0.
     */
    virtual void presetup(double fraction) override;

    void build_nodes(const std::vector<ReferencePointElasto>& discretized_points);

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

    /**
     * @brief Returns total length of mooring.
     */
    virtual double get_length() const override;

  protected:
    virtual void assemble_this(SystemElasto& system) override;

  private:
    /** @brief Actual length of mooring before presetup. */
    double length0 = -1.0;
    /**
     * @brief Builds the mooring with ANCF cable elements.
     */
    void build_elements();

    Vector3d gravitational_acceleration{0.0, 0.0, -9.81};  // for buoyancy calculations
};

}  // namespace elasto
}  // namespace seahowl
