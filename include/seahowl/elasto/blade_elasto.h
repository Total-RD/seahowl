#pragma once

#include "seahowl/elasto/component_elasto.h"
#include "seahowl/elasto/reference_point_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"

namespace seahowl {
namespace elasto {

/**
 * @brief Base class for blade of wind turbine.
 */
class BladeElasto : public virtual ComponentElasto {
  public:
    /** @brief Body at the root of the blade. */
    std::unique_ptr<BodyElastoChrono> body_root;
    /** @brief Body for mounting point of blade (to link to other structures, e.g. hub). */
    std::unique_ptr<BodyElastoChrono> body_mount;
    /** @brief Link between blade and pitch axis body. */
    std::unique_ptr<Link> link_root;
    /** @brief Link between blade root and mounting point. */
    std::unique_ptr<Link> link_root_mount;
    /** @brief Link between blade and body (usually hub). */
    std::unique_ptr<Link> link_blade;
    /** @brief Pitch of the blade (in radians). */
    double pitch = 0.0;
    /** @brief Initial azimuth of the blade relative to rotor azimuth (in radians). */
    double azimuth0 = 0.0;
    /** @brief Precone of the blade (in radians). */
    double precone = 0.0;
    /** @brief List of reference points describing the blade properties along its longitudinal axis. */
    std::vector<BladeReferencePointElasto> reference_points;
    /** @brief Discretization fractions (normalized abscissa) in the range [0, 1] to discretize the FEA component. */
    std::vector<double> discretization_fractions;

    BladeElasto();

    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual double get_mass() const override;

    /**
     * @brief Applies pitch increment to the blade (i.e. rotates the blade around its longitudinal axis).
     *
     * @param pitch_increment Pitch increment value (in radians).
     */
    virtual void apply_pitch_increment(double pitch_increment);

    /**
     * @brief Returns current pitch of blade.
     */
    double get_pitch() const;

    /**
     * @brief Returns blade root moment.
     */
    virtual Vector3d get_blade_root_moment() const = 0;

    /**
     * @brief Returns blade root force.
     */
    virtual Vector3d get_blade_root_force() const = 0;

    virtual EntityDynamicEigen get_entity_along_blade(double eta, int element_index = 0) const = 0;

    virtual void accumulate_load_along_blade(const Vector3d& load,
                                             const Vector3d& moment,
                                             int element_index,
                                             double eta,
                                             const Vector3d& offset) = 0;

    virtual void attach_blade_to_body(const BodyElasto& body);

  protected:
    /** @brief Whether the blade is mounted (e.g. on a rotor) or not. */
    bool is_mounted = false;

    virtual void assemble_this(SystemElasto& system) override;

    virtual void update_root_constraint() = 0;
};

/**
 * @brief Blade of wind turbine as an elastodynamic FEA component.
 *
 * Blades are discretized into beam elements that are either 1) simple Timoshenko elements described through lineic
 * density, flap and edge stiffnesses, or 2) Fully Populated Matrix (FPM) Timoshenko elements with 6x6 mass and
 * stiffness matrices.
 */
class BladeElastoFEA : public BladeElasto, public ComponentElastoFEA {
  public:
    /** @brief List of discretized points (interpolated reference points) describing the blade properties. */
    std::vector<BladeReferencePointElasto> discretized_points;
    /** @brief Whether the blade uses FPM or simple Timoshenko elements. */
    bool fpm_mode = false;
    using BladeElasto::discretization_fractions;

    /**
     * @brief Constructor.
     */
    BladeElastoFEA();

    virtual void build() override;
    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual double get_mass() const override;

    virtual void evaluate_position_rotation(Vector3d& position,
                                            Quaternion& rotation,
                                            int element_index,
                                            double eta) const override;

    virtual Vector3d get_blade_root_moment() const override;
    virtual Vector3d get_blade_root_force() const override;
    virtual EntityDynamicEigen get_entity_along_blade(double eta, int element_index = 0) const override;
    virtual void accumulate_load_along_blade(const Vector3d& load,
                                             const seahowl::Vector3d& moment,
                                             int element_index,
                                             double eta,
                                             const Vector3d& offset) override;

  private:
    virtual void assemble_this(SystemElasto& system) override;

    /**
     * @brief Builds the blade with simple Timoshenko elements (lineic density, flap stiffness, edge stiffness).
     */
    void build_elements_tapered_timoshenko();

    /**
     * @brief Builds the blade with FPM Timoshenko elements (6x6 mass and stiffness matrices).
     */
    void build_elements_tapered_timoshenko_fpm();

    virtual void update_root_constraint() override;
};

/**
 * @brief Blade of wind turbine as a rigid blade.
 *
 * The blade is considered rigid and laying on the reference (local) Z axis.
 */
class BladeElastoRigid : public BladeElasto {
  public:
    /**
     * @brief Constructor.
     */
    BladeElastoRigid();

    virtual void build() override;
    virtual void rotate(double angle, const Vector3d& axis) const override;
    virtual void translate(const Vector3d& translation_vector) const override;
    virtual double get_mass() const override;

    virtual Vector3d get_blade_root_moment() const override;
    virtual Vector3d get_blade_root_force() const override;
    virtual EntityDynamicEigen get_entity_along_blade(double eta, int element_index = 0) const override;
    virtual void reset_loads() override;
    virtual void accumulate_load_along_blade(const Vector3d& load,
                                             const seahowl::Vector3d& moment,
                                             int element_index,
                                             double eta,
                                             const Vector3d& offset) override;

  private:
    /** @brief Length of the blade. */
    double length = 0.0;
    /** @brief Body at the COG of the blade. */
    std::unique_ptr<BodyElastoChrono> body_cog;

    virtual void assemble_this(SystemElasto& system) override;

    virtual void update_root_constraint() override;
};

}  // namespace elasto
}  // namespace seahowl
