#pragma once

#include <seahowl/elasto/component_elasto.h>

namespace seahowl {
namespace elasto {

/**
 * @brief Blade of wind turbine as an elastodynamic FEA component.
 *
 * Blades are discretized into beam elements that are either 1) simple Timoshenko elements described through lineic
 * density, flap and edge stiffnesses, or 2) Fully Populated Matrix (FPM) Timoshenko elements with 6x6 mass and
 * stiffness matrices.
 */
class BladeElasto : public ComponentElastoFEA {
  public:
    /** @brief List of reference points describing the blade properties along its longitudinal axis. */
    std::vector<BladeReferencePointElasto> reference_points;
    /** @brief List of discretized points (interpolated reference points) describing the blade properties. */
    std::vector<BladeReferencePointElasto> discretized_points;
    /** @brief Pitch of the blade (in radians). */
    double pitch = 0.0;
    /** @brief Whether the blade uses FPM or simple Timoshenko elements. */
    bool fpm_mode = false;
    /** @brief Initial azimuth of the blade relative to rotor azimuth (in radians). */
    double azimuth0 = 0.0;

    /**
     * @brief Constructor.
     */
    BladeElasto();

    /**
     * @brief Builds the blade (to call before assemble).
     */
    void build();

    virtual void evaluate_position_rotation(Vector3d& position,
                                            Quaternion& rotation,
                                            int element_index,
                                            double eta) const override;

    /**
     * @brief Applies pitch increment to the blade (i.e. rotates the blade around its longitudinal axis).
     *
     * @param pitch_increment Pitch increment value (in radians).
     */
    void apply_pitch_increment(double pitch_increment);

    /**
     * @brief Returns blade root moment (first node of first element of blade).
     */
    Vector3d get_blade_root_moment() const;

  private:
    /**
     * @brief Builds the blade with simple Timoshenko elements (lineic density, flap stiffness, edge stiffness).
     */
    void build_elements_tapered_timoshenko();

    /**
     * @brief Builds the blade with FPM Timoshenko elements (6x6 mass and stiffness matrices).
     */
    void build_elements_tapered_timoshenko_fpm();
};

}  // namespace elasto
}  // namespace seahowl
