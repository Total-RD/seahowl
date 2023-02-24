#pragma once
#include <seahowl/elasto/elasto.h>

#include <seahowl/elasto/utils_elasto.h>  // WeightedElasto

#include <chrono/physics/ChLoad.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/fea/ChMesh.h>
#include <chrono/fea/ChNodeFEAxyzrot.h>
#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>

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

    /**
     * @brief Sets damping coefficients of the blade.
     *
     * @param[in] axial Axial damping coefficient.
     * @param[in] edge Edge damping coefficient.
     * @param[in] flap Flap damping coefficient.
     * @param[in] torsion Torsion damping coefficient.
     */
    virtual void set_damping_coefficients(double axial, double edge, double flap, double torsion);

    virtual void evaluate_position_rotation(chrono::ChVector<double>& position,
                                            chrono::ChQuaternion<double>& rotation,
                                            int element_index,
                                            double eta) const override;

    /**
     * @brief Applies pitch increment to the blade (i.e. rotates the blade around its longitudinal axis).
     *
     * @param pitch_increment Pitch increment value (in radians).
     */
    void apply_pitch_increment(double pitch_increment);

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
