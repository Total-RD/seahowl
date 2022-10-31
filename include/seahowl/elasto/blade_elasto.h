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

/**@brief Elastodynamic model for blade */
class BladeElasto : public ComponentElastoFEA {
  public:
    std::vector<BladeReferencePointElasto> reference_points;
    std::vector<BladeReferencePointElasto> discretized_points;
    std::vector<std::shared_ptr<chrono::ChLoad<ChLoaderWeighted>>> loaders_aero;

    double pitch = 0.0;     ///< pitch of blade
    bool fpm_mode = false;  ///< FPM mode
    double azimuth0 = 0.0;  ///< Initial azimuth of blade

    BladeElasto();

    void build();
    void build_elements_tapered_timoshenko();
    void build_elements_tapered_timoshenko_fpm();
    // void build_loads(chrono::ChSystemSMC& system);
    virtual void set_damping_coefficients(double axial, double edge, double flap, double torsion);
    virtual void evaluate_position_rotation(chrono::ChVector<double>& position,
                                            chrono::ChQuaternion<double>& rotation,
                                            int element_index,
                                            double eta) const override;
    void apply_pitch_increment(double pitch_increment);
};

}  // namespace elasto
}  // namespace seahowl
