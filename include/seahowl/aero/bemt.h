#pragma once

#include <seahowl/aero/airfoil.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/tower_aero.h>

#include <chrono/core/ChVector.h>
#include <chrono/core/ChVector2.h>

namespace seahowl {
namespace aero {

double get_phi(const chrono::ChVector2<double>& fluid_velocity);

double get_alpha_from_phi(const double phi, const double pitch);

double get_alpha(const chrono::ChVector2<double>& fluid_velocity, const double pitch);

AirfoilCoefficients get_aero_coefficients_from_alpha(
    const double alpha,
    std::vector<AirfoilProperties>& airfoil_properties);

chrono::ChVector2<double> get_induced_velocity(BladeNodeAero& element,
                                               const chrono::ChVector2<double>& local_velocity_rotor0,
                                               const double blade_pitch = 0.0,
                                               const size_t nblades = 3,
                                               const bool tip_loss = true,
                                               const bool hub_loss = true);

void apply_tower_shadow_effect_on_wind(chrono::ChVector<double>& wind_velocity,
                                       const chrono::ChVector<double>& position,
                                       double blade_azimuth,
                                       const TowerAero& tower_aero);
}  // namespace aero
}  // namespace seahowl
