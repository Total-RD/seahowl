#include "seahowl/core/rotor.h"

#include <seahowl/elasto/blade_elasto.h>

#include <memory>
#include <vector>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::aero;

Rotor::Rotor(seahowl::elasto::RotorElasto& elasto, seahowl::aero::RotorAero& aero) : elasto(elasto), aero(aero) {}

void Rotor::initialize(double time, double dt) {
    for (auto& blade : blades) {
        blade->initialize(time, dt);
        // update initial azimuth of aero blade
        blade->aero.azimuth0 = blade->elasto.azimuth0;
    }
    update_positions_aero();
    // initialize aero variables after updating positions
    aero.initialize();
}

void Rotor::prestep(double time, double dt) {
    for (auto& blade : blades) {
        blade->prestep(time, dt);
    }
}

void Rotor::poststep(double time, double dt) {
    for (auto& blade : blades) {
        blade->poststep(time, dt);
    }
    update_positions_aero();
}

void Rotor::update_positions_aero() {
    // azimuth
    aero.azimuth = elasto.get_azimuth();
    // body_hub
    aero.body_hub.set_position(elasto.body_hub->get_position());
    aero.body_hub.set_rotation(elasto.body_hub->get_rotation());
    aero.body_hub.set_velocity(elasto.body_hub->get_velocity());
    aero.body_hub.set_acceleration(elasto.body_hub->get_acceleration());
    aero.body_hub.set_rotational_velocity(elasto.body_hub->get_rotational_velocity());
    aero.body_hub.set_rotational_acceleration(elasto.body_hub->get_rotational_acceleration());
    // body_nacelle
    aero.body_nacelle.set_position(elasto.body_nacelle->get_position());
    aero.body_nacelle.set_rotation(elasto.body_nacelle->get_rotation());
    aero.body_nacelle.set_velocity(elasto.body_nacelle->get_velocity());
    aero.body_nacelle.set_acceleration(elasto.body_nacelle->get_acceleration());
    aero.body_nacelle.set_rotational_velocity(elasto.body_nacelle->get_rotational_velocity());
    aero.body_nacelle.set_rotational_acceleration(elasto.body_nacelle->get_rotational_acceleration());
}

void Rotor::build() {
    // build elasto
    elasto.build();
    // update hub position from elasto
    update_positions_aero();
    // build aero
    aero.build();
}
