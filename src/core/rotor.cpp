#include "seahowl/core/rotor.h"

#include <seahowl/elasto/blade_elasto.h>

#include <chrono/physics/ChBody.h>

#include <memory>
#include <vector>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::aero;

Rotor::Rotor() {
    elasto = RotorElasto();
    aero = RotorAero();
}

void Rotor::init(double time, double dt) {
    for (auto& blade : blades) {
        blade->init(time, dt);
    }
    update_positions_aero();
    aero.compute_chords_solidity();
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
    aero.compute_chords_solidity();
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

void Rotor::assemble(std::shared_ptr<seahowl::elasto::SystemElasto> system, std::shared_ptr<MeshElasto> mesh) {
    for (auto& blade : blades) {
        blade->assemble(mesh);
    }

    elasto.assemble(system);
}

void Rotor::build() {
    // build blades
    for (auto& blade : blades) {
        blade->build();
    }

    // get elasto and aero blades pointers
    std::vector<std::shared_ptr<BladeElasto>> blades_elasto;
    std::vector<std::shared_ptr<BladeAero>> blades_aero;
    for (auto& blade : blades) {
        blades_elasto.push_back(blade->elasto);
        blades_aero.push_back(blade->aero);
    }

    // build elasto
    elasto.build();
    // update blade aero positions from new elasto positions
    for (auto& blade : blades) {
        blade->update_positions_aero();
        blade->aero->azimuth0 = blade->elasto->azimuth0;
    }
    // update hub position from elasto
    update_positions_aero();
    // build aero
    aero.build();
}
