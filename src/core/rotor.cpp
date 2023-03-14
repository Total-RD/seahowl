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
    prestep(time, dt);
    poststep(time, dt);
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
    aero.azimuth = elasto.get_azimuth();
}

void Rotor::update_positions_aero() {
    aero.hub_position = elasto.body_hub->get_position();
    aero.hub_rotation = elasto.body_hub->get_rotation();
}

void Rotor::assemble(std::shared_ptr<seahowl::elasto::SystemElasto> system) {
    elasto.assemble(system);
}

void Rotor::build(std::vector<std::shared_ptr<Blade>> blades) {
    this->blades = blades;

    // get elasto and aero blades pointers
    std::vector<std::shared_ptr<BladeElasto>> blades_elasto;
    std::vector<std::shared_ptr<BladeAero>> blades_aero;
    for (auto& blade : blades) {
        blades_elasto.push_back(blade->elasto);
        blades_aero.push_back(blade->aero);
        blade->update_positions_aero();
    }

    // build elasto
    elasto.build(blades_elasto);
    // update blade aero positions from new elasto positions
    for (auto& blade : blades) {
        blade->update_positions_aero();
        blade->aero->azimuth0 = blade->elasto->azimuth0;
    }
    // update hub position from elasto
    update_positions_aero();
    // build aero
    aero.build(blades_aero);
}
