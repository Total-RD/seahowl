#include "seahowl/core/turbine_floating.h"
#include "seahowl/elasto/chrono_adapters.h"

using namespace seahowl::core;
using namespace seahowl::servo;
using namespace seahowl::elasto;
using namespace seahowl::hydro;

TurbineFloating::TurbineFloating(seahowl::elasto::TurbineElasto& elasto, seahowl::aero::TurbineAero& aero)
    : Turbine(elasto, aero) {
    link_floater_tower = std::move(std::make_unique<seahowl::elasto::LinkChrono>());
}

void TurbineFloating::initialize(double time, double dt) {
    // parent class initialize
    Turbine::initialize(time, dt);

    // initialize floater
    if (floater) {
        floater->initialize(time, dt);
    }
}

void TurbineFloating::assemble(seahowl::elasto::SystemElasto& system) {
    if (floater) {
        // assemble floater
        floater->assemble(system);
        // make floater-tower connection
        auto& body_floater = floater->get_tower_connection_body();
        auto& node_tower = *(tower.elasto.nodes.front().get());
        link_floater_tower->initialize(body_floater, node_tower);
        link_floater_tower->set_constraints(true, true, true, true, true, true);
        // add link to system
        system.add(*(link_floater_tower.get()));
    }
}

void TurbineFloating::prestep(double time, double dt) {
    // parent class prestep
    Turbine::prestep(time, dt);
}

void TurbineFloating::poststep(double time, double dt) {
    // parent class poststep
    Turbine::poststep(time, dt);
}

void TurbineFloating::build() {
    // parent class build
    Turbine::build();
}

void TurbineFloating::translate(Vector3d translation_vector) {
    // parent class translate
    Turbine::translate(translation_vector);
    // floater translate
    if (floater) {
        floater->translate(translation_vector);
    }
}

void TurbineFloating::rotate(double angle, Vector3d axis) {
    // parent class rotate
    Turbine::rotate(angle, axis);
    // floater rotate
    if (floater) {
        floater->rotate(angle, axis);
    }
}
