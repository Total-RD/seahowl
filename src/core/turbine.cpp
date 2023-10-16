#include "seahowl/core/turbine.h"

#include "seahowl/elasto/turbine_elasto.h"
#include "seahowl/aero/turbine_aero.h"
#include "seahowl/core/blade.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/servo/controller.h"

#include <spdlog/spdlog.h>

using namespace seahowl::core;
using namespace seahowl::servo;
using namespace seahowl::elasto;

Turbine::Turbine(seahowl::elasto::TurbineElasto& elasto, seahowl::aero::TurbineAero& aero)
    : elasto(elasto), aero(aero), rna(elasto.rna, aero.rna), tower(elasto.tower, aero.tower) {
    controller = std::make_shared<Controller>();
}

void Turbine::initialize(double time, double dt) {
    spdlog::info("Initializing turbine.");

    rna.initialize(time, dt);
    tower.initialize(time, dt);
    controller->initialize(time, dt, *this);

    aero.initialize(time, dt);

    spdlog::info("Initializing turbine finished.");
}

void Turbine::prestep(double time, double dt) {
    rna.prestep(time, dt);
    tower.prestep(time, dt);

    if (aero.use_disktheory) {
        rna.elasto.rotor->body_hub->accumulate_torque(Vector3d(rna.aero.torque_aero, 0, 0), true);
        rna.elasto.rotor->body_hub->accumulate_force(Vector3d(rna.aero.thrust_aero, 0, 0), true);
    }
}

void Turbine::poststep(double time, double dt) {
    // hub loads
    // reset external loads applied on rotor hub
    rna.elasto.rotor->body_hub->reset_loads();
    // apply aero torque losses from gearbox efficiency for next step
    auto aero_torque = rna.elasto.get_axial_torque();
    if (controller->has_torque_control) {
        // remove previously applied electrical torque from total rotor torque
        aero_torque += controller->get_torque_elec() * gearbox_ratio * gearbox_efficiency;
    }
    rna.elasto.accumulate_axial_torque(-aero_torque * (1.0 - gearbox_efficiency));

    // controller step
    controller->step(time, dt, *this);
    // apply torque from controller
    if (controller->has_torque_control) {
        auto torque_elec = controller->get_torque_elec() * gearbox_ratio * gearbox_efficiency;
        // torque elec is applied on hub body (locally)
        rna.elasto.accumulate_axial_torque(-torque_elec);
    }
    // apply pitch from controller
    if (controller->has_pitch_control) {
        auto collective_pitch_increment = controller->get_collective_pitch() - rna.elasto.rotor->pitch_collective;
        rna.elasto.rotor->apply_collective_pitch_increment(collective_pitch_increment);
        if (rna.blades.size() <= 3) {
            // individual pitch only works with up to 3 blades
            for (int idx_blade = 0; idx_blade < rna.blades.size(); idx_blade++) {
                auto& blade = rna.blades[idx_blade]->elasto;
                // individual pitch increment difference with collective pitch increment that was already applied
                auto blade_pitch_increment =
                    (controller->get_pitch_blade(idx_blade) - collective_pitch_increment) - blade.pitch;
                blade.apply_pitch_increment(blade_pitch_increment);
            }
        }
    }

    // poststeps
    rna.poststep(time, dt);
    tower.poststep(time, dt);

    // controller poststep
    controller->poststep(time, dt, *this);
}

void Turbine::build() {
    spdlog::info("Building turbine.");

    elasto.build();
    aero.build();

    spdlog::info("Building turbine finished.");
}

void Turbine::translate(Vector3d translation_vector) {
    rna.elasto.translate(translation_vector);
    tower.elasto.translate(translation_vector);
}

void Turbine::rotate(double angle, Vector3d axis) {
    rna.elasto.rotate(angle, axis);
    tower.elasto.rotate(angle, axis);
}

double Turbine::get_shaft_power() const {
    // get generator rotation in rad/s scaled by gearbox ratio and efficiency
    auto rot_rads = rna.elasto.get_rpm() * (2.0 * PI / 60.0) * gearbox_ratio;
    // get torque elec from rotor
    auto torque_elec = controller->get_torque_elec();

    // calculate power
    auto power = torque_elec * rot_rads;
    return power;
}

double Turbine::get_generated_power() const {
    // get generator rotation in rad/s scaled by gearbox ratio and efficiency
    auto rot_rads = get_generator_rpm() * (2.0 * PI / 60.0);
    // get torque elec from rotor
    auto torque_elec = controller->get_torque_elec();

    // calculate power
    auto power = torque_elec * rot_rads * generator_efficiency;
    return power;
}

double Turbine::get_generator_rpm() const {
    // get generator rotation in rad/s scaled by gearbox ratio and efficiency
    auto rpm = rna.elasto.get_rpm() * gearbox_ratio;
    return rpm;
}
