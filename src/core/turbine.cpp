#include "seahowl/core/turbine.h"

#include "seahowl/elasto/turbine_elasto.h"
#include "seahowl/aero/turbine_aero.h"
#include "seahowl/core/blade.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/servo/controller.h"

using namespace seahowl::core;
using namespace seahowl::servo;
using namespace seahowl::elasto;

Turbine::Turbine(seahowl::elasto::TurbineElasto& elasto, seahowl::aero::TurbineAero& aero)
    : elasto(elasto), aero(aero), rna(elasto.rna, aero.rna), tower(elasto.tower, aero.tower) {
    controller = std::make_shared<Controller>();
}

void Turbine::initialize(double time, double dt) {
    rna.initialize(time, dt);
    tower.initialize(time, dt);
    controller->initialize(time, dt, *this);

    aero.initialize(time, dt);
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
    // controller step
    controller->step(time, dt, *this);
    // apply torque from comtroller
    if (controller->has_torque_control) {
        auto torque_elec = controller->get_torque_elec() * gearbox_ratio * gearbox_efficiency;
        // apply torque elec to hub rigid body
        rna.elasto.rotor->body_hub->reset_loads();
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
    elasto.build();
    aero.build();
}

void Turbine::translate(Vector3d translation_vector) {
    rna.elasto.translate(translation_vector);
    tower.elasto.translate(translation_vector);
}

void Turbine::rotate(double angle, Vector3d axis) {
    rna.elasto.rotate(angle, axis);
    tower.elasto.rotate(angle, axis);
}

double Turbine::get_generated_power() const {
    // get generator rotation in rad/s scaled by gearbox ratio and efficiency
    auto rot_rads = rna.elasto.get_rpm() * (2.0 * PI / 60.0) * gearbox_ratio * gearbox_efficiency;
    // get torque elec from rotor
    auto torque_elec = controller->get_torque_elec();

    // calculate power
    auto power = torque_elec * rot_rads * generator_efficiency;
    return power;
}

double Turbine::get_generator_rpm() const {
    // get generator rotation in rad/s scaled by gearbox ratio and efficiency
    auto rpm = rna.elasto.get_rpm() * gearbox_ratio * gearbox_efficiency;
    return rpm;
}
