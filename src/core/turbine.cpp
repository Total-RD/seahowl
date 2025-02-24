#include "seahowl/core/turbine.h"

#include "seahowl/elasto/turbine_elasto.h"
#include "seahowl/aero/turbine_aero.h"
#include "seahowl/core/blade.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/elasto/floater_elasto.h"
#include "seahowl/servo/controller.h"

#include <spdlog/spdlog.h>

using namespace seahowl::core;
using namespace seahowl::servo;
using namespace seahowl::elasto;

Turbine::Turbine(seahowl::elasto::TurbineElasto& elasto, seahowl::aero::TurbineAero& aero)
    : elasto(elasto), aero(aero), rna(elasto.rna, aero.rna), tower(elasto.tower, aero.tower) {
    controller = std::make_shared<Controller>();
}

void Turbine::initialize_this(double time, double dt) {
    rna.initialize(time, dt);
    tower.initialize(time, dt);
    controller->initialize(time, dt, *this);

    aero.initialize(time, dt);

    if (foundation) {
        foundation->initialize(time, dt);
    }

    spdlog::info("Initialized turbine of total mass {:.4}kg.", elasto.get_mass());
}

void Turbine::apply_control(double time, double dt) {
    // controller step
    controller->step(time, dt, *this);

    // apply electrical torque from controller
    if (controller->has_torque_control) {
        auto torque_elec = controller->get_torque_elec() * gearbox_ratio * gearbox_efficiency;
        // torque elec is applied on hub body (locally)
        rna.elasto.accumulate_electrical_torque(torque_elec);
    }

    // apply pitch from controller
    if (controller->has_pitch_control) {
        if (rna.blades.size() >= 1 && rna.blades.size() <= 3) {
            // individual pitch only works with rotors from 1 to 3 blades
            for (int idx_blade = 0; idx_blade < rna.blades.size(); idx_blade++) {
                auto& blade = *rna.blades[idx_blade];
                if (blade.elasto.actuator_pitch->is_fixed_actuator()) {
                    auto blade_pitch_increment = controller->get_pitch_blade(idx_blade) - blade.elasto.get_pitch();
                    blade.apply_pitch_increment(blade_pitch_increment);
                } else {
                    std::vector<double> time_array{time, time + dt};
                    std::vector<double> angle_array{blade.elasto.get_pitch(), controller->get_pitch_blade(idx_blade)};
                    blade.elasto.actuator_pitch->set_timeseries(time_array, angle_array);
                }
            }
        } else {
            // collective pitch for more than 3 blades or 0 blade (e.g. actuator disk)
            for (auto& blade : rna.blades) {
                if (blade->elasto.actuator_pitch->is_fixed_actuator()) {
                    auto collective_pitch_increment =
                        controller->get_collective_pitch() - rna.elasto.rotor->pitch_collective;
                    rna.elasto.rotor->apply_collective_pitch_increment(collective_pitch_increment);
                } else {
                    std::vector<double> time_array{time, time + dt};
                    std::vector<double> angle_array{blade->elasto.get_pitch(), controller->get_collective_pitch()};
                    blade->elasto.actuator_pitch->set_timeseries(time_array, angle_array);
                }
            }
        }
        // store collective pitch value from controller for information purposes
        // (even if pitch was applied individually to blades)
        rna.elasto.rotor->pitch_collective = controller->get_collective_pitch();
    }

    // apply yaw control from controller
    if (controller->has_yaw_control) {
        auto yaw_rate = controller->get_yaw_rate();
        auto yaw_increment = yaw_rate * dt;
        rna.elasto.apply_yaw_increment(yaw_increment);
        rna.update_positions_aero();
    }
}

void Turbine::prestep(double time, double dt) {
    // presteps
    rna.prestep(time, dt);
    tower.prestep(time, dt);

    if (foundation) {
        foundation->prestep(time, dt);
    }
}

void Turbine::poststep(double time, double dt) {
    // hub loads
    // reset external loads applied on RNA
    rna.elasto.reset_loads();
    // apply aero torque losses from gearbox efficiency for next step
    auto aero_torque = rna.elasto.get_axial_torque();
    if (controller->has_torque_control) {
        // remove previously applied electrical torque from total rotor torque
        aero_torque += controller->get_torque_elec() * gearbox_ratio * gearbox_efficiency;
    }
    rna.elasto.rotor->accumulate_axial_torque(-aero_torque * (1.0 - gearbox_efficiency));

    // poststeps
    rna.poststep(time, dt);
    tower.poststep(time, dt);

    if (foundation) {
        foundation->poststep(time, dt);
    }

    // controller poststep
    controller->poststep(time, dt, *this);
}

void Turbine::build() {
    elasto.build();
    aero.build();
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

void Turbine::apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time) {
    aero.compute_fluid_loads(fluid_model, time);
}

void Turbine::apply_soil_model(seahowl::env::SoilModel& soil_model, double time) {
    if (foundation) {
        foundation->apply_soil_model(soil_model, time);
    }
}
