#include "seahowl/servo/controller.h"

#include "seahowl/core/turbine.h"
#include "seahowl/elasto/rotor_elasto.h"
#include "seahowl/aero/rotor_aero.h"

#include <spdlog/spdlog.h>

using namespace seahowl::servo;

Controller::Controller() {
    has_pitch_control = false;
    has_torque_control = false;
}

void Controller::initialize(double time, double dt, const seahowl::core::Turbine& turbine) {}

void Controller::step(double time, double dt, const seahowl::core::Turbine& turbine) {}

void Controller::poststep(double time, double dt, const seahowl::core::Turbine& turbine) {}

double Controller::get_torque_elec() const {
    return 0.0;
}

double Controller::get_collective_pitch() const {
    return 0.0;
}

double Controller::get_pitch_blade(int index_blade) const {
    return 0.0;
}

ControllerVariableTorque::ControllerVariableTorque() {
    has_pitch_control = false;
    has_torque_control = true;
}

void ControllerVariableTorque::poststep(double time, double dt, const seahowl::core::Turbine& turbine) {
    torque_elec_previous = torque_elec;
}

void ControllerVariableTorque::step(double time, double dt, const seahowl::core::Turbine& turbine) {
    if (fabs(target_rpm) > 1e-6) {
        double rpm = turbine.rna.elasto.get_rpm();
        double torque_aero = turbine.rna.elasto.get_axial_torque();

        // @todo Line below is specific to actuator disk (otherwise torque_aero is zero), need to move it
        // inside turbine.rna.elasto.get_axial_torque()
        torque_aero += turbine.rna.aero.rotor->hub_torque_aero;

        torque_elec = torque_aero * std::pow(rpm / target_rpm, 2);
        // torque_elec must be the torque at the generator --> scaled by gearbox ratio and efficiency
        torque_elec *= turbine.gearbox_efficiency / turbine.gearbox_ratio;
        // check if rpm and target_rpm have the same sign
        if (rpm / target_rpm < 0.0) {
            torque_elec = 0.0;
        }
    } else {
        spdlog::warn("Warning: target RPM of controller is too low (" + std::to_string(target_rpm) + "), not applied.");
    }
}

double ControllerVariableTorque::get_torque_elec() const {
    return torque_elec;
}
