#include "seahowl/servo/controller.h"

#include "seahowl/core/turbine.h"
#include "seahowl/elasto/rotor_elasto.h"

#include <cmath>

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
    double rpm = turbine.rotor.elasto.get_rpm();
    double torque_total = turbine.rotor.elasto.get_axial_torque();
    // total_torque includes aero torque + previous elec torque
    double torque_aero = torque_total + torque_elec_previous;
    double torque_elec = torque_aero * std::pow(rpm / target_rpm, 2);
    if (rpm / target_rpm < 0.0) {
        torque_elec = 0.0;
    }
}

double ControllerVariableTorque::get_torque_elec() const {
    return torque_elec;
}
