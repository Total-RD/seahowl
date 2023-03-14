#include "seahowl/core/turbine.h"

using namespace seahowl::core;
using namespace seahowl::servo;
using namespace seahowl::elasto;

Turbine::Turbine() {
    blades.resize(0);
    rotor = Rotor();
    tower = Tower();
    controller = std::make_shared<Controller>();
}

void Turbine::init(double time, double dt) {
    rotor.init(time, dt);
    tower.init(time, dt);
    controller->init(time, dt, *this);
#ifdef HAVE_AERODYN
    if (use_aerodyn) {
        aerodyn->init(time, dt, *this);
    }
#endif
}

void Turbine::prestep(double time, double dt) {
    rotor.prestep(time, dt);
    tower.prestep(time, dt);
}

void Turbine::poststep(double time, double dt) {
    // controller step
    controller->step(time, dt, *this);
    // apply torque from comtroller
    if (controller->has_torque_control) {
        auto torque_elec = controller->get_torque_elec();
        // apply torque elec to hub rigid body
        rotor.elasto.body_hub->reset_forces();
        // torque elec is applied on hub body (locally)
        rotor.elasto.accumulate_axial_torque(-torque_elec);
    }
    // apply pitch from controller
    if (controller->has_pitch_control) {
        auto collective_pitch_increment = controller->get_collective_pitch() - rotor.elasto.pitch_collective;
        rotor.elasto.apply_collective_pitch_increment(collective_pitch_increment);
    }

    // poststeps
    rotor.poststep(time, dt);
    tower.poststep(time, dt);

    // controller poststep
    controller->poststep(time, dt, *this);
}

void Turbine::assemble(std::shared_ptr<SystemElasto> system, std::shared_ptr<MeshElasto> mesh) {
    // assemble blades
    for (auto& blade : blades) {
        blade->assemble(mesh);
    }

    // assemble rotor & tower
    rotor.assemble(system);
    tower.assemble(mesh);

    // link tower to rotor
    rotor.elasto.link_tower(tower.elasto, system);
}

void Turbine::build() {
    // build blades
    for (auto& blade : blades) {
        blade->build();
    }

    // build rotor & tower
    rotor.build(blades);
    tower.build();
}

void Turbine::translate(Vector3d translation_vector) {
    rotor.elasto.translate(translation_vector);
    tower.elasto.translate(translation_vector);
}

void Turbine::rotate(double angle, Vector3d axis) {
    rotor.elasto.rotate(angle, axis);
    tower.elasto.rotate(angle, axis);
}

void Turbine::compute_wind_loads(seahowl::aero::WindModel& wind_model, double time) {
#ifdef HAVE_AERODYN
    if (use_aerodyn) {
        aerodyn->calcul(time, *this);
        rotor.aero.compute_wind_loads_aerodyn(aerodyn->pImpl.MeshFrc, wind_model, time, tower.aero, true, true, true);
    } else {
        rotor.aero.compute_wind_loads_bemt(wind_model, time, tower.aero, true, true, true);
    }
#else
    rotor.aero.compute_wind_loads_bemt(wind_model, time, tower.aero, true, true, true);
#endif
    tower.aero.compute_wind_loads_morison(wind_model, time);
}

double Turbine::get_generated_power() const {
    // get generator rotation in rad/s scaled by gearbox ratio and efficiency
    auto rot_rads = rotor.elasto.get_rpm() * (2.0 * PI / 60.0) * gearbox_ratio * gearbox_efficiency;
    // get torque elec from rotor
    auto torque_elec = controller->get_torque_elec();

    // calculate power
    auto power = torque_elec * rot_rads * generator_efficiency;
    return power;
}

double Turbine::get_generator_rpm() const {
    // get generator rotation in rad/s scaled by gearbox ratio and efficiency
    auto rpm = rotor.elasto.get_rpm() * gearbox_ratio * gearbox_efficiency;
    return rpm;
}
