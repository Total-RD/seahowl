#include "seahowl/hydro/morison.h"

#include "seahowl/env/fluid_models.h"

#include <spdlog/spdlog.h>

using namespace seahowl;
using namespace seahowl::hydro;

HydroCoefficients HydroCoefficients::operator*(const double factor) const {
    HydroCoefficients new_point = *this;
    new_point.drag_normal *= factor;
    new_point.drag_axial *= factor;
    new_point.added_mass_normal *= factor;
    new_point.added_mass_axial *= factor;
    new_point.buoyancy_factor *= factor;
    new_point.inertia_factor *= factor;
    new_point.nodal_acceleration_factor *= factor;
    return new_point;
};

HydroCoefficients HydroCoefficients::operator+(const HydroCoefficients& other) const {
    HydroCoefficients new_point = *this;
    new_point.drag_normal += other.drag_normal;
    new_point.drag_axial += other.drag_axial;
    new_point.added_mass_normal += other.added_mass_normal;
    new_point.added_mass_axial += other.added_mass_axial;
    new_point.buoyancy_factor += other.buoyancy_factor;
    new_point.inertia_factor += other.inertia_factor;
    new_point.nodal_acceleration_factor += other.nodal_acceleration_factor;
    return new_point;
};

MorisonNode::MorisonNode() {}

void MorisonNode::compute_fluid_loads(const env::FluidModel& fluid_model, double time) {
    // reset total load
    load = Vector3d(0.0, 0.0, 0.0);

    auto position = get_position();
    auto velocity = get_velocity();
    auto area = PI * pow(diameter * 0.5, 2);

    // fluid density
    double fluid_density = fluid_model.get_fluid_density(position, time);
    // fluid velocity
    auto velocity_fluid = fluid_model.get_fluid_velocity(position, time);

    auto dir = get_direction();  // axial direction
    auto velocity_relative = velocity_fluid - velocity;
    auto velocity_relative_axial = dir * velocity_relative.dot(dir);
    auto velocity_relative_normal = velocity_relative - velocity_relative_axial;

    // drag
    auto load_drag_normal = 0.5 * fluid_density * coefficients.drag_normal * diameter *
                            velocity_relative_normal.norm() * velocity_relative_normal;
    auto load_drag_axial = 0.5 * fluid_density * coefficients.drag_axial * diameter * PI *
                           velocity_relative_axial.norm() * velocity_relative_axial;
    load += load_drag_normal + load_drag_axial;

    if (coefficients.inertia_factor != 0.0) {
        // fluid acceleration
        auto acceleration_fluid = fluid_model.get_fluid_acceleration(position, time);
        // relative acceleration
        auto acceleration = get_acceleration() * coefficients.nodal_acceleration_factor;
        auto acceleration_relative = acceleration_fluid - acceleration;
        auto acceleration_relative_axial = dir * acceleration_relative.dot(dir);
        auto acceleration_relative_normal = acceleration_relative - acceleration_relative_axial;

        // added mass (with Cm = 1 + Ca)
        auto load_added_mass_fluid = fluid_density * area * acceleration_fluid;
        auto load_added_mass_normal =
            fluid_density * area * coefficients.added_mass_normal * acceleration_relative_normal;
        auto load_added_mass_axial = fluid_density * area * coefficients.added_mass_axial * acceleration_relative_axial;
        // total inertia load
        auto load_inertia =
            (load_added_mass_fluid + load_added_mass_normal + load_added_mass_axial) * coefficients.inertia_factor;
        load += load_inertia;
    }

    // buoyancy
    Vector3d gravitational_acceleration{0.0, 0.0, -9.81};
    auto load_buoyancy = fluid_density * area * (-gravitational_acceleration);
    load += load_buoyancy * coefficients.buoyancy_factor;
}

MorisonElement::MorisonElement(const MorisonNode& node1, const MorisonNode& node2) : node1(node1), node2(node2) {
    length = (node1.get_position() - node2.get_position()).norm();
}

Vector3d MorisonElement::get_load() const {
    return 0.5 * (node1.load + node2.load) * length;
}

Vector3d MorisonElement::get_position() const {
    return 0.5 * (node1.get_position() + node2.get_position());
}

Quaternion MorisonElement::get_rotation() const {
    // returning rotation of node1
    // TODO: average rotation of node1 and node2
    return node1.get_rotation();
}

MorisonPlate::MorisonPlate() {}

void MorisonPlate::compute_fluid_loads(const env::FluidModel& fluid_model, double time) {
    auto area = PI * pow(diameter * 0.5, 2);

    // vector pointing inwards of the plate
    auto global_direction = get_direction();  // vector pointing inwards of the plate
    if (!reverse_direction) {
        global_direction *= -1.0;
    }

    auto position = get_position();
    auto velocity = get_velocity();
    // fluid density
    double fluid_density = fluid_model.get_fluid_density(position, time);
    // fluid velocity
    auto velocity_fluid = fluid_model.get_fluid_velocity(position, time);
    auto velocity_relative = velocity_fluid - velocity;

    load = Vector3d(0.0, 0.0, 0.0);
    auto dot = velocity_relative.dot(global_direction);
    if (dot > 0) {
        // get magnitude of drag
        double load_drag_area = 0.5 * fluid_density * drag_coefficient * area * dot * dot;
        // project in global direction
        load = load_drag_area * global_direction;
    }
}
