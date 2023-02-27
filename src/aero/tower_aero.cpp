#include "seahowl/aero/tower_aero.h"

using seahowl::aero::TowerElementAero;
using seahowl::aero::TowerAero;
using seahowl::Vector3d;

TowerElementAero::TowerElementAero(const TowerReferencePointAero& point1, const TowerReferencePointAero& point2) {
    properties = (point1 + point2) * 0.5;
    length = (point1.coordinates - point2.coordinates).Length();
}

TowerAero::TowerAero() {}

void TowerAero::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough aero reference points defined for tower.");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    } else if (discretization_fractions.size() == 1) {
        double npoints = discretization_fractions[0] + 1;
        double dp = 1.0 / (npoints - 1);
        discretization_fractions.clear();
        for (int ii = 0; ii < int(npoints); ii++) {
            discretization_fractions.push_back(ii * dp);
        }
    }

    // build
    discretized_points = seahowl::core::get_discretized_points(discretization_fractions, reference_points);
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        // make element
        auto element = TowerElementAero(discretized_points[ii], discretized_points[ii + 1]);
        elements.push_back(element);
        // push empty load
        loads.push_back(Vector3d(0.0, 0.0, 0.0));
    }
}

void TowerAero::compute_wind_loads_morison(WindModel& wind_model, double time) {
    auto density = wind_model.get_density();
    for (int ii = 0; ii < elements.size(); ii++) {
        auto& element = elements[ii];
        auto& properties = element.properties;

        // get fluid relative velocity
        auto wind_velocity = wind_model.get_wind_velocity(properties.coordinates, time);
        auto velocity_relative = wind_velocity - properties.velocity;
        auto dir = Vector3d(properties.rotation.GetVector());  // tangent direction
        auto dot = velocity_relative.dot(dir);
        auto velocity_tangent = dir * dot;
        auto velocity_normal = velocity_relative - velocity_tangent;

        auto length = element.length;
        auto diameter = properties.diameter;
        auto cd = properties.drag_coefficient;
        auto load_drag =
            0.5 * density * cd * chrono::CH_C_PI * diameter * velocity_normal.Length() * velocity_normal * length;

        loads[ii] = load_drag;
    }
}
