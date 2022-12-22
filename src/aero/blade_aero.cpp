#include "seahowl/aero/blade_aero.h"
#include <seahowl/aero/bemt.h>

using seahowl::aero::BladeElementAero;
using seahowl::aero::BladeAero;
using seahowl::aero::get_induced_velocity;

BladeElementAero::BladeElementAero(BladeReferencePointAero& point1, BladeReferencePointAero& point2) {
    properties = (point1 + point2) * 0.5;
    length = (point1.coordinates - point2.coordinates).Length();
}

BladeElementAero::~BladeElementAero() {}

chrono::ChVector2<double> BladeElementAero::get_induced_velocity_rotor(chrono::ChVector2<double>& local_velocity_rotor0,
                                                                       size_t nblades,
                                                                       bool tip_loss,
                                                                       bool hub_loss) {
    return get_induced_velocity(*this, local_velocity_rotor0, nblades, tip_loss, hub_loss);
}

BladeAero::BladeAero() {}

BladeAero::~BladeAero() {}

void BladeAero::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough aero reference points defined for blade.");
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
        auto element = BladeElementAero(discretized_points[ii], discretized_points[ii + 1]);
        elements.push_back(element);
        // push empty load
        loads.push_back(chrono::ChVector<double>(0.0, 0.0, 0.0));
        relative_velocities_induced.push_back(chrono::ChVector<double>(0.0, 0.0, 0.0));
        wind_velocities.push_back(chrono::ChVector<double>(0.0, 0.0, 0.0));
        wind_velocities_shadowed.push_back(chrono::ChVector<double>(0.0, 0.0, 0.0));
    }

    // get distance from tip
    compute_distances_from_tip();
}

void BladeAero::compute_distances_from_tip() {
    // this is the position of the element at the tip
    auto& element_tip_position = elements.back().properties.coordinates;
    // need to add 0.5*length of the element to get actual distance from tip
    double offset = 0.5 * elements.back().length;
    for (int ii = 0; ii < elements.size(); ii++) {
        auto& element = elements[ii];
        element.distance_from_tip = (element.properties.coordinates - element_tip_position).Length() + offset;
    }
}

void BladeAero::compute_distances_from_hub(chrono::ChVector<double> hub_apex_position, double hub_radius) {
    for (auto& element : elements) {
        element.distance_from_hub = (element.properties.coordinates - hub_apex_position).Length() - hub_radius;
    }
}

void BladeAero::compute_radii(chrono::ChVector<double> hub_apex_position) {
    for (auto& element : elements) {
        element.radius = (element.properties.coordinates - hub_apex_position).Length();
    }
}

chrono::ChVector<double> BladeAero::get_average_wind_velocity() {
    auto average = chrono::ChVector<double>(0.0, 0.0, 0.0);
    for (auto& vel : wind_velocities_shadowed) {
        average += vel;
    }
    average /= wind_velocities_shadowed.size();
    return average;
}

chrono::ChVector<double> BladeAero::get_total_load() {
    auto total = chrono::ChVector<double>(0.0, 0.0, 0.0);
    for (auto& load : loads) {
        total += load;
    }
    return total;
}