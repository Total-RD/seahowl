#include "seahowl/elasto/mooring_elasto.h"

#include "seahowl/commons/numerics.h"
#include "seahowl/commons/utils.h"
#include "seahowl/elasto/reference_point_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"

using namespace seahowl::elasto;

MooringElasto::MooringElasto() {}

void MooringElasto::build() {
    // make reference points between fairlead and anchor
    std::vector<ReferencePointElasto> points;
    for (auto& fraction : discretization_fractions) {
        auto point = ReferencePointElasto();
        point.coordinates = fairlead_position + (anchor_position - fairlead_position) * fraction;
        point.fraction = fraction;
        points.push_back(point);
    }
    // build
    build_nodes(points);
    build_elements();
}

void MooringElasto::build_nodes(const std::vector<ReferencePointElasto>& discretized_points) {
    nodes.clear();
    const auto nnodes = discretized_points.size();

    for (size_t ii = 0; ii < nnodes; ii++) {
        auto& discretized_point = discretized_points[ii];
        auto& node_pos = discretized_point.coordinates;

        // get node main axis (direction)
        Vector3d node_axis;
        if (ii == 0) {
            node_axis = (discretized_points[ii + 1].coordinates - node_pos).normalized();
        } else if (ii == nnodes - 1) {
            node_axis = (node_pos - discretized_points[ii - 1].coordinates).normalized();
        } else {
            node_axis = (discretized_points[ii + 1].coordinates - discretized_points[ii - 1].coordinates).normalized();
        }

        // make node
        auto node = std::make_shared<NodeElastoChronoD>(node_pos, node_axis);
        nodes.push_back(node);
    };
};

void MooringElasto::build_elements() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    if (nelements <= 0) {
        throw std::runtime_error("Trying to build blade with no element.");
    }

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = std::make_shared<ElementMooringElastoChrono>();
        // add element to elements vector
        elements.push_back(element);
        // set element nodes
        element->set_nodes(nodes[ii - 1], nodes[ii]);
        // set section
        element->set_properties(density, diameter, stiffness_axial);
    }
}

void MooringElasto::compute_hydro_loads() {
    auto density = 1000.0;

    for (int ii = 0; ii < nodes.size(); ii++) {
        nodes[ii]->set_force(Vector3d(0.0, 0.0, 0.0));
    }
    for (int ii = 0; ii < elements.size(); ii++) {
        auto& element = elements[ii];
        auto element_length = std::dynamic_pointer_cast<ElementMooringElastoChrono>(element)->get_rest_length();

        // loads nodes
        for (int ii = 0; ii < 2; ii++) {
            auto dir = element->nodes[ii]->get_direction();

            // drag
            auto fluid_velocity = Vector3d(0.0, 0.0, 0.0);
            auto velocity_relative = fluid_velocity - element->nodes[ii]->get_velocity();
            auto velocity_tangent = dir * velocity_relative.dot(dir);
            auto velocity_normal = velocity_relative - velocity_tangent;

            // load drag per unit length
            auto drag_coefficient_axial = drag_coefficient;
            auto drag_coefficient_normal = drag_coefficient;
            auto load_drag_axial =
                0.5 * density * drag_coefficient * PI * diameter * velocity_tangent.norm() * velocity_tangent;
            auto load_drag_normal =
                0.5 * density * drag_coefficient * diameter * velocity_normal.norm() * velocity_normal;
            auto load_drag = load_drag_axial + load_drag_normal;

            // added mass
            auto fluid_acceleration = Vector3d(0.0, 0.0, 0.0);
            auto acceleration_relative = fluid_acceleration - element->nodes[ii]->get_acceleration();
            auto acceleration_tangent = dir * acceleration_relative.dot(dir);
            auto acceleration_normal = acceleration_relative - acceleration_tangent;
            // load added mass per unit length
            auto added_mass_coefficient_axial = added_mass_coefficient;
            auto added_mass_coefficient_normal = added_mass_coefficient;
            auto area = PI * pow(diameter, 2) / 4.0;
            auto load_added_mass_axial = density * added_mass_coefficient_axial * area * acceleration_tangent;
            auto load_added_mass_normal = density * added_mass_coefficient_normal * area * acceleration_normal;
            auto load_added_mass_fluid = density * area * fluid_acceleration;
            auto load_added_mass = load_added_mass_axial + load_added_mass_normal;

            // apply load drag over half element (each node gets half of a given element)
            auto load_half_element = (load_drag + load_added_mass) * 0.5 * element_length;
            element->nodes[ii]->set_force(element->nodes[ii]->get_force() + load_half_element);
        }
    }
}
