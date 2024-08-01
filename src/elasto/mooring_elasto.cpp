#include "seahowl/elasto/mooring_elasto.h"

#include "seahowl/commons/numerics.h"
#include "seahowl/commons/utils.h"
#include "seahowl/elasto/reference_point_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"

#include <spdlog/spdlog.h>

using namespace seahowl::elasto;

MooringSystemElasto::MooringSystemElasto() {}

void MooringSystemElasto::prestep(double time, double dt) {
    for (auto& mooring : moorings) {
        mooring->prestep(time, dt);
    }
}

void MooringSystemElasto::build() {
    for (auto& mooring : moorings) {
        mooring->build();
    }
}

void MooringSystemElasto::assemble_this(SystemElasto& system) {
    for (auto& mooring : moorings) {
        mooring->assemble(system);
    }
    for (auto& anchor : anchors) {
        system.add(*anchor);
    }
}

void MooringSystemElasto::rotate(double angle, const Vector3d& axis) const {
    // moorings
    for (auto& mooring : moorings) {
        mooring->rotate(angle, axis);
    }
    // anchors
    for (auto& anchor : anchors) {
        anchor->rotate(angle, axis);
    }
}

void MooringSystemElasto::translate(const Vector3d& translation_vector) const {
    // moorings
    for (auto& mooring : moorings) {
        mooring->translate(translation_vector);
    }
    // anchors
    for (auto& anchor : anchors) {
        anchor->translate(translation_vector);
    }
}

double MooringSystemElasto::get_mass() const {
    double mass_total = 0.0;
    // moorings
    for (auto& mooring : moorings) {
        mass_total += mooring->get_mass();
    }
    // anchors
    for (auto& anchor : anchors) {
        mass_total += anchor->get_mass();
    }
    return mass_total;
}

MooringElasto::MooringElasto(BodyElasto& fairlead, BodyElasto& anchor) : fairlead(fairlead), anchor(anchor) {}

MooringElastoFEA::MooringElastoFEA(BodyElasto& fairlead, BodyElasto& anchor) : MooringElasto(fairlead, anchor) {
    fairlead_link = std::make_unique<seahowl::elasto::LinkChronoCable>();
    anchor_link = std::make_unique<seahowl::elasto::LinkChronoCable>();
}

void MooringElastoFEA::build() {
    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        throw std::runtime_error("Mooring elasto discretization was not defined.");
    } else if (discretization_fractions.size() == 1) {
        double npoints = discretization_fractions[0] + 1;
        double dp = 1.0 / (npoints - 1);
        discretization_fractions.clear();
        for (int ii = 0; ii < int(npoints); ii++) {
            discretization_fractions.push_back(ii * dp);
        }
    }

    // make reference points between fairlead and anchor
    std::vector<ReferencePointElasto> points;
    for (auto& fraction : discretization_fractions) {
        auto point = ReferencePointElasto();
        point.coordinates = fairlead.get_position() + (anchor.get_position() - fairlead.get_position()) * fraction;
        point.fraction = fraction;
        points.push_back(point);
    }
    // build
    build_nodes(points);
    build_elements();
}

void MooringElastoFEA::build_nodes(const std::vector<ReferencePointElasto>& discretized_points) {
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

    // initialize links
    fairlead_link->initialize(*nodes.front(), fairlead);
    anchor_link->initialize(*nodes.back(), anchor);
};

void MooringElastoFEA::build_elements() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    if (nelements <= 0) {
        throw std::runtime_error("Trying to build mooring with no element.");
    }

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = std::make_shared<ElementMooringElastoChrono>();
        // add element to elements vector
        elements.push_back(element);
        // set element nodes
        element->set_nodes(nodes[ii - 1], nodes[ii]);
        // set section
        auto area = PI * pow(diameter, 2) / 4.0;
        auto density = density_linear / area;
        element->set_properties(density, diameter, stiffness_axial, stiffness_bending);
    }
}

void MooringElastoFEA::assemble_this(SystemElasto& system) {
    ComponentElastoFEA::assemble_this(system);
    system.add(*fairlead_link);
    system.add(*anchor_link);
    gravitational_acceleration = system.get_gravitational_acceleration();
}

void MooringElastoFEA::compute_seabed_loads(const seahowl::env::SoilModel& seabed) {
    for (auto& element : elements) {
        auto element_length = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*element).get_rest_length();
        auto element_mass = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*element).get_mass();
        for (auto& node : element->nodes) {
            auto contact_area = diameter * (0.5 * element_length);
            auto penetration_load = seabed.get_penetration_load(*node, contact_area, element_mass);
            node->set_force(node->get_force() + penetration_load);
        }
    }
}

seahowl::Vector3d MooringElastoFEA::get_tension_fairlead() const {
    return fairlead_link->get_reaction_force();
}

seahowl::Vector3d MooringElastoFEA::get_tension_anchor() const {
    return anchor_link->get_reaction_force();
};

double MooringElastoFEA::get_length() const {
    double total_length = 0.0;
    for (auto& element : elements) {
        total_length += dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*element).get_rest_length();
    }
    return total_length;
}
