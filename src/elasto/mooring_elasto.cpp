#include "seahowl/elasto/mooring_elasto.h"

#include "seahowl/commons/numerics.h"
#include "seahowl/commons/utils.h"
#include "seahowl/elasto/reference_point_elasto.h"
#include "seahowl/elasto/chrono_adapters.h"

#include <spdlog/spdlog.h>

using namespace seahowl::elasto;

MooringSystemElasto::MooringSystemElasto() {}

void MooringSystemElasto::add_mooring(std::shared_ptr<MooringElasto> mooring) {
    if (std::find(moorings.begin(), moorings.end(), mooring) == moorings.end()) {
        moorings.push_back(mooring);
    } else {
        spdlog::warn("Mooring Elasto already exists in the system, not adding again.");
    }
}

void MooringSystemElasto::presetup(double fraction) {
    for (auto& mooring : moorings) {
        mooring->presetup(fraction);
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

void MooringElastoFEA::set_length(double length) {
    this->length = length;
    for (int ii = 0; ii < elements.size(); ii++) {
        dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*elements[ii])
            .set_rest_length(length * abs(discretization_fractions[ii + 1] - discretization_fractions[ii]));
    }
}

void MooringElastoFEA::set_diameter(double diameter) {
    this->diameter = diameter;
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

void MooringElastoFEA::presetup(double fraction) {
    int nb_elements = elements.size();
    auto mooring_length_current = get_length();
    if (fraction > 0.0 && fraction <= 1.0) {
        if (length0 < 0.0) {
            throw std::runtime_error("Initial length of mooring was not set, cannot perform presetup.");
        }
        for (int idx_el = 0; idx_el < nb_elements; idx_el++) {
            auto& element = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*elements[idx_el]);
            auto element_length = element.get_rest_length();
            auto element_fraction_range = abs(discretization_fractions[idx_el + 1] - discretization_fractions[idx_el]);

            element.set_rest_length(length0 / nb_elements + (length - length0) * element_fraction_range * fraction);
            element.set_properties(density_linear / (seahowl::PI * pow(diameter / 2.0, 2)), diameter, stiffness_axial,
                                   stiffness_bending);
        }
        spdlog::debug(
            "Presetup for mooring at fraction {}: length of {} (target length: {}), mass of {} (target mass: {}).",
            fraction, get_length(), length, get_mass(), density_linear * length);
    } else if (fraction > 1.0) {
        throw std::runtime_error("Presetup of mooring: cannot have a fraction of " + std::to_string(fraction) + ".");
    }
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

    // section properties
    auto area = PI * pow(diameter, 2) / 4.0;
    auto density = density_linear / area;

    // set length-equivalent properties at setup
    // necessary for Chrono cable element as changing linear density before the first time step has no effect
    // @todo see if there is a way to get rid of this density-equivalent requirement on the Chrono side.
    length0 = (fairlead.get_position() - anchor.get_position()).norm();
    auto density_equivalent = density * (length / length0);

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = std::make_shared<ElementMooringElastoChrono>();
        // add element to elements vector
        elements.push_back(element);
        // set element nodes
        element->set_nodes(nodes[ii - 1], nodes[ii]);

        element->set_properties(density_equivalent, diameter, stiffness_axial, stiffness_bending);
    }
}

void MooringElastoFEA::assemble_this(SystemElasto& system) {
    ComponentElastoFEA::assemble_this(system);
    system.add(*fairlead_link);
    system.add(*anchor_link);
    gravitational_acceleration = system.get_gravitational_acceleration();
}

void MooringElastoFEA::compute_seabed_loads(const seahowl::env::EnvModel& env_model) {
    for (auto& element : elements) {
        auto element_length = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*element).get_rest_length();
        auto element_mass = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*element).get_mass();
        for (auto& node : element->nodes) {
            auto contact_area = diameter * (0.5 * element_length);
            auto penetration_load = env_model.soil_models.get_penetration_load(*node, contact_area, element_mass);
            node->accumulate_force_internals(penetration_load);
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
