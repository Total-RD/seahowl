#include <seahowl/elasto/mooring_elasto.h>
#include <seahowl/core/utils.h>
#include <seahowl/elasto/chrono_adapters.h>

#include <chrono/fea/ChContactSurfaceNodeCloud.h>
#include <chrono/physics/ChMaterialSurfaceSMC.h>

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
    build_elements_euler();
}

void MooringElasto::build_elements_euler() {
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
