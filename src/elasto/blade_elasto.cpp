#include <seahowl/elasto/blade_elasto.h>

#include <seahowl/elasto/chrono_adapters.h>
#include <seahowl/core/utils.h>  // For DiscretizationPoint
#include <seahowl/elasto/reference_point_elasto.h>

#include <numeric>

using namespace seahowl::elasto;

BladeElasto::BladeElasto() {}

void BladeElasto::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() <= 2) {
        throw std::runtime_error("Not enough elasto reference points defined for blade.");
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
    ///@todo find better way to build ReferencePointElasto from BladeReferencePointElasto
    std::vector<ReferencePointElasto> discretized_points0;
    for (int ii = 0; ii < discretized_points.size(); ii++) {
        auto discretized_point0 = ReferencePointElasto();
        discretized_point0.coordinates = discretized_points[ii].coordinates;
        discretized_point0.fraction = discretized_points[ii].fraction;
        discretized_points0.push_back(discretized_point0);
    }
    // build nodes
    build_nodes(discretized_points0);
    // apply properties
    for (int ii = 0; ii < nodes.size(); ii++) {
        std::dynamic_pointer_cast<NodeElastoChrono>(nodes[ii])->set_properties(discretized_points[ii], fpm_mode);
    }
    // apply structural twist
    for (int ii = 0; ii < nodes.size(); ii++) {
        auto& node = nodes[ii];
        auto& point = discretized_points[ii];
        auto axis = node->get_direction();
        auto twist_matrix = AngleAxisd(-point.structural_twist, axis);
        auto rotation_matrix = twist_matrix * nodes[ii]->get_rotation().toRotationMatrix();
        nodes[ii]->set_rotation(Quaternion(rotation_matrix));
    }

    if (fpm_mode) {
        build_elements_tapered_timoshenko_fpm();
    } else {
        build_elements_tapered_timoshenko();
    }
    // commented out since loads are applied to nodes;
    // build_loads(system);
};

void BladeElasto::build_elements_tapered_timoshenko() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    if (nelements <= 0) {
        throw std::runtime_error("Trying to build blade with no element.");
    }

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = std::make_shared<ElementBladeElastoChrono>();
        // add element to blade elements vector
        elements.push_back(element);
        // set element nodes
        element->set_nodes(nodes[ii - 1], nodes[ii]);

        // apply prebend and structural twist
        auto rotation_relative = (nodes[ii]->get_rotation() * nodes[ii - 1]->get_rotation().inverse()).normalized();
        // switch from IEC standard (Z along blade) to chrono element coordinate system (X along element)
        rotation_relative =
            Quaternion(rotation_relative.w(), rotation_relative.z(), rotation_relative.y(), rotation_relative.x());
        element->set_prebend(rotation_relative);
    }
}

void BladeElasto::build_elements_tapered_timoshenko_fpm() {
    elements.clear();
    const auto nelements = nodes.size() - 1;

    if (nelements <= 0) {
        throw std::runtime_error("Trying to build blade with no element.");
    }

    for (size_t ii = 1; ii < nelements + 1; ii++) {
        // create element
        auto element = std::make_shared<ElementBladeElastoChronoFPM>();
        // add element to blade elements vector
        elements.push_back(element);
        // set element nodes
        element->set_nodes(nodes[ii - 1], nodes[ii]);

        // apply prebend and structural twist
        auto rotation_relative = (nodes[ii]->get_rotation() * nodes[ii - 1]->get_rotation().inverse()).normalized();
        // switch from IEC standard (Z along blade) to chrono element coordinate system (X along element)
        rotation_relative =
            Quaternion(rotation_relative.w(), rotation_relative.z(), rotation_relative.y(), rotation_relative.x());
        element->set_prebend(rotation_relative);
    }
}

// void BladeElasto::build_loads(chrono::ChSystemSMC& system) {
//    auto loadcontainer = chrono_types::make_shared<chrono::ChLoadContainer>();
//    system.Add(loadcontainer);
//
//    for (auto element : elements) {
//        std::shared_ptr<chrono::ChLoad<ChLoaderWeighted>> loader_weighted(
//            new chrono::ChLoad<ChLoaderWeighted>(element));
//        loaders_aero.push_back(loader_weighted);
//        loadcontainer->Add(loader_weighted);
//    }
//}

void BladeElasto::evaluate_position_rotation(Vector3d& position,
                                             Quaternion& rotation,
                                             int element_index,
                                             double eta) const {
    auto element = std::dynamic_pointer_cast<ElementBladeElastoChrono>(elements[element_index]);

    // // unfortunately line below does not always work (returns nans sometimes when fpm_mode is true)
    // // @todo fix this (Chrono issue ?)
    // element->evaluate_position_rotation(eta, position, rotation);
    auto w1 = std::abs(eta - 1.0) * 0.5;
    auto w2 = std::abs(eta + 1.0) * 0.5;
    position = w1 * element->nodes0[0]->get_position() + w2 * element->nodes0[1]->get_position();
    rotation = element->nodes0[0]->get_rotation();
}

void BladeElasto::apply_pitch_increment(double pitch_increment) {
    // apply pitch from root node direction and position
    auto root_dir = nodes.front()->get_direction();
    auto root_pos = nodes.front()->get_position();
    translate(-root_pos);
    rotate(-pitch_increment, root_dir);
    translate(root_pos);
    pitch += pitch_increment;
};
