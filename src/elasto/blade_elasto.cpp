#include "seahowl/elasto/blade_elasto.h"

#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/commons/utils.h"  // For DiscretizationPoint
#include "seahowl/elasto/reference_point_elasto.h"

#include <numeric>

using namespace seahowl::elasto;

BladeElasto::BladeElasto() {
    discretization_fractions = {0.0, 1.0};
    link_root = std::make_unique<LinkChrono>();
    link_root->set_constraints(true, true, true, true, true, true);
}

BladeElastoFEA::BladeElastoFEA() {}

void BladeElastoFEA::assemble(SystemElasto& system) const {
    system.add(*(link_root.get()));
    ComponentElastoFEA::assemble(system);
}

void BladeElastoFEA::build() {
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
    discretized_points = seahowl::get_discretized_points(discretization_fractions, reference_points);
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
};

void BladeElastoFEA::build_elements_tapered_timoshenko() {
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
        element->set_prebend(rotation_relative);
    }
}

void BladeElastoFEA::build_elements_tapered_timoshenko_fpm() {
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
        element->set_prebend(rotation_relative);
    }
}

void BladeElastoFEA::evaluate_position_rotation(Vector3d& position,
                                                Quaternion& rotation,
                                                int element_index,
                                                double eta) const {
    elements[element_index]->evaluate_position_rotation(eta, position, rotation);
}

void BladeElastoFEA::apply_pitch_increment(double pitch_increment) {
    // apply pitch from root node direction and position
    auto root_dir = nodes.front()->get_direction();
    auto root_pos = nodes.front()->get_position();
    translate(-root_pos);
    rotate(-pitch_increment, root_dir);
    translate(root_pos);
    pitch += pitch_increment;
}

seahowl::Vector3d BladeElastoFEA::get_blade_root_moment() const {
    return elements[0]->get_torque(-1.0);
}

seahowl::EntityDynamicEigen BladeElastoFEA::get_entity_along_blade(double eta, int element_index) const {
    auto entity = seahowl::EntityDynamicEigen();
    Vector3d new_position;
    Quaternion new_rotation;
    evaluate_position_rotation(new_position, new_rotation, element_index, eta);
    entity.set_position(new_position);
    entity.set_rotation(new_rotation);

    // update properties of aero nodes
    double weight1 = 0.5 * fabs(eta - 1.0);
    double weight2 = 0.5 * fabs(eta + 1.0);
    auto& element = elements[element_index];
    auto& node1 = element->nodes[0];
    auto& node2 = element->nodes[1];
    entity.set_velocity(weight1 * node1->get_velocity() + weight2 * node2->get_velocity());
    entity.set_rotational_velocity(weight1 * node1->get_rotational_velocity() +
                                   weight2 * node2->get_rotational_velocity());
    entity.set_acceleration(weight1 * node1->get_acceleration() + weight2 * node2->get_acceleration());
    entity.set_rotational_acceleration(weight1 * node1->get_rotational_acceleration() +
                                       weight2 * node2->get_rotational_acceleration());

    return entity;
}

void BladeElastoFEA::accumulate_load_along_blade(const seahowl::Vector3d& load,
                                                 int element_index,
                                                 double eta,
                                                 const seahowl::Vector3d& offset) {
    accumulate_element_load(load, element_index, eta, offset);
}

void BladeElastoFEA::attach_root_to_body(const BodyElasto& body) {
    link_root->initialize(*(nodes.front().get()), body);
};

BladeElastoRigid::BladeElastoRigid() {}

void BladeElastoRigid::build() {
    body_root = std::make_unique<BodyElastoChrono>();
    // body_root->set_mass(0.0);
    length = reference_points.back().coordinates.z();
}

void BladeElastoRigid::assemble(SystemElasto& system) const {
    system.add(*(link_root.get()));
    system.add(*(body_root.get()));
}

void BladeElastoRigid::rotate(double angle, const Vector3d& axis) const {
    auto rotation = AngleAxisd(angle, axis);
    // blade root
    auto new_position_root = rotation * body_root->get_position();
    auto new_rotation_root = (rotation * body_root->get_rotation()).normalized();
    body_root->set_position(new_position_root);
    body_root->set_rotation(new_rotation_root);
}

void BladeElastoRigid::translate(const Vector3d& translation_vector) const {
    // blade root
    body_root->set_position(body_root->get_position() + translation_vector);
}

double BladeElastoRigid::get_mass() const {
    return mass;
}

void BladeElastoRigid::apply_pitch_increment(double pitch_increment) {
    // update rotation around local Z-axis
    auto root_dir = body_root->get_rotation() * Vector3d(0.0, 0.0, 1.0);
    auto root_pos = body_root->get_position();
    translate(-root_pos);
    rotate(-pitch_increment, root_dir);
    translate(root_pos);
    pitch += pitch_increment;
}

seahowl::Vector3d BladeElastoRigid::get_blade_root_moment() const {
    return body_root->get_torque();
}

seahowl::EntityDynamicEigen BladeElastoRigid::get_entity_along_blade(double eta, int element_index) const {
    auto entity = seahowl::EntityDynamicEigen();

    // relative position along z axis of blade
    auto z_position = (0.5 * fabs(eta + 1.0)) * length;

    // position
    auto position = body_root->get_position() + body_root->get_rotation() * Vector3d(0.0, 0.0, z_position);
    entity.set_position(position);
    entity.set_rotation(body_root->get_rotation());

    // below has to be explicitly declared as Vector3d or there is an issue;
    Vector3d radius_vector = (entity.get_position() - body_root->get_position());

    // velocity
    auto velocity = body_root->get_velocity() + (body_root->get_rotational_velocity(false)).cross(radius_vector);
    entity.set_velocity(velocity);
    entity.set_rotational_velocity(body_root->get_rotational_velocity());

    // acceleration
    auto acceleration =
        body_root->get_acceleration() + (body_root->get_rotational_acceleration(false)).cross(radius_vector);
    entity.set_acceleration(acceleration);
    entity.set_rotational_acceleration(body_root->get_rotational_acceleration());

    return entity;
}

void BladeElastoRigid::reset_loads() {
    body_root->reset_loads();
}

void BladeElastoRigid::accumulate_load_along_blade(const seahowl::Vector3d& load,
                                                   int element_index,
                                                   double eta,
                                                   const seahowl::Vector3d& offset) {
    // apply force on root
    body_root->accumulate_force(load, false);

    // apply moment on root
    auto entity = get_entity_along_blade(eta, element_index);
    auto distance = (entity.get_position() + offset - body_root->get_position());
    auto moment = distance.cross(load);
    body_root->accumulate_torque(moment, false);
}

void BladeElastoRigid::attach_root_to_body(const BodyElasto& body) {
    link_root->initialize(*body_root, body);
};
