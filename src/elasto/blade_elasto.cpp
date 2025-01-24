#include "seahowl/elasto/blade_elasto.h"

#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/commons/utils.h"  // For DiscretizationPoint
#include "seahowl/elasto/reference_point_elasto.h"

#include <numeric>
#include <spdlog/spdlog.h>

using namespace seahowl::elasto;

BladeElasto::BladeElasto() {
    // body mount
    body_mount = std::make_unique<BodyElastoChrono>();
    // body root
    body_root = std::make_unique<BodyElastoChrono>();
    // links
    link_root = std::make_unique<LinkChrono>();
    link_root->set_constraints(true, true, true, true, true, true);
    link_root_mount = std::make_unique<LinkChrono>();
    link_root_mount->set_constraints(true, true, true, true, true, true);
    actuator_pitch = std::make_unique<ActuatorRotationChrono>();
    link_blade = std::make_unique<LinkChrono>();
    link_blade->set_constraints(true, true, true, true, true, true);
    //
    discretization_fractions = {0.0, 1.0};
    reset_bodies();
}

void BladeElasto::apply_pitch_increment(double pitch_increment) {
    // apply pitch from root node direction and position
    auto root_dir = body_root->get_rotation() * Vector3d(0.0, 0.0, 1.0);
    auto root_pos = body_root->get_position();
    translate(-root_pos);
    rotate(-pitch_increment, root_dir);
    body_mount->rotate(pitch_increment, root_dir);  // rotate mounting point back
    link_root_mount->initialize(*body_root, *body_mount);
    translate(root_pos);

    // update actuator pitch timeseries in case it is used
    if (has_pitch_actuator_dynamics) {
        if (is_assembled) {
            spdlog::warn(
                "Blade pitch has been incremented instantaneously by {} degrees, resetting dynamic pitch actuator to "
                "current pitch value ({} degrees) as a constant over time.",
                pitch_increment * 180 / seahowl::PI, get_pitch() * 180 / seahowl::PI);
        }
        actuator_pitch->set_timeseries(std::vector<double>{0.0, 0.0}, std::vector<double>{get_pitch(), get_pitch()});
    }
}

double BladeElasto::get_pitch() const {
    // get angle between quaternions
    auto qq = (body_root->get_rotation().conjugate() * body_mount->get_rotation()).normalized();
    double angle0 = 2 * std::atan2(qq.vec().z(), qq.w());
    // get angle between 0 and 2pi
    double angle1 = fmod(angle0, 2 * PI);
    return angle1;
}

void BladeElasto::attach_blade_to_body(const BodyElasto& body) {
    is_mounted = true;
    link_blade->initialize(*body_mount, body);
}

void BladeElasto::assemble_this(SystemElasto& system) {
    system.add(*(body_root.get()));
    system.add(*(link_root.get()));
    system.add(*(body_mount.get()));
    if (has_pitch_actuator_dynamics) {
        system.add(*(actuator_pitch.get()));
    } else {
        system.add(*(link_root_mount));
    }
    if (is_mounted) {
        system.add(*(link_blade.get()));
    }
}

void BladeElasto::rotate(double angle, const Vector3d& axis) const {
    // blade root
    body_root->rotate(angle, axis);
    body_mount->rotate(angle, axis);
}

void BladeElasto::translate(const Vector3d& translation_vector) const {
    // blade root
    body_root->translate(translation_vector);
    body_mount->translate(translation_vector);
}

double BladeElasto::get_mass() const {
    // adding body_root for consistency even if mass is supposed to be zero.
    return body_root->get_mass() + body_mount->get_mass();
}

void BladeElasto::reset_bodies() {
    // body mount
    body_mount->set_position(Vector3d(0.0, 0.0, 0.0));
    body_mount->set_rotation(Quaternion(1.0, 0.0, 0.0, 0.0));
    body_mount->set_mass(0.0);
    body_mount->set_inertia_diagonal(Vector3d(0.0, 0.0, 0.0));
    // body root
    body_root->set_position(Vector3d(0.0, 0.0, 0.0));
    body_root->set_rotation(Quaternion(1.0, 0.0, 0.0, 0.0));
    body_root->set_mass(0.0);
    body_root->set_inertia_diagonal(Vector3d(0.0, 0.0, 0.0));
}

seahowl::Vector3d BladeElasto::get_blade_root_moment() const {
    return link_root->get_reaction_torque() + body_root->get_torque(true);
}

seahowl::Vector3d BladeElasto::get_blade_root_force() const {
    return link_root->get_reaction_force() + body_root->get_force(true);
}

void BladeElasto::update_root_constraint() {
    // attach COG body to root body
    link_root_mount->initialize(*body_root, *body_mount);
    // attach root body to mounting point
    actuator_pitch->initialize(*body_root, *body_mount, Vector3d(-1.0, 0.0, 0.0));
    // set initial pitch
    actuator_pitch->set_timeseries(std::vector<double>{0.0, 0.0}, std::vector<double>{get_pitch(), get_pitch()});
}

BladeElastoFEA::BladeElastoFEA() {}

void BladeElastoFEA::assemble_this(SystemElasto& system) {
    BladeElasto::assemble_this(system);
    ComponentElastoFEA::assemble_this(system);
}

void BladeElastoFEA::rotate(double angle, const Vector3d& axis) const {
    // blade root
    BladeElasto::rotate(angle, axis);
    // blade nodes
    ComponentElastoFEA::rotate(angle, axis);
}

void BladeElastoFEA::translate(const Vector3d& translation_vector) const {
    // blade root
    BladeElasto::translate(translation_vector);
    // blade nodes
    ComponentElastoFEA::translate(translation_vector);
}

double BladeElastoFEA::get_mass() const {
    // adding body_root for consistency even if mass is supposed to be zero.
    return ComponentElastoFEA::get_mass() + BladeElasto::get_mass();
}

void BladeElastoFEA::presetup(double fraction) {
    for (int ii = 0; ii < nodes.size(); ii++) {
        BladeReferencePointElasto ref = discretized_points[ii];
        for (int jj = 0; jj < ref.damping_coefficients.size(); jj++) {
            ref.damping_coefficients[jj] = (1.0 - fraction) + ref.damping_coefficients[jj] * fraction;
        }
        auto node = std::dynamic_pointer_cast<NodeElastoChrono>(nodes[ii]);
        node->set_properties(ref);
    }
    for (auto& element : elements) {
        std::dynamic_pointer_cast<ElementElastoChrono>(element)->update_properties();
    }
    spdlog::debug("Presetup for blade at fraction {} (relaxing damping to target value gradually).", fraction);
}

void BladeElastoFEA::build() {
    reset_bodies();

    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough elasto reference points defined for blade (" +
                                 std::to_string(reference_points.size()) + ").");
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

    // attach node to root body
    link_root->initialize(*nodes[0], *body_root);

    // apply initial pitch
    apply_pitch_increment(pitch0);
    update_root_constraint();
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

seahowl::EntityDynamicEigen BladeElastoFEA::get_entity_along_blade(double eta, int element_index) const {
    return get_entity_along_component_slerp(eta, element_index);
}

void BladeElastoFEA::accumulate_load_along_blade(const seahowl::Vector3d& load,
                                                 const seahowl::Vector3d& moment,
                                                 int element_index,
                                                 double eta,
                                                 const seahowl::Vector3d& offset) {
    accumulate_element_load(load, moment, element_index, eta, offset);
}

BladeElastoRigid::BladeElastoRigid() {}

void BladeElastoRigid::build() {
    reset_bodies();

    body_cog = std::make_unique<BodyElastoChrono>();
    length = reference_points.back().coordinates.z();

    // calculate mass
    double mass_total = 0.0;
    double inertia_total = 0.0;
    double z_pos = 0.0;
    for (int ii = 0; ii < reference_points.size() - 1; ii++) {
        auto& point1 = reference_points[ii];
        auto& point2 = reference_points[ii + 1];
        auto rho1 = point1.mass_matrix(2, 2);
        auto rho2 = point2.mass_matrix(2, 2);
        auto l1 = point1.coordinates.z();
        auto l2 = point2.coordinates.z();
        auto length_segment = abs(l2 - l1);

        // mass of blade segment
        auto mass_segment = 0.5 * (rho1 + rho2) * length_segment;
        mass_total += mass_segment;
        z_pos += 0.5 * (l1 + l2) * mass_segment;

        // inertia assuming rod element with non-uniform linear density
        inertia_total += ((pow(l2, 4) - pow(l1, 4)) / 4.0 * (rho2 - rho1) +
                          (pow(l2, 3) - pow(l1, 3)) / 3.0 * (rho1 * l2 - rho2 * l1)) /
                         length_segment;
    };
    z_pos /= mass_total;

    // set mass and inertia at root
    body_cog->set_position(Vector3d(0., 0., z_pos));
    body_cog->set_mass(mass_total);
    body_cog->set_inertia_diagonal(Vector3d(0., 0., 0.));
    body_root->set_inertia_diagonal(Vector3d(inertia_total, inertia_total, 0.0));

    // attach COG body to root body
    link_root->initialize(*body_cog, *body_root);

    // apply initial pitch
    apply_pitch_increment(pitch0);
    update_root_constraint();
}

void BladeElastoRigid::assemble_this(SystemElasto& system) {
    BladeElasto::assemble_this(system);
    system.add(*(body_cog.get()));
}

void BladeElastoRigid::rotate(double angle, const Vector3d& axis) const {
    // blade root
    BladeElasto::rotate(angle, axis);
    // blade cog
    body_cog->rotate(angle, axis);
}

void BladeElastoRigid::translate(const Vector3d& translation_vector) const {
    // blade root
    BladeElasto::translate(translation_vector);
    // blade cog
    body_cog->translate(translation_vector);
}

double BladeElastoRigid::get_mass() const {
    return BladeElasto::get_mass() + body_cog->get_mass();
}

seahowl::EntityDynamicEigen BladeElastoRigid::get_entity_along_blade(double eta, int element_index) const {
    auto entity = seahowl::EntityDynamicEigen();

    // relative position along z axis of blade
    auto z_position = (0.5 * fabs(eta + 1.0)) * length;

    // position
    Vector3d position = body_root->get_position() + body_root->get_rotation() * Vector3d(0.0, 0.0, z_position);
    entity.set_position(position);

    // get structural twist
    std::vector<double> eta_vector = {0.5 * (eta + 1.0)};
    auto twist = seahowl::get_discretized_points(eta_vector, reference_points)[0].structural_twist;

    // rotation
    auto twist_matrix = AngleAxisd(-twist, body_root->get_rotation() * Vector3d(0.0, 0.0, 1.0));
    auto rotation_matrix = twist_matrix * body_root->get_rotation().toRotationMatrix();
    entity.set_rotation(Quaternion(rotation_matrix));

    // below has to be explicitly declared as Vector3d or there is an issue;
    Vector3d pos1 = entity.get_position();
    Vector3d radius_vector = (pos1 - body_root->get_position());

    // velocity
    Vector3d velocity = body_root->get_velocity() + (body_root->get_rotational_velocity(false)).cross(radius_vector);
    entity.set_velocity(velocity);
    entity.set_rotational_velocity(body_root->get_rotational_velocity());

    // acceleration
    Vector3d acceleration =
        body_root->get_acceleration() + (body_root->get_rotational_acceleration(false)).cross(radius_vector);
    entity.set_acceleration(acceleration);
    entity.set_rotational_acceleration(body_root->get_rotational_acceleration());

    return entity;
}

void BladeElastoRigid::reset_loads() {
    body_root->reset_loads();
    body_cog->reset_loads();
}

void BladeElastoRigid::accumulate_load_along_blade(const seahowl::Vector3d& load,
                                                   const seahowl::Vector3d& moment,
                                                   int element_index,
                                                   double eta,
                                                   const seahowl::Vector3d& offset) {
    // apply force on root
    body_root->accumulate_force(load, false);

    // apply moment on root
    auto entity = get_entity_along_blade(eta, element_index);
    auto distance = (entity.get_position() + offset - body_root->get_position());
    body_root->accumulate_torque(moment + distance.cross(load), false);
}
