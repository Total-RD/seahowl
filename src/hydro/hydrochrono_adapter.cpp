#include "seahowl/hydro/hydrochrono_adapter.h"

#include "seahowl/elasto/chrono_adapters.h"

#include <chrono/physics/ChBody.h>

using namespace seahowl::hydro;

FloaterHydroChronoRigid::FloaterHydroChronoRigid() {
    waves = std::make_shared<NoWave>(1);
};

void FloaterHydroChronoRigid::initialize() {
    if (h5_filepath == "") {
        throw std::runtime_error("Path of h5 file for HydroChrono floater was not defined.");
    } else if (floater_body_name == "") {
        throw std::runtime_error("Must provide name of floater for HydroChrono.");
    }
    auto& chbody = dynamic_cast<seahowl::elasto::BodyElastoChrono&>(*floater_body);
    chbody.chobj->SetName(floater_body_name.c_str());

    std::vector<std::shared_ptr<chrono::ChBody>> chbodies;
    chbodies.push_back(chbody.chobj);
    hydrochrono_setter = std::make_unique<TestHydro>(chbodies, h5_filepath);
    hydrochrono_setter->AddWaves(waves);
}

void FloaterHydroChronoRigid::set_name(std::string& name) {
    floater_body_name = name;
}

void FloaterHydroChronoRigid::set_h5_filepath(std::string& h5_filepath) {
    h5_filepath = h5_filepath;
}

FloaterHydroChrono::FloaterHydroChrono() {
    waves = std::make_shared<NoWave>(1);
};

void FloaterHydroChrono::add_fairlead(Vector3d& position) {
    // fairlead
    fairleads.push_back(std::unique_ptr<seahowl::elasto::BodyElastoChrono>());
    auto& fairlead = *(fairleads.back());
    fairlead.set_position(position);

    // link
    links_fairlead_floater.push_back(std::unique_ptr<seahowl::elasto::LinkChrono>());
    auto& link = *(links_fairlead_floater.back().get());
    link.set_constraints(true, true, true, false, true, true);
    if (bodies_map.size() > 0) {
        link.initialize(fairlead, *(bodies_map.begin()->second));
    } else {
        throw std::runtime_error("Need to add body to floater before creating fairleads.");
    }
}

void FloaterHydroChrono::add_body(std::string& name) {
    bodies_map[name] = std::make_unique<seahowl::elasto::BodyElastoChrono>();
}

seahowl::elasto::BodyElasto& FloaterHydroChrono::get_body(std::string& name) {
    return *bodies_map[name];
}

std::vector<std::string> FloaterHydroChrono::get_body_names_list() {
    std::vector<std::string> names;
    for (auto& body : bodies_map) {
        names.push_back(body.first);
    }
    return names;
}

void FloaterHydroChrono::assemble(seahowl::elasto::SystemElasto& system) {
    for (auto& bodymap : bodies_map) {
        auto& body = *bodymap.second;
        system.add(body);
    }
    for (auto& fairlead : fairleads) {
        system.add(*fairlead);
    }
    for (auto& link : links_fairlead_floater) {
        system.add(*link);
    }
}

void FloaterHydroChrono::initialize() {
    if (bodies_map.size() < 1) {
        throw std::runtime_error("List of bodies for HydroChrono floater was not initialized.");
    } else if (h5_filepath == "") {
        throw std::runtime_error("Path of h5 file for HydroChrono floater was not defined.");
    }
    std::vector<std::shared_ptr<chrono::ChBody>> chbodies = {};
    for (auto& body : bodies_map) {
        auto& chbody = dynamic_cast<seahowl::elasto::BodyElastoChrono&>(*body.second);
        chbody.chobj->SetName(body.first.c_str());
        chbodies.push_back(chbody.chobj);
    }
    hydrochrono_setter = std::make_unique<TestHydro>(chbodies, h5_filepath);
    hydrochrono_setter->AddWaves(waves);
}

seahowl::elasto::BodyElasto& FloaterHydroChrono::get_tower_connection_body() {
    auto body_names = get_body_names_list();
    if (body_names.size() < 1) {
        throw std::runtime_error(
            "Trying to connect tower to floater but list of bodies for HydroChrono floater was not initialized.");
    }
    return get_body(body_names[0]);
}

void FloaterHydroChrono::translate(const Vector3d& translation_vector) const {
    // translate all bodies
    for (auto& bodymap : bodies_map) {
        auto& body = *bodymap.second;
        body.set_position(body.get_position() + translation_vector);
    }
}

void FloaterHydroChrono::rotate(double angle, const Vector3d& axis) const {
    // rotate all bodies
    auto rotation = AngleAxisd(angle, axis);
    for (auto& bodymap : bodies_map) {
        auto& body = *bodymap.second;
        auto new_position_body = rotation * body.get_position();
        auto new_rotation_body = (rotation * body.get_rotation()).normalized();
        body.set_position(new_position_body);
        body.set_rotation(new_rotation_body);
    }
}

double FloaterHydroChrono::get_mass() const {
    double total_mass = 0;
    for (auto& bodymap : bodies_map) {
        auto& body = *bodymap.second;
        total_mass += body.get_mass();
    }
    return total_mass;
}
