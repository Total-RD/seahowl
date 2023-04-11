#include "seahowl/hydro/floater_hydro.h"

#include "seahowl/elasto/chrono_adapters.h"

#include <chrono/physics/ChBody.h>

void FloaterHydroChrono::set_h5filepath(const std::string& filepath) {
    filepath_h5_potential = filepath;
};

void FloaterHydroChrono::add_body(std::unique_ptr<seahowl::elasto::BodyElasto>&& body, std::string& name) {
    bodies_map[name] = std::move(body);
}

seahowl::elasto::BodyElasto& FloaterHydroChrono::get_body(std::string& name) {
    return *bodies_map[name];
}

void FloaterHydroChrono::initialize() {
    if (bodies_map.size() < 1) {
        throw std::runtime_error("List of bodies for floater was not initialized.");
    } else if (filepath_h5_potential == "") {
        throw std::runtime_error("Path of h5 file for floater was not defined.");
    }
    std::vector<std::shared_ptr<chrono::ChBody>> chbodies = {};
    for (auto& body : bodies_map) {
        try {
            auto chbody = dynamic_cast<seahowl::elasto::BodyElastoChrono&>(*body.second);
            chbody.chobj->SetName(body.first.c_str());
            chbodies.push_back(chbody.chobj);
        } catch (const std::bad_cast& e) {
            throw std::runtime_error("Body passed to floater is not a Chrono Body");
        }
    }
    hydrochrono_setter = TestHydro(chbodies, filepath_h5_potential, hydro_inputs);
}
