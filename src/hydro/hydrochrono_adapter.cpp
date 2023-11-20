#include "seahowl/hydro/hydrochrono_adapter.h"

#include "seahowl/elasto/chrono_adapters.h"

#include <chrono/physics/ChBody.h>

#include <spdlog/spdlog.h>

using namespace seahowl::hydro;
using namespace seahowl::env;

FloaterHydroChrono::FloaterHydroChrono(){};

void FloaterHydroChrono::initialize() {
    if (h5_filepath == "") {
        throw std::runtime_error("Path of h5 file for HydroChrono floater was not defined.");
    } else if (floater_bodies.size() == 0) {
        throw std::runtime_error("Must add bodies to floater before initializing.");
    }

    if (!waves) {
        throw std::runtime_error("Need to attach waves to floater before initializing when using HydroChrono.");
    }

    // give names to Chrono bodies
    std::vector<std::shared_ptr<chrono::ChBody>> chbodies;
    for (auto& bodymap : floater_bodies) {
        auto& chbody = dynamic_cast<seahowl::elasto::BodyElastoChrono&>(*bodymap.second);
        chbody.chobj->SetName(bodymap.first.c_str());
        chbodies.push_back(chbody.chobj);
    }

    // send ChBody list to HydroChrono
    hydrochrono_setter = std::make_unique<TestHydro>(chbodies, h5_filepath);
    hydrochrono_setter->AddWaves(waves);
}

void FloaterHydroChrono::set_h5_filepath(std::string filepath) {
    h5_filepath = filepath;
}

void FloaterHydroChrono::set_waves(std::shared_ptr<WaveBase> waves) {
    this->waves = waves;
}

WaveModelHydroChrono::WaveModelHydroChrono() {
    waves = std::make_shared<NoWave>();
}

seahowl::Vector3d WaveModelHydroChrono::get_fluid_velocity(const Vector3d& position, double time) const {
    return Vector3d(0.0, 0.0, 0.0);
}

double WaveModelHydroChrono::get_fluid_density(const Vector3d& position, double time) const {
    return density;
}

bool WaveModelHydroChrono::is_in_water(const Vector3d& position, double time) const {
    return false;
}
