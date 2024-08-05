#include "seahowl/core/system.h"

#include "seahowl/elasto/system_elasto.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/elasto/mooring_elasto.h"
#include "seahowl/elasto/turbine_elasto.h"
#include "seahowl/aero/turbine_aero.h"
#include "seahowl/env/wind_models.h"
#include "seahowl/core/blade.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/elasto/floater_elasto.h"

#include <vector>
#include <spdlog/spdlog.h>
#include <map>
#include <iostream>

using namespace seahowl::core;

System::System(seahowl::elasto::SystemElasto& elasto, seahowl::aero::SystemAero& aero) : elasto(elasto), aero(aero) {}

void System::build() {
    // build all turbines
    for (auto& turbine : turbines) {
        turbine->build();
    }
    // build all extra components
    for (auto& component : components) {
        component->build();
    }
}

void System::initialize_this(double time, double dt) {
    // assemble elasto system if it hasn't been already
    if (!elasto.is_assembled) {
        elasto.assemble();
    } else {
        spdlog::warn("Elasto system was already assembled, not reassembling.");
    }

    // initialize all turbines
    for (auto& turbine : turbines) {
        turbine->initialize(time, dt);
    }
    // initialize all extra components
    for (auto& component : components) {
        component->initialize(time, dt);
    }

    // check if fluid model exists
    if (!fluid_model) {
        spdlog::warn("No fluid model was attached to the system.");
    }

    // check if soil model exists
    if (!soil_model) {
        spdlog::warn("No soil model was attached to the system.");
    }

    spdlog::info("Initialized system with number of turbines: {}.", turbines.size());
}

void System::prestep(double time, double dt) {
    // apply control first
    for (auto& turbine : turbines) {
        turbine->apply_control(time, dt);
    }

    // compute forces from fluid model
    if (fluid_model) {
        apply_fluid_model(*fluid_model, time);
    }

    // compute forces from soil model
    if (soil_model) {
        apply_soil_model(*soil_model, time);
    }

    // prestep (accumulates loads from aero to elasto)
    for (auto& turbine : turbines) {
        // turbine prestep
        turbine->prestep(time, dt);
    }
    for (auto& component : components) {
        // component poststep
        component->prestep(time, dt);
    }
}

void System::step(double dt) {
    elasto.step(dt);
}

void System::poststep(double time, double dt) {
    for (auto& turbine : turbines) {
        // turbine poststep
        turbine->poststep(time, dt);
    }
    for (auto& component : components) {
        // component poststep
        component->poststep(time, dt);
    }
}

void System::apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time) {
    for (auto& turbine : turbines) {
        // compute forces from fluid model
        turbine->apply_fluid_model(fluid_model, time);
    }
    for (auto& component : components) {
        // compute forces from fluid model
        component->apply_fluid_model(fluid_model, time);
    }
}

void System::apply_soil_model(seahowl::env::SoilModel& soil_model, double time) {
    for (auto& turbine : turbines) {
        // compute forces from fluid model
        turbine->apply_soil_model(soil_model, time);
    }
    for (auto& component : components) {
        // compute forces from fluid model
        component->apply_soil_model(soil_model, time);
    }
}

double System::get_time() const {
    return elasto.get_time();
}

void System::set_time(double time) {
    elasto.set_time(time);
}

void System::run_presetup(double presetup_duration, double presetup_dt) {
    if (presetup_duration == 0.0 || presetup_dt == 0.0 || presetup_dt > presetup_duration) {
        // do nothing if dt or number of steps is zero
        return;
    }
    int nsteps = int(presetup_duration / presetup_dt);
    spdlog::info("Presetup of simulation for {} steps with dt={}.", nsteps, presetup_dt);

    double time_init = get_time();

    std::vector<bool> tower_was_fixed;
    for (int idx_turbine = 0; idx_turbine < turbines.size(); idx_turbine++) {
        auto& turbine = *turbines[idx_turbine];
        tower_was_fixed.push_back(turbine.tower.elasto.nodes.front()->is_fixed());
        turbine.tower.elasto.nodes.front()->set_fixed(true);
        turbine.rna.elasto.rotor->body_hub->set_fixed(true);
    }

    // apply length increments dynamically
    int step_frac = int(nsteps / 20.0);
    std::cout << "|0% -----------> 100%|" << std::endl;
    std::cout << "|";

    for (int step = 0; step <= nsteps; step++) {
        if (step > 0 && step % step_frac == 0) {
            std::cout << "*";
        }
        elasto.presetup(double(step) / double(nsteps));

        prestep(0.0, presetup_dt);

        elasto.step(presetup_dt);

        poststep(0.0, presetup_dt);

        set_time(0.0);
    }

    std::cout << "|" << std::endl;
    // unfix floating turbine
    for (int idx_turbine = 0; idx_turbine < turbines.size(); idx_turbine++) {
        auto& turbine = *turbines[idx_turbine];
        if (tower_was_fixed[idx_turbine]) {
            turbines[idx_turbine]->elasto.tower.nodes.front()->set_fixed(true);
        } else {
            turbines[idx_turbine]->elasto.tower.nodes.front()->set_fixed(false);
        }
        turbine.rna.elasto.rotor->body_hub->set_fixed(false);
    }

    // reset time
    set_time(time_init);

    spdlog::info("Presetup finished.");
}

void System::run_presimulation(double presim_duration, double presim_dt, bool fix_towers) {
    if (presim_duration == 0.0 || presim_dt == 0.0 || presim_dt > presim_duration) {
        // do nothing if duration is zero
        return;
    }
    int nsteps = int(presim_duration / presim_dt);
    spdlog::info("Presimulation for {} steps with dt={}.", nsteps, presim_dt);
    if (get_time() != 0) {
        throw std::runtime_error(
            "Trying to do a presimulation step after simulation started (t=" + std::to_string(get_time()) + ").");
    }
    std::cout << "|0% -----------> 100%|" << std::endl;
    std::cout << "|";

    // fix tower
    std::vector<bool> tower_was_fixed;
    for (auto& turbine : turbines) {
        tower_was_fixed.push_back(turbine->elasto.tower.nodes.front()->is_fixed());
        if (!tower_was_fixed.back() && fix_towers) {
            turbine->elasto.tower.nodes.front()->set_fixed(true);
        }
    }

    int presim_frac = nsteps / 20;
    int istep = 0;
    while (istep < nsteps) {
        istep += 1;
        if (istep % presim_frac == 0) {
            std::cout << "*";
        }
        // prestep
        prestep(0.0, presim_dt);

        // step
        step(presim_dt);

        // poststep
        poststep(0.0, presim_dt);

        set_time(0.0);
    }
    std::cout << "|" << std::endl;
    for (int ii = 0; ii < turbines.size(); ii++) {
        if (tower_was_fixed[ii]) {
            turbines[ii]->elasto.tower.nodes.front()->set_fixed(true);
        } else {
            turbines[ii]->elasto.tower.nodes.front()->set_fixed(false);
        }
    }
    spdlog::info("Presimulation finished.");
}

void System::add(std::shared_ptr<Turbine> turbine) {
    turbines.push_back(turbine);
}

void System::add(std::shared_ptr<ComponentDynamic> component) {
    components.push_back(component);
}
