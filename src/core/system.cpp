#include "seahowl/core/system.h"

#include "seahowl/elasto/system_elasto.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/elasto/mooring_elasto.h"
#include "seahowl/elasto/turbine_elasto.h"
#include "seahowl/aero/turbine_aero.h"
#include "seahowl/env/wind_models.h"
#include "seahowl/core/turbine_floating.h"
#include "seahowl/core/blade.h"
#include "seahowl/elasto/blade_elasto.h"

#include <vector>
#include <spdlog/spdlog.h>
#include <map>
#include <iostream>

using namespace seahowl::core;

System::System(seahowl::elasto::SystemElasto& elasto, seahowl::aero::SystemAero& aero) : elasto(elasto), aero(aero) {}

void System::initialize_this(double time, double dt) {
    // assemble elasto system if it hasn't been already
    if (!elasto.is_assembled) {
        elasto.assemble();
    }

    // initialize all turbines
    for (auto& turbine : turbines) {
        turbine->initialize(time, dt);
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
    for (auto& turbine : turbines) {
        turbine->apply_control(time, dt);

        // compute forces from fluid model
        if (fluid_model) {
            turbine->apply_fluid_model(*fluid_model, time);
        }
        // compute forces from soil model
        if (soil_model) {
            turbine->apply_soil_model(*soil_model, time);
        }
        // turbine prestep (accumulates loads from aero to elasto)
        turbine->prestep(time, dt);
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

    std::map<int, std::vector<std::vector<double>>> lengths_initial_moorings;
    std::map<int, std::vector<std::vector<double>>> lengths_final_moorings;
    std::map<int, std::vector<std::vector<double>>> lengths_delta_moorings;
    for (int idx_turbine = 0; idx_turbine < turbines.size(); idx_turbine++) {
        auto& turbine = *turbines[idx_turbine];
        try {
            auto& turbine_floating = dynamic_cast<TurbineFloating&>(turbine);

            // fix turbine
            turbine_floating.tower.elasto.nodes.front()->set_fixed(true);
            turbine_floating.rna.elasto.rotor->body_hub->set_fixed(true);

            for (auto& mooring_ptr : turbine_floating.elasto.mooring_system->moorings) {
                auto& mooring_elasto = *mooring_ptr;
                auto& mooring = dynamic_cast<seahowl::elasto::MooringElastoFEA&>(mooring_elasto);

                // check that mooring was built properly
                int nb_elements = mooring.elements.size();
                if (nb_elements != mooring.discretization_fractions.size() - 1) {
                    throw std::runtime_error(
                        "Number of elements and discretization fractions on mooring do not match (build mooring "
                        "first?).");
                }

                // get initial and final lengths of mooring, and length increment to apply
                std::vector<double> lengths_initial(nb_elements);
                std::vector<double> lengths_final(nb_elements);
                std::vector<double> lengths_delta(nb_elements);

                auto distance_fairlead_anchor =
                    (mooring.fairlead.get_position() - mooring.anchor.get_position()).norm();
                auto density_equivalent = mooring.density_linear * (mooring.length / distance_fairlead_anchor) /
                                          (seahowl::PI * pow(mooring.diameter / 2.0, 2));
                for (size_t idx_el = 0; idx_el < nb_elements; idx_el++) {
                    auto& element = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*mooring.elements[idx_el]);
                    lengths_final[idx_el] = mooring.length * (mooring.discretization_fractions[idx_el + 1] -
                                                              mooring.discretization_fractions[idx_el]);
                    lengths_initial[idx_el] =
                        (element.nodes[1]->get_position() - element.nodes[0]->get_position()).norm();
                    lengths_delta[idx_el] = (lengths_final[idx_el] - lengths_initial[idx_el]) / nsteps;
                    // initializing element with density-equivalent length
                    element.set_properties(density_equivalent, mooring.diameter, mooring.stiffness_axial,
                                           mooring.stiffness_bending);
                }

                lengths_initial_moorings[idx_turbine].push_back(lengths_initial);
                lengths_final_moorings[idx_turbine].push_back(lengths_final);
                lengths_delta_moorings[idx_turbine].push_back(lengths_delta);
            }
        } catch (const std::bad_cast& e) {
            // do nothing
        }
    }

    // apply length increments dynamically
    int step_frac = int(nsteps / 20.0);
    std::cout << "|0% -----------> 100%|" << std::endl;
    std::cout << "|";
    for (int step = 0; step <= nsteps; step++) {
        if (step > 0 && step % step_frac == 0) {
            std::cout << "*";
        }
        for (int idx_turbine = 0; idx_turbine < turbines.size(); idx_turbine++) {
            auto& turbine = *turbines[idx_turbine];
            try {
                auto& turbine_floating = dynamic_cast<TurbineFloating&>(turbine);
                for (int idx_mooring = 0; idx_mooring < turbine_floating.elasto.mooring_system->moorings.size();
                     idx_mooring++) {
                    auto& mooring_elasto = *turbine_floating.elasto.mooring_system->moorings[idx_mooring];
                    auto& mooring = dynamic_cast<seahowl::elasto::MooringElastoFEA&>(mooring_elasto);
                    auto lengths_initial = lengths_initial_moorings[idx_turbine][idx_mooring];
                    auto lengths_delta = lengths_delta_moorings[idx_turbine][idx_mooring];
                    int nb_elements = mooring.elements.size();
                    for (int idx_el = 0; idx_el < nb_elements; idx_el++) {
                        auto& element = dynamic_cast<seahowl::elasto::ElementMooringElasto&>(*mooring.elements[idx_el]);
                        element.set_rest_length(lengths_initial[idx_el] + lengths_delta[idx_el] * step);
                        if (step == 1) {
                            // reset to actual properties after initializing with density-equivalent length
                            element.set_properties(
                                mooring.density_linear / (seahowl::PI * pow(mooring.diameter / 2.0, 2)),
                                mooring.diameter, mooring.stiffness_axial, mooring.stiffness_bending);
                        }
                    }
                    mooring.compute_hydro_loads(*fluid_model, 0.0);

                    if (soil_model) {
                        mooring.compute_seabed_loads(*soil_model);
                    }
                }
            } catch (const std::bad_cast& e) {
                // do nothing
            }
        }

        elasto.step(presetup_dt);
    }

    std::cout << "|" << std::endl;
    // unfix floating turbine
    for (int idx_turbine = 0; idx_turbine < turbines.size(); idx_turbine++) {
        auto& turbine = *turbines[idx_turbine];
        try {
            auto& turbine_floating = dynamic_cast<TurbineFloating&>(turbine);
            turbine_floating.tower.elasto.nodes.front()->set_fixed(false);
            turbine_floating.rna.elasto.rotor->body_hub->set_fixed(false);
        } catch (const std::bad_cast& e) {
            // do nothing
        }
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

void System::add_turbine(std::shared_ptr<Turbine> turbine) {
    turbines.push_back(turbine);
}
