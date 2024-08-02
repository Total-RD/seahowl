#include "seahowl/core/mooring.h"

#include "seahowl/commons/utils.h"
#include "seahowl/elasto/mooring_elasto.h"
#include "seahowl/hydro/mooring_hydro.h"

#include <spdlog/spdlog.h>

using namespace seahowl::core;
using namespace seahowl::elasto;
using namespace seahowl::hydro;

Mooring::Mooring(MooringElastoFEA& elasto, MooringHydro& hydro) : elasto(elasto), hydro(hydro) {}

void Mooring::set_length(double length) {
    elasto.set_length(length);
    hydro.set_length(length);
}

void Mooring::set_diameter(double diameter) {
    elasto.set_diameter(diameter);
    hydro.set_diameter(diameter);
}

void Mooring::perform_sanity_check() {
    if (elasto.length != hydro.length) {
        throw std::runtime_error("Mooring has different lengths: " + std::to_string(elasto.length) +
                                 " for elasto and " + std::to_string(hydro.length) + " for hydro.");
    }
    if (elasto.diameter != hydro.diameter) {
        throw std::runtime_error("Mooring has different diameters: " + std::to_string(elasto.diameter) +
                                 " for elasto and " + std::to_string(hydro.diameter) + " for hydro.");
    }
}

void Mooring::initialize_this(double time, double dt) {
    perform_sanity_check();

    // mappings
    compute_mapping_hydro2elasto();
    compute_mapping_elasto2hydro();
    // update position of hydro points
    update_positions_hydro();

    spdlog::info("Initialized mooring of total mass {:.4}kg with {} elasto and {} hydro elements.", elasto.get_mass(),
                 elasto.elements.size(), hydro.elements.size());
}

void Mooring::prestep(double time, double dt) {
    // update loads on elasto part
    update_loads_elasto();
}

void Mooring::poststep(double time, double dt) {
    // update position of hydro points
    update_positions_hydro();
}

void Mooring::apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time) {
    hydro.compute_fluid_loads(fluid_model, time);
}

void Mooring::apply_soil_model(seahowl::env::SoilModel& soil_model, double time) {
    elasto.compute_seabed_loads(soil_model);
}

void Mooring::build() {
    // build
    elasto.build();
    hydro.build();
}

void Mooring::set_discretization_elasto(std::vector<double> fractions) {
    elasto.discretization_fractions = fractions;
}

void Mooring::set_discretization_hydro(std::vector<double> fractions) {
    hydro.discretization_fractions = fractions;
}

void Mooring::compute_mapping_hydro2elasto() {
    mapping_hydro2elasto_nodes =
        get_indice_and_positions(hydro.discretization_fractions, elasto.discretization_fractions);

    // get hydro element position (center) from which loads will be applied
    if (hydro.elements.size() != hydro.discretization_fractions.size() - 1) {
        throw std::runtime_error("There are " + std::to_string(hydro.elements.size()) + " elements for " +
                                 std::to_string(hydro.discretization_fractions.size() - 1) +
                                 "discretization fractions (" + std::to_string(hydro.nodes.size()) + " nodes).");
    }
    std::vector<double> hydro_discretization_fractions_elements;
    for (int ii = 0; ii < hydro.elements.size(); ii++) {
        auto element_fraction = 0.5 * (hydro.discretization_fractions[ii] + hydro.discretization_fractions[ii + 1]);
        hydro_discretization_fractions_elements.push_back(element_fraction);
    }
    mapping_hydro2elasto_elements =
        get_indice_and_positions(hydro_discretization_fractions_elements, elasto.discretization_fractions);
}

void Mooring::compute_mapping_elasto2hydro() {
    mapping_elasto2hydro = get_indice_and_positions(elasto.discretization_fractions, hydro.discretization_fractions);
}

void Mooring::update_positions_hydro() {
    for (int ii = 0; ii < hydro.nodes.size(); ii++) {
        auto& node_hydro = hydro.nodes[ii];

        // update position and rotation of hydro elements
        int elasto_element_index = mapping_hydro2elasto_nodes[ii].index;
        double eta = mapping_hydro2elasto_nodes[ii].eta;
        auto entity = elasto.get_entity_along_component(eta, elasto_element_index);
        node_hydro.set_rotation(entity.get_rotation());
        node_hydro.set_position(entity.get_position());
        node_hydro.set_velocity(entity.get_velocity());
        node_hydro.set_rotational_velocity(entity.get_rotational_velocity());
        node_hydro.set_acceleration(entity.get_acceleration());
        node_hydro.set_rotational_acceleration(entity.get_rotational_acceleration());
    }
}

void Mooring::update_loads_elasto() {
    elasto.reset_loads();
    if (hydro.loads.size() != mapping_hydro2elasto_elements.size()) {
        throw std::runtime_error("Mooring: length of vector of loads (" + std::to_string(hydro.loads.size()) +
                                 ") and length of hydro to elasto mapping (" +
                                 std::to_string(mapping_hydro2elasto_elements.size()) + ") do not match.");
    }
    auto offset = Vector3d(0.0, 0.0, 0.0);
    for (int ii = 0; ii < hydro.loads.size(); ii++) {
        elasto.accumulate_element_load(hydro.loads[ii], Vector3d(0.0, 0.0, 0.0),
                                       mapping_hydro2elasto_elements[ii].index, mapping_hydro2elasto_elements[ii].eta,
                                       offset);
    }
}

MooringSystem::MooringSystem(seahowl::elasto::MooringSystemElasto& elasto, seahowl::hydro::MooringSystemHydro& hydro)
    : elasto(elasto), hydro(hydro) {}

void MooringSystem::add_mooring(std::shared_ptr<Mooring> mooring) {
    moorings.push_back(mooring);
}

void MooringSystem::perform_sanity_check() {
    for (auto& mooring : moorings) {
        // check that moorings are also present in their elasto and hydro system counterparts
        bool found_elasto = false;
        bool found_hydro = false;
        for (auto& mooring_elasto : elasto.moorings) {
            if (&mooring->elasto == &*mooring_elasto) {
                found_elasto = true;
            }
        }
        if (!found_elasto) {
            throw std::runtime_error("Did not find mooring in mooring_system elasto class");
        }
        for (auto& mooring_hydro : hydro.moorings) {
            if (&mooring->hydro == &*mooring_hydro) {
                found_hydro = true;
            }
        }
        if (!found_hydro) {
            throw std::runtime_error("Did not find mooring in mooring_system hydro class");
        }
    }
}

void MooringSystem::initialize_this(double time, double dt) {
    perform_sanity_check();
    for (auto& mooring : moorings) {
        mooring->initialize(time, dt);
    }
}

void MooringSystem::prestep(double time, double dt) {
    for (auto& mooring : moorings) {
        mooring->prestep(time, dt);
    }
}

void MooringSystem::poststep(double time, double dt) {
    for (auto& mooring : moorings) {
        mooring->poststep(time, dt);
    }
}

void MooringSystem::apply_fluid_model(seahowl::env::FluidModel& fluid_model, double time) {
    for (auto& mooring : moorings) {
        mooring->apply_fluid_model(fluid_model, time);
    }
}

void MooringSystem::apply_soil_model(seahowl::env::SoilModel& soil_model, double time) {
    for (auto& mooring : moorings) {
        mooring->apply_soil_model(soil_model, time);
    }
}

void MooringSystem::build() {
    for (auto& mooring : moorings) {
        mooring->build();
    }
}
