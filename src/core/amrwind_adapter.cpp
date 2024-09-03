#include "seahowl/core/amrwind_adapter.h"

#include "seahowl/core/system.h"
#include "seahowl/elasto/system_elasto.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/aero/system_aero.h"
#include "seahowl/elasto/chrono_adapters.h"
#include "seahowl/io/read_json.h"
#include "seahowl/io/write_csv.h"
#include "seahowl/io/output_manager.h"

#include <fstream>
#include <filesystem>  // C++17
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <typeinfo>
#include <iostream>

using namespace seahowl::core;
namespace fs = std::filesystem;
using json = nlohmann::json;

AmrWindAdapter::AmrWindAdapter() {
    system_elasto = std::make_unique<seahowl::elasto::SystemElastoChrono>();
    system_aero = std::make_unique<seahowl::aero::SystemAero>();
    system_core = std::make_unique<System>(*system_elasto, *system_aero);
    outputs = std::make_unique<seahowl::io::OutputManager>(*system_core);
}

void AmrWindAdapter::populate_from_file(const std::string& filepath) {
    spdlog::stopwatch sw_setup;
    spdlog::set_pattern("[%^%l%$] %v");
    spdlog::info("**************************************************************");
    spdlog::info("INITIAL AmrWindAdapter SETUP.");
    spdlog::info("**************************************************************");

    populate_system_from_json(filepath, *system_core);

    // get main file info
    std::ifstream json_file(filepath);
    json json_obj;
    json_file >> json_obj;
    json_file.close();

    // NUMERICS options
    auto num_json = json_obj.at("numerics");
    // timestepping
    dt = num_json.at("dt").get<double>();
    duration = num_json.at("t_end").get<double>();
    // outputs
    auto outputs_json = json_obj.at("outputs");
    dt_output = outputs_json.at("dt").get<double>();
    std::string output_folder = "./output";
    if (outputs_json.contains("folder")) {
        output_folder = outputs_json.at("folder").get<std::string>();
    }

    // outputs
    outputs = std::make_unique<seahowl::io::OutputManager>(*system_core);
    outputs->set_output_folder(output_folder);
    outputs->has_vtk = outputs_json.at("VTK").get<bool>();
    outputs->has_gui = outputs_json.at("gui").get<bool>();

    spdlog::debug("Populated system in {:.3}s.", sw_setup);

    // turbine aero discretisation
    auto turbines_json = json_obj.at("turbines");
    // AMR-Wind coupling on single WT, to be fixed for multiple turbines
    auto turbine_json = turbines_json[0];
    auto main_path = fs::path(filepath).parent_path();
    auto filepath_turbine = (main_path / turbine_json.at("file").get<std::string>()).generic_string();

    std::ifstream turbines_json_file(filepath_turbine);
    json turbines_json_obj;
    turbines_json_file >> turbines_json_obj;
    turbines_json_file.close();

    // number of blades
    // numBlade = system_aero->turbines[0]->rna.rotor->blades.size();
    auto blades_json = turbines_json_obj.at("rotor").at("blades");
    numBlade = blades_json.size();

    // number of nodes per blade
    // numBladeNode = system_aero->turbines[0]->rna.rotor->blades[0]nodes.size();
    auto blade_discretisation = turbines_json_obj.at("rotor").at("discretization").at("aero").get<std::vector<int>>();
    numBladeNode = blade_discretisation[0] + 1;

    // number of nodes on tower
    // numTowerNode = system_aero->turbines[0].tower->nodes.size();
    auto tower_discretisation = turbines_json_obj.at("tower").at("discretization").at("aero").get<std::vector<int>>();
    numTowerNode = tower_discretisation[0] + 1;
}

void AmrWindAdapter::initialize() {
    if (is_initialized) {
        throw std::runtime_error("AmrWindAdapter was already initialized.");
    }

    outputs->initialize();

    system_core->initialize(system_core->get_time(), dt);

    is_initialized = true;
}

void AmrWindAdapter::initialize_from_file(const std::string& filepath) {
    if (is_initialized) {
        throw std::runtime_error("AmrWindAdapter was already initialized.");
    }
    spdlog::stopwatch sw_setup;

    outputs->initialize();

    initialize_system_from_json(filepath, *system_core);

    outputs->output_all(0);

    t_output_next = dt_output;

    is_initialized = true;
    spdlog::info("Initial setup time: {:.3}s.", sw_setup);
}

void AmrWindAdapter::init_OpFM_data(int* NumActForcePtsBlade,
                                    int* NumActForcePtsTower,
                                    OpFM_InputType* to_cfd,
                                    OpFM_OutputType* from_cfd) {
    /* Motion nodes from Seahowl */

    // Hub node (As the coupling between AMR-Wind and OpenFAST, hub is first point always)
    auto nNodesVel = 1;

    // Blade nodes
    nNodesVel = nNodesVel + numBlade * numBladeNode;

    // Tower nodes
    nNodesVel = nNodesVel + numTowerNode;

    // postion of Seahowl aero nodes
    to_cfd->pxVel_Len = nNodesVel;
    to_cfd->pyVel_Len = nNodesVel;
    to_cfd->pzVel_Len = nNodesVel;

    AllocPAry(to_cfd->pxVel, to_cfd->pxVel_Len, "pxVel");
    AllocPAry(to_cfd->pyVel, to_cfd->pyVel_Len, "pyVel");
    AllocPAry(to_cfd->pzVel, to_cfd->pzVel_Len, "pzVel");

    // velocity at Seahowl aero nodes
    from_cfd->u_Len = nNodesVel;
    from_cfd->v_Len = nNodesVel;
    from_cfd->w_Len = nNodesVel;

    AllocPAry(from_cfd->u, from_cfd->u_Len, "u");
    AllocPAry(from_cfd->v, from_cfd->v_Len, "v");
    AllocPAry(from_cfd->w, from_cfd->w_Len, "w");

    /* Actuator nodes from AMR-Wind */

    // Hub node
    auto nNodesForce = 1;

    // Blade nodes
    nNodesForce = nNodesForce + numBlade * *NumActForcePtsBlade;

    // Tower nodes
    nNodesForce = nNodesForce + *NumActForcePtsTower;

    // postion of AMR-Wind actuator nodes
    to_cfd->pxForce_Len = nNodesForce;
    to_cfd->pyForce_Len = nNodesForce;
    to_cfd->pzForce_Len = nNodesForce;

    AllocPAry(to_cfd->pxForce, to_cfd->pxForce_Len, "pxForce");
    AllocPAry(to_cfd->pyForce, to_cfd->pyForce_Len, "pyForce");
    AllocPAry(to_cfd->pzForce, to_cfd->pzForce_Len, "pzForce");

    // velocity at AMR-Wind actuator nodes
    to_cfd->xdotForce_Len = nNodesForce;
    to_cfd->ydotForce_Len = nNodesForce;
    to_cfd->zdotForce_Len = nNodesForce;

    AllocPAry(to_cfd->xdotForce, to_cfd->xdotForce_Len, "xdotForce");
    AllocPAry(to_cfd->ydotForce, to_cfd->ydotForce_Len, "ydotForce");
    AllocPAry(to_cfd->zdotForce, to_cfd->zdotForce_Len, "zdotForce");

    // Orientation matrix at AMR-Wind actuator nodes
    to_cfd->pOrientation_Len = 3 * 3 * nNodesForce;

    AllocPAry(to_cfd->pOrientation, to_cfd->pOrientation_Len, "pOrientation");

    // normalized force at AMR-Wind actuator nodes
    to_cfd->fx_Len = nNodesForce;
    to_cfd->fy_Len = nNodesForce;
    to_cfd->fz_Len = nNodesForce;

    AllocPAry(to_cfd->fx, to_cfd->fx_Len, "fx");
    AllocPAry(to_cfd->fy, to_cfd->fy_Len, "fy");
    AllocPAry(to_cfd->fz, to_cfd->fz_Len, "fz");

    // normalized moment at AMR-Wind actuator nodes
    to_cfd->momentx_Len = nNodesForce;
    to_cfd->momenty_Len = nNodesForce;
    to_cfd->momentz_Len = nNodesForce;

    AllocPAry(to_cfd->momentx, to_cfd->momentx_Len, "momentx");
    AllocPAry(to_cfd->momenty, to_cfd->momenty_Len, "momenty");
    AllocPAry(to_cfd->momentz, to_cfd->momentz_Len, "momentz");

    // chord distribution at AMR-Wind actuator nodes
    to_cfd->forceNodesChord_Len = nNodesForce;

    AllocPAry(to_cfd->forceNodesChord, to_cfd->forceNodesChord_Len, "forceNodesChord");
}

void AmrWindAdapter::step() {
    if (!is_initialized) {
        initialize();
    }
    spdlog::stopwatch sw_step;
    // prestep
    system_core->prestep(system_core->get_time(), dt);

    // step
    system_core->step(dt);
    nstep += 1;

    // poststep
    system_core->poststep(system_core->get_time(), dt);

    // output
    if (system_core->get_time() >= (t_output_next - 1e-6)) {
        spdlog::info("time: {:.6}s, step: {}, stopwatch: {:.3}s", system_core->get_time(), nstep, sw_step);
        outputs->output_all(nstep);
        t_output_next += dt_output;
    }
}

void AmrWindAdapter::AllocPAry(float* array, int size, const std::string& name) {
    try {
        array = new float[size];
        // Initialize to zero
        for (int i = 0; i < size; i++) {
            array[i] = 0.0;
        }
    } catch (const std::bad_alloc&) {
        spdlog::error("Allocation failed for {}", name);
    }
}
