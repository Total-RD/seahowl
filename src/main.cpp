
#include <chrono/fea/ChVisualizationFEAmesh.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChIterativeSolverLS.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <chrono/solver/ChDirectSolverLS.h>

#ifdef HAVE_VTK
    #include <seahowl/io/write_vtk.h>
#endif

#include <cmath>

#include <seahowl/io/read_json.h>
#include <seahowl/io/write_csv.h>
#include <seahowl/servo/controller.h>

#ifdef HAVE_ROSCO
    #include <seahowl/servo/controller_discon.h>
#endif

#include <seahowl/aero/wind_models.h>
#include <seahowl/core/system.h>

#include <filesystem>  // C++17

using std::filesystem::path;
using std::filesystem::create_directory;
using std::filesystem::remove_all;

void output_results(seahowl::core::System& seahowl_system, chrono::ChSystemSMC& system) {
    // output
    auto time = system.GetChTime();
    chrono::GetLog() << "time " << system.GetChTime() << " rpm: " << seahowl_system.turbine.rotor.elasto.get_rpm()
                     << "\n";
    write_turbine_info_to_csv("output.csv", seahowl_system, system.GetChTime());
}

/**@brief Driver main function */
int main(int argc, char* argv[]) {
    // SETUP

    auto DATADIR = absolute(path(u8"../data"));
    auto logoname = (DATADIR / ".." / "doc" / "source" / "totalenergies_alpha.png").generic_string();

    // path of main input file
    auto filepath_main = DATADIR / "IEA15MW_main.json";
    if (argc > 1) {
        filepath_main = absolute(path(argv[1]));
    }

    std::ifstream json_file(filepath_main);
    // populate json object
    json json_obj;
    json_file >> json_obj;

    auto filepath_turbine =
        (absolute(filepath_main.parent_path()) / json_obj.at("turbines")[0].at("file").get<std::string>())
            .generic_string();
    auto environment_json = json_obj.at("environment");
    auto gravity = environment_json.at("gravity").get<std::vector<double>>();
    auto wind_json = environment_json.at("wind");

    bool statics_prestep = true;
    // timestepping
    auto timestepper_type = chrono::ChTimestepper::Type::HHT;
    double dt = json_obj.at("numerics").at("dt").get<double>();
    double dt_outputs = json_obj.at("numerics").at("dt_outputs").get<double>();
    // system
    chrono::ChSystemSMC system;
    system.Set_G_acc(chrono::ChVector<double>(gravity[0], gravity[1], gravity[2]));
    system.SetNumThreads(chrono::ChOMP::GetNumProcs(), 0, 1);

    // wind
    seahowl::aero::WindRamp wind_model;
    if (wind_json.at("type").get<std::string>() == "ramp") {
        auto wind_options = wind_json.at("options");
        wind_model = seahowl::aero::WindRamp();
        auto v0 = wind_options.at("velocity_start").get<std::vector<double>>();
        wind_model.wind_velocity_start = chrono::ChVector<double>(v0[0], v0[1], v0[2]);
        auto v1 = wind_options.at("velocity_stop").get<std::vector<double>>();
        wind_model.wind_velocity_stop = chrono::ChVector<double>(v1[0], v1[1], v1[2]);
        wind_model.direction_gravity = system.Get_G_acc().GetNormalized();
        wind_model.reference_height = wind_options.at("reference_height").get<double>();
        wind_model.time_start = wind_options.at("time_start").get<double>();
        wind_model.time_stop = wind_options.at("time_stop").get<double>();
        wind_model.shear_coefficient = wind_options.at("shear_coefficient").get<double>();
        wind_model.density = environment_json.at("air_density").get<double>();
    } else {
        throw std::runtime_error("Only wind ramp is allowed as input.");
    }

    // solver
    auto solver = chrono_types::make_shared<chrono::ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    // timestepper
    system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<chrono::ChTimestepperHHT>(system.GetTimestepper())) {
        mystepper->SetStepControl(false);
        mystepper->SetModifiedNewton(false);
    }

    // mesh
    auto mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system.AddMesh(mesh);

    auto seahowl_system =
        seahowl::core::System(seahowl::core::Turbine(get_turbine_from_json(filepath_turbine)), wind_model);
    auto& turbine = seahowl_system.turbine;

    // build turbine (Chrono)
    turbine.build();
    turbine.assemble(system, mesh);
    turbine.tower.elasto.nodes.front()->SetFixed(true);  // foundation of the tower

    auto turbine_json = json_obj.at("turbines")[0];
    // rotate turbine to align tower with gravity vector
    auto v1 = -system.Get_G_acc().GetNormalized();
    auto v2 = (seahowl_system.turbine.tower.elasto.nodes[1]->GetPos() -
               seahowl_system.turbine.tower.elasto.nodes[0]->GetPos())
                  .GetNormalized();
    auto rot_axis = v2 % v1;
    auto rot_angle = acos(v1 ^ v2);
    turbine.rotate(rot_angle, rot_axis);
    // rotation around axis opposite to gravity (yaw)
    turbine.rotate(turbine_json.at("rotation").get<double>(), -system.Get_G_acc().GetNormalized());
    // translate turbine
    auto trans = turbine_json.at("translation").get<std::vector<double>>();
    turbine.translate(chrono::ChVector<double>(trans[0], trans[1], trans[2]));

    // apply initial pitches
    for (auto blade : turbine.blades) {
        auto pitch0 = blade->elasto->pitch;
        blade->elasto->apply_pitch_increment(pitch0);
        blade->elasto->pitch = pitch0;
    }
    auto rotor_pitch0 = turbine.rotor.elasto.pitch_collective;
    turbine.rotor.elasto.apply_collective_pitch_increment(rotor_pitch0);
    turbine.rotor.elasto.pitch_collective = rotor_pitch0;

#ifdef HAVE_VTK
    std::vector<OutputMeshVTK> vtk_outputs;
    remove_all("./vtk");
    create_directory("./vtk");
    for (int ii = 0; ii < seahowl_system.turbine.rotor.blades.size(); ii++) {
        auto& post_blade = vtk_outputs.emplace_back(*seahowl_system.turbine.rotor.blades[ii]->elasto.get());
        post_blade.init(("./vtk/blade" + std::to_string(ii + 1)).c_str());
    }
    auto& post_tower = vtk_outputs.emplace_back(seahowl_system.turbine.tower.elasto);
    post_tower.init("./vtk/tower");
#endif

    // simulation loop

    // initialization
    // statics
    if (statics_prestep) {
        system.DoStaticLinear();
        system.DoStaticNonlinear(10, true);
    }

    seahowl_system.init(system.GetChTime(), dt);

    output_results(seahowl_system, system);
#ifdef HAVE_VTK
    for (auto const& vtk_output : vtk_outputs) {
        vtk_output.write(time, step);
    }
#endif
    int step = 0;
    double dt_outputs_next = dt_outputs;
    while (true) {
        // prestep
        seahowl_system.prestep(system.GetChTime(), dt);

        // step
        system.DoStepDynamics(dt);
        step += 1;

        // poststep
        seahowl_system.poststep(system.GetChTime(), dt);

        // output
        if (system.GetChTime() >= (dt_outputs_next - dt_outputs_next * 1e-6)) {
            output_results(seahowl_system, system);
#ifdef HAVE_VTK
            for (auto const& vtk_output : vtk_outputs) {
                vtk_output.write(time, step);
            }
#endif
        }
        while (system.GetChTime() >= (dt_outputs_next - dt_outputs_next * 1e-6)) {
            dt_outputs_next += dt_outputs;
        }
    }

    return 0;
}
