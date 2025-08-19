#include "seahowl/io/viz_insitu_irrlicht.h"

#include "seahowl/commons/numerics.h"
#include "seahowl/core/system.h"
#include "seahowl/elasto/chrono_adapters.h"

#include <chrono/physics/ChSystem.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <spdlog/spdlog.h>
#include <filesystem>  // C++17

namespace fs = std::filesystem;

using seahowl::Vector3d;
using namespace seahowl::io;

VisualizationInSituIrrlicht::VisualizationInSituIrrlicht() {
    application_irrlicht = chrono_types::make_shared<chrono::irrlicht::ChVisualSystemIrrlicht>();
    application_irrlicht->SetWindowTitle("SEAHOWL");
    application_irrlicht->Initialize();
    application_irrlicht->SetCameraVertical(chrono::CameraVerticalDir::Z);
    auto DATADIR = fs::absolute(fs::path(u8"../data"));
    auto logoname = (DATADIR / ".." / "doc" / "source" / "_static" / "totalenergies_alpha.png").generic_string();
    application_irrlicht->AddLogo(logoname);
}

void VisualizationInSituIrrlicht::initialize_elasto(seahowl::elasto::SystemElasto& system_elasto) {
    spdlog::debug("Initializing in situ visualization (Irrlicht).");

    try {
        system_chrono = dynamic_cast<seahowl::elasto::SystemElastoChrono&>(system_elasto).chobj;
    } catch (const std::bad_cast& e) {
        throw std::runtime_error("Can only use Chrono system for in situ visualization.");
    }

    // initialize default
    application_irrlicht->AddTypicalLights();
    application_irrlicht->AddSkyBox();
    if (system_elasto.turbines.size() > 0) {
        auto zpos = system_elasto.turbines[0]->rna->rotor->body_hub->get_position().z();
        application_irrlicht->AddCamera(Vector3d(-zpos, -zpos, zpos), Vector3d(0, 0, zpos));
    } else {
        application_irrlicht->AddCamera(Vector3d(-150, -150, 150), Vector3d(0, 0, 150.));
    }

    // meshes
    for (auto mesh : system_chrono->GetMeshes()) {
        // beams
        auto beams_visu = std::make_shared<chrono::ChVisualShapeFEA>(mesh);
        beams_visu->SetFEMdataType(chrono::ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
        beams_visu->SetColorscaleMinMax(-0.4, 0.4);
        mesh->AddVisualShapeFEA(beams_visu);

        // nodes
        auto nodes_visu = std::make_shared<chrono::ChVisualShapeFEA>(mesh);
        nodes_visu->SetFEMglyphType(chrono::ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
        nodes_visu->SetFEMdataType(chrono::ChVisualShapeFEA::DataType::NODE_DISP_Y);
        nodes_visu->SetSymbolsThickness(1.5);
        nodes_visu->SetSymbolsScale(1.0);
        nodes_visu->SetZbufferHide(false);
        mesh->AddVisualShapeFEA(nodes_visu);

        // nodes coordinate system
        auto nodes_visu2 = std::make_shared<chrono::ChVisualShapeFEA>(mesh);
        nodes_visu2->SetFEMglyphType(chrono::ChVisualShapeFEA::GlyphType::NODE_CSYS);
        nodes_visu2->SetFEMdataType(chrono::ChVisualShapeFEA::DataType::NONE);
        nodes_visu2->SetSymbolsThickness(10.0);
        nodes_visu2->SetSymbolsScale(1.0);
        nodes_visu2->SetZbufferHide(false);
        mesh->AddVisualShapeFEA(nodes_visu2);
    }

    // bind assets

    application_irrlicht->AttachSystem(system_chrono.get());
    application_irrlicht->SetShadows(true);

    spdlog::debug("Finished initializing in situ visualization (Irrlicht).");
};

void VisualizationInSituIrrlicht::initialize(seahowl::core::System& system) {
    initialize_elasto(system.elasto);
};

void VisualizationInSituIrrlicht::draw() {
    application_irrlicht->GetDevice()->run();

    // irrlicht must prepare frame to draw
    application_irrlicht->BeginScene(true, true, chrono::ChColor(255, 140, 161));

    // draw items belonging to Irrlicht scene, if any
    application_irrlicht->Render();

    // draw bodies
    for (auto body : system_chrono->GetBodies()) {
        chrono::irrlicht::tools::drawCircle(application_irrlicht.get(), 5.0, body->GetCoordsys());
    }
    chrono::irrlicht::tools::drawAllCOGs(application_irrlicht.get(), 5.0);

    // grid
    // chrono::irrlicht::tools::drawGrid(application, 10, 10);

    application_irrlicht->EndScene();
}
