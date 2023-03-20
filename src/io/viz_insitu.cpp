#include "seahowl/io/viz_insitu.h"

#include <seahowl/commons/numerics.h>

using seahowl::Vector3d;

void draw_system_init(std::shared_ptr<chrono::ChSystem> system,
                      std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> application) {
    // initialize default
    application->AddTypicalLights();
    application->AddSkyBox();
    application->AddCamera(Vector3d(-150, -150, 150), Vector3d(0, 0, 150.));

    // meshes
    for (auto mesh : system->Get_meshlist()) {
        // beams
        auto beams_visu = chrono_types::make_shared<chrono::ChVisualShapeFEA>(mesh);
        beams_visu->SetFEMdataType(chrono::ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
        beams_visu->SetColorscaleMinMax(-0.4, 0.4);
        mesh->AddVisualShapeFEA(beams_visu);

        // nodes
        auto nodes_visu = chrono_types::make_shared<chrono::ChVisualShapeFEA>(mesh);
        nodes_visu->SetFEMglyphType(chrono::ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
        nodes_visu->SetFEMdataType(chrono::ChVisualShapeFEA::DataType::NODE_DISP_Y);
        nodes_visu->SetSymbolsThickness(1.5);
        nodes_visu->SetSymbolsScale(1.0);
        nodes_visu->SetZbufferHide(false);
        mesh->AddVisualShapeFEA(nodes_visu);

        // nodes coordinate system
        auto nodes_visu2 = chrono_types::make_shared<chrono::ChVisualShapeFEA>(mesh);
        nodes_visu2->SetFEMglyphType(chrono::ChVisualShapeFEA::GlyphType::NODE_CSYS);
        nodes_visu2->SetFEMdataType(chrono::ChVisualShapeFEA::DataType::NONE);
        nodes_visu2->SetSymbolsThickness(10.0);
        nodes_visu2->SetSymbolsScale(1.0);
        nodes_visu2->SetZbufferHide(false);
        mesh->AddVisualShapeFEA(nodes_visu2);
    }

    // bind assets

    application->AttachSystem(system.get());
    application->SetShadows(true);
}

void draw_system(const std::shared_ptr<chrono::ChSystem> system,
                 std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> application) {
    // irrlicht must prepare frame to draw
    application->BeginScene(true, true, chrono::ChColor(255, 140, 161));

    // draw items belonging to Irrlicht scene, if any
    application->Render();

    // draw bodies
    for (auto body : system->Get_bodylist()) {
        chrono::irrlicht::tools::drawCircle(application.get(), 5.0, body->GetCoord());
    }
    chrono::irrlicht::tools::drawAllCOGs(application.get(), 5.0);

    // grid
    // chrono::irrlicht::tools::drawGrid(application, 10, 10);
}
