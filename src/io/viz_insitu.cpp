#include "seahowl/io/viz_insitu.h"

#include <chrono/fea/ChVisualizationFEAmesh.h>

void draw_system_init(chrono::ChSystem& system, chrono::irrlicht::ChIrrApp& application) {
    // initialize default
    application.AddTypicalLights();
    application.AddTypicalSky();
    application.AddTypicalCamera(irr::core::vector3df(-150, -150, 150), irr::core::vector3df(0, 0, 150.));

    // meshes
    for (auto mesh : system.Get_meshlist()) {
        // beams
        auto beams_visu = chrono_types::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(mesh.get()));
        beams_visu->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
        beams_visu->SetColorscaleMinMax(-0.4, 0.4);
        mesh->AddAsset(beams_visu);

        // nodes
        auto nodes_visu = chrono_types::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(mesh.get()));
        nodes_visu->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
        nodes_visu->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NODE_DISP_Y);
        nodes_visu->SetSymbolsThickness(1.5);
        nodes_visu->SetSymbolsScale(1.0);
        nodes_visu->SetZbufferHide(false);
        mesh->AddAsset(nodes_visu);

        // nodes coordinate system
        auto nodes_visu2 = chrono_types::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(mesh.get()));
        nodes_visu2->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
        nodes_visu2->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NONE);
        nodes_visu2->SetSymbolsThickness(10.0);
        nodes_visu2->SetSymbolsScale(1.0);
        nodes_visu2->SetZbufferHide(false);
        mesh->AddAsset(nodes_visu2);
    }

    // bind assets
    application.AssetBindAll();
    application.AssetUpdateAll();
    application.AddShadowAll();
}

void draw_system(chrono::ChSystem& system, chrono::irrlicht::ChIrrApp& application) {
    // irrlicht must prepare frame to draw
    application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));

    // draw items belonging to Irrlicht scene, if any
    application.DrawAll();

    // draw bodies
    for (auto body : system.Get_bodylist()) {
        chrono::irrlicht::tools::drawCircle(application.GetVideoDriver(), 5.0, body->GetCoord());
    }
    chrono::irrlicht::tools::drawAllCOGs(system, application.GetVideoDriver(), 5.0);

    // grid
    chrono::irrlicht::tools::drawGrid(application.GetVideoDriver(), 10, 10);

    application.GetIGUIEnvironment()->drawAll();
}