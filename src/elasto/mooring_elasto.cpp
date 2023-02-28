#pragma once

#include <seahowl/elasto/mooring_elasto.h>
#include <seahowl/core/utils.h>

#include <chrono/fea/ChElementBeamEuler.h>
#include <chrono/fea/ChContactSurfaceNodeCloud.h>
#include <chrono/physics/ChMaterialSurfaceSMC.h>

using namespace seahowl::elasto;

MooringElasto::MooringElasto() {}

MooringElasto::~MooringElasto() {}

void MooringElasto::build() {
    //    // make reference points between fairlead and anchor
    //    std::vector<ReferencePointElasto> points;
    //    for (auto& fraction : discretization_fractions) {
    //        auto point = ReferencePointElasto();
    //        point.coordinates = fairlead_position + (anchor_position - fairlead_position) * fraction;
    //        point.fraction = fraction;
    //        points.push_back(point);
    //    }
    //    // build
    //    build_nodes(points);
    //    build_elements_euler();
    //}
    //
    // void MooringElasto::build_elements_euler() {
    //    elements.clear();
    //    const auto nelements = nodes.size() - 1;
    //
    //    if (nelements <= 0) {
    //        throw std::runtime_error("Trying to build blade with no element.");
    //    }
    //
    //    // make section
    //    auto section = chrono_types::make_shared<chrono::fea::ChBeamSectionEulerAdvanced>();
    //    // material properties
    //    section->SetDensity(density);
    //    // axial
    //    double area = chrono::CH_C_PI * pow(diameter, 2) / 4.0;
    //    section->SetArea(area);
    //    section->SetYoungModulus(stiffness_axial / area);
    //    section->SetGshearModulus(0.0);
    //    section->SetAsCircularSection(diameter);
    //
    //    for (size_t ii = 1; ii < nelements + 1; ii++) {
    //        // create element
    //        auto element = chrono_types::make_shared<chrono::fea::ChElementBeamEuler>();
    //        // add element to elements vector
    //        elements.push_back(element);
    //        // set element nodes
    //        element->SetNodes(nodes[ii - 1], nodes[ii]);
    //        // set section
    //        element->SetSection(section);
    //        // set rest length
    //        element->SetRestLength(length * abs(discretization_fractions[ii] - discretization_fractions[ii - 1]));
    //    }
}

void MooringElasto::assemble(std::shared_ptr<MeshElasto> mesh) {
    //// call parent class assemble
    // ComponentElastoFEA::assemble(mesh);

    //// make contact material
    // contact_material = chrono_types::make_shared<chrono::ChMaterialSurfaceSMC>();
    // double area = chrono::CH_C_PI * pow(diameter, 2) / 4.0;
    // contact_material->SetYoungModulus(stiffness_axial / area);
    // contact_material->SetFriction(0.3f);
    // contact_material->SetRestitution(0.2f);
    // contact_material->SetAdhesion(0);

    //// assemble contact node cloud
    // contact_cloud = chrono_types::make_shared<chrono::fea::ChContactSurfaceNodeCloud>(contact_material, mesh.get());
    // for (auto& node : nodes) {
    //     contact_cloud->AddNode(node, diameter);
    // }
    // mesh->AddContactSurface(contact_cloud);
}
