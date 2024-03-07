#include "seahowl/io/write_vtk.h"
#include "seahowl/commons/numerics.h"
#include "seahowl/core/system.h"
#include "seahowl/core/blade.h"
#include "seahowl/elasto/blade_elasto.h"

#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>

#include <filesystem>  // C++17
#include <string>
#include <vector>
#include <map>
#include <cstdio>

namespace fs = std::filesystem;

using seahowl::Vector3d;
using namespace seahowl::io;

OutputMeshVTK::OutputMeshVTK(seahowl::elasto::ComponentElastoFEA& component) : component(component) {
    mesh = vtkUnstructuredGrid::New();
}

OutputMeshVTK::OutputMeshVTK(const OutputMeshVTK& rhs) : component(rhs.component) {
    mesh = vtkUnstructuredGrid::New();
    mesh->ShallowCopy(rhs.mesh);
    base = rhs.base;
}
OutputMeshVTK::OutputMeshVTK(OutputMeshVTK&& source) noexcept : component(source.component) {
    mesh = source.mesh;
    source.mesh = nullptr;
    base = source.base;
}

OutputMeshVTK::~OutputMeshVTK() {
    if (mesh != nullptr)
        mesh->Delete();
}

void OutputMeshVTK::initialize(const char* base_name) {
    base = base_name;

    auto coords = component.get_nodes_positions();

    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetDataTypeToDouble();
    const vtkIdType nPoints = coords.size();
    points->SetNumberOfPoints(nPoints);
    double* pDst = static_cast<double*>(points->GetVoidPointer(0));
    memcpy(pDst, &coords[0], sizeof(double) * nPoints * 3);
    mesh->SetPoints(points);

    const vtkIdType nCells = nPoints - 1;
    vtkIdType ptIds[2] = {0, 1};

    for (vtkIdType iCell = 0; iCell < nCells; ++iCell) {
        ptIds[0] = iCell;
        ptIds[1] = iCell + 1;
        mesh->InsertNextCell(VTK_LINE, 2, ptIds);
    }

    std::vector<std::string> myKeys = {"Displacement", "Forces", "Velocity", "Acceleration", "Direction", "Rotation"};

    // initialize arrays properties
    for (auto const& key : myKeys) {
        auto val = vtkSmartPointer<vtkDoubleArray>::New();

        val->SetName(key.c_str());

        if (key == "Rotation") {
            val->SetNumberOfComponents(4);
        } else {
            val->SetNumberOfComponents(3);
        }
        val->SetNumberOfTuples(nPoints);
        val->Fill(0.0);
        mesh->GetPointData()->AddArray(val);
    }
}

void OutputMeshVTK::write(double time, int time_step) const {
    std::map<std::string, std::vector<Vector3d>> arrays_values;

    // positions
    auto points = mesh->GetPoints();
    double* pDst0 = static_cast<double*>(points->GetVoidPointer(0));
    auto values0 = component.get_nodes_positions();
    memcpy(pDst0, &values0[0], sizeof(double) * values0.size() * 3);
    mesh->SetPoints(points);

    // vectors
    arrays_values.insert({"Displacement", component.get_nodes_positions()});
    arrays_values.insert({"Forces", component.get_nodes_loads()});
    arrays_values.insert({"Velocity", component.get_nodes_velocities()});
    arrays_values.insert({"Acceleration", component.get_nodes_accelerations()});
    arrays_values.insert({"Direction", component.get_nodes_directions()});

    auto* initial_coords = static_cast<double*>(mesh->GetPoints()->GetVoidPointer(0));

    for (auto const& keyval : arrays_values) {
        auto& key = keyval.first;
        auto& val = keyval.second;
        auto arr = mesh->GetPointData()->GetArray(key.c_str());

        double* pDst = static_cast<double*>(arr->GetVoidPointer(0));

        memcpy(pDst, &val[0], sizeof(double) * val.size() * 3);

        // remove initial coords for displacement
        if (key == "Displacement") {
            for (auto idx = 0; idx < 3 * val.size(); ++idx) {
                pDst[idx] -= initial_coords[idx];
            }
        }
    }

    // quaternions
    auto arr = mesh->GetPointData()->GetArray("Rotation");
    double* pDst = static_cast<double*>(arr->GetVoidPointer(0));
    auto values = component.get_nodes_rotations();
    memcpy(pDst, &values[0], sizeof(double) * values.size() * 4);

    {
        char fname[2048];
        std::sprintf(fname, "%s_%03d.vtu", base.c_str(), time_step);
        auto writer = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
        writer->SetFileName(fname);
        writer->SetInputData(mesh);
        writer->Write();
    }
}

OutputSystemVTK::OutputSystemVTK(seahowl::core::System& system_core, const std::string& output_folder)
    : system_core(system_core), output_folder(output_folder) {}

void OutputSystemVTK::initialize() {
    vtk_meshes.clear();
    for (auto [turbine_ptr, idx_turbine] = std::tuple{system_core.turbines.begin(), 0};
         turbine_ptr != system_core.turbines.end(); turbine_ptr++, idx_turbine++) {
        auto& turbine = *turbine_ptr;
        fs::create_directory(output_folder);
        for (auto [blade_ptr, idx_blade] = std::tuple{turbine->rna.blades.begin(), 0};
             blade_ptr != turbine->rna.blades.end(); blade_ptr++, idx_blade++) {
            auto& blade = *blade_ptr;
            auto& post_blade = vtk_meshes.emplace_back(dynamic_cast<seahowl::elasto::BladeElastoFEA&>(blade->elasto));
            post_blade.initialize(
                (output_folder + "/turbine" + std::to_string(idx_turbine) + "_blade" + std::to_string(idx_blade))
                    .c_str());
        }
        auto& post_tower = vtk_meshes.emplace_back(turbine->tower.elasto);
        post_tower.initialize((output_folder + "/turbine" + std::to_string(idx_turbine) + "_tower").c_str());
    }
}

void OutputSystemVTK::write(int step) {
    for (auto const& vtk_output : vtk_meshes) {
        vtk_output.write(system_core.elasto.get_time(), step);
    }
}
