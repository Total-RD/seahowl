#include <seahowl/io/write_vtk.h>

#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>

#include <chrono/core/ChVector.h>

OutputMeshVTK::OutputMeshVTK(seahowl::elasto::ComponentElastoFEA& component) : component(component) {
    mesh = vtkSmartPointer<vtkUnstructuredGrid>::New();
    writer = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
}

OutputMeshVTK::~OutputMeshVTK() {}

void OutputMeshVTK::init(const char* base_name) {
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

    arrays_map.insert({"Displacement", vtkSmartPointer<vtkDoubleArray>::New()});
    arrays_map.insert({"Forces", vtkSmartPointer<vtkDoubleArray>::New()});
    arrays_map.insert({"Velocity", vtkSmartPointer<vtkDoubleArray>::New()});
    arrays_map.insert({"Acceleration", vtkSmartPointer<vtkDoubleArray>::New()});
    arrays_map.insert({"Direction", vtkSmartPointer<vtkDoubleArray>::New()});
    arrays_map.insert({"Rotation", vtkSmartPointer<vtkDoubleArray>::New()});
    // initialize arrays properties
    for (auto const& keyval : arrays_map) {
        auto& key = keyval.first;
        auto& val = keyval.second;
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
    std::map<std::string, std::vector<chrono::ChVector<double>>> arrays_values;

    // positions
    auto points = mesh->GetPoints();
    double* pDst0 = static_cast<double*>(points->GetVoidPointer(0));
    auto& values0 = component.get_nodes_positions();
    memcpy(pDst0, &values0[0], sizeof(double) * values0.size() * 3);
    mesh->SetPoints(points);

    // vectors
    arrays_values.insert({"Displacement", component.get_nodes_positions()});
    arrays_values.insert({"Forces", component.get_nodes_loads()});
    arrays_values.insert({"Velocity", component.get_nodes_velocities()});
    arrays_values.insert({"Acceleration", component.get_nodes_accelerations()});
    arrays_values.insert({"Direction", component.get_nodes_directions()});

    auto* initial_coords = static_cast<double*>(mesh->GetPoints()->GetVoidPointer(0));

    for (auto const& keyval : arrays_map) {
        auto& key = keyval.first;
        auto& val = keyval.second;
        auto arr = mesh->GetPointData()->GetArray(key.c_str());

        double* pDst = static_cast<double*>(val->GetVoidPointer(0));
        auto& values = arrays_values[key];
        memcpy(pDst, &values[0], sizeof(double) * values.size() * 3);

        // remove initial coords for displacement
        if (key == "Displacement") {
            for (auto idx = 0; idx < 3 * values.size(); ++idx) {
                pDst[idx] -= initial_coords[idx];
            }
        }
    }

    // quaternions
    auto arr = mesh->GetPointData()->GetArray("Rotation");
    double* pDst = static_cast<double*>(arr->GetVoidPointer(0));
    auto& values = component.get_nodes_rotations();
    memcpy(pDst, &values[0], sizeof(double) * values.size() * 4);

    char fname[2048];
    std::sprintf(fname, "%s_%03d.vtu", base.c_str(), time_step);
    writer->SetFileName(fname);
    writer->SetInputData(mesh);
    writer->Write();
}