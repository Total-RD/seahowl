#include <seahowl/elasto/elasto.h>

#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>


struct OutputMeshVTK {
    vtkSmartPointer<vtkUnstructuredGrid> mesh;
    vtkSmartPointer<vtkXMLUnstructuredGridWriter> writer;
    std::map<std::string, vtkSmartPointer<vtkDoubleArray>> arrays_map;
    seahowl::elasto::ComponentElastoFEA& component;

    double time;
    double dt;
    std::string base = "";

    OutputMeshVTK(seahowl::elasto::ComponentElastoFEA& component);
    ~OutputMeshVTK();

    void init(const char* base_name);
    void write(double time, int time_step) const;
};