#include <seahowl/elasto/elasto.h>



// Forward declaration
class vtkUnstructuredGrid;
class vtkXMLUnstructuredGridWriter;

/**@brief Output with VTK format */
struct OutputMeshVTK {
    vtkUnstructuredGrid* mesh;
    vtkXMLUnstructuredGridWriter* writer;

    seahowl::elasto::ComponentElastoFEA& component;

    double time;
    double dt;
    std::string base = "";

    OutputMeshVTK(seahowl::elasto::ComponentElastoFEA& component);
    ~OutputMeshVTK();

    void init(const char* base_name);
    void write(double time, int time_step) const;
};