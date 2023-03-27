#include <seahowl/elasto/component_elasto.h>

// Forward declaration
class vtkUnstructuredGrid;

/**@brief Output with VTK format */
struct OutputMeshVTK {
    vtkUnstructuredGrid* mesh;

    seahowl::elasto::ComponentElastoFEA& component;

    double time = 0.0;
    double dt = 0.0;
    std::string base = "";

    OutputMeshVTK(seahowl::elasto::ComponentElastoFEA& component);
    OutputMeshVTK(const OutputMeshVTK&);
    OutputMeshVTK(OutputMeshVTK&&) noexcept;
    OutputMeshVTK& operator=(const OutputMeshVTK&) = delete;
    OutputMeshVTK& operator=(OutputMeshVTK&&) = delete;

    ~OutputMeshVTK();

    void initialize(const char* base_name);
    void write(double time, int time_step) const;
};
