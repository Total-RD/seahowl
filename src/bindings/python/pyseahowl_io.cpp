#include <pybind11/pybind11.h>

#include <seahowl/io/read_json.h>
#include <seahowl/core/blade.h>
#include <seahowl/core/tower.h>
#include <seahowl/core/rotor.h>
#include <seahowl/core/turbine.h>
#include <seahowl/core/system.h>
#include <seahowl/elasto/blade_elasto.h>
#include <seahowl/elasto/tower_elasto.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/tower_aero.h>
#include <seahowl/aero/rotor_aero.h>
#include <seahowl/io/write_csv.h>
#include <seahowl/io/output_manager.h>
#include <seahowl/env/combined_models.h>

namespace py = pybind11;

void initialize_pyseahowl_io(py::module& m) {
    // submodule
    auto m_io = m.def_submodule("io", "Inpout/output submodule.");

    // io/read_json.h
    m_io.def("populate_blade_from_json", &populate_blade_from_json);
    m_io.def("populate_blade_elasto_from_json", &populate_blade_elasto_from_json);
    m_io.def("populate_blade_aero_from_json", &populate_blade_aero_from_json);
    m_io.def("populate_tower_from_json", &populate_tower_from_json);
    m_io.def("populate_tower_elasto_from_json", &populate_tower_elasto_from_json);
    m_io.def("populate_tower_aero_from_json", &populate_tower_aero_from_json);
    m_io.def("populate_rna_from_json", &populate_rna_from_json);
    m_io.def("populate_turbine_from_json", &populate_turbine_from_json);
    m_io.def("add_turbine_to_system_from_json", &add_turbine_to_system_from_json);
    m_io.def("populate_environmental_conditions_from_json", &populate_environmental_conditions_from_json);
    m_io.def("get_environmental_model_from_json", &get_environmental_model_from_json);
    m_io.def("populate_system_from_json", &populate_system_from_json);
    m_io.def("initialize_system_from_json", &initialize_system_from_json);

    // io/write_csv.h
    m_io.def("write_turbine_info_to_csv", &write_turbine_info_to_csv);

    // io/output_manager.h
    py::class_<seahowl::io::OutputManager, std::shared_ptr<seahowl::io::OutputManager>>(m_io, "OutputManager")
        .def(py::init<seahowl::core::System&>())
        .def("set_output_folder", &seahowl::io::OutputManager::set_output_folder)
        .def("initialize", &seahowl::io::OutputManager::initialize)
        .def("output_all", &seahowl::io::OutputManager::output_all)
        .def_readwrite("dt_output", &seahowl::io::OutputManager::dt_output)
        .def_readwrite("has_vtk", &seahowl::io::OutputManager::has_vtk)
        .def_readwrite("has_gui", &seahowl::io::OutputManager::has_gui)
        .def_readwrite("has_csv", &seahowl::io::OutputManager::has_csv);
}
