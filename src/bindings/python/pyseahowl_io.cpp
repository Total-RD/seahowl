#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <seahowl/io/read_input.h>
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

namespace py = pybind11;

void initialize_pyseahowl_io(py::module& m) {
    // submodule
    auto m_io = m.def_submodule("io", "Inpout/output submodule.");

    // io/read_input.h
    m_io.def("populate_blade_from_file", &seahowl::io::populate_blade_from_file);
    m_io.def("populate_blade_elasto_from_file", &seahowl::io::populate_blade_elasto_from_file);
    m_io.def("populate_blade_aero_from_file", &seahowl::io::populate_blade_aero_from_file);
    m_io.def("populate_tower_from_file", &seahowl::io::populate_tower_from_file);
    m_io.def("populate_tower_elasto_from_file", &seahowl::io::populate_tower_elasto_from_file);
    m_io.def("populate_tower_aero_from_file", &seahowl::io::populate_tower_aero_from_file);
    m_io.def("populate_rna_from_file", &seahowl::io::populate_rna_from_file);
    m_io.def("populate_turbine_from_file", &seahowl::io::populate_turbine_from_file);
    m_io.def("add_turbine_to_system_from_file", &seahowl::io::add_turbine_to_system_from_file);
    m_io.def("populate_environmental_conditions_from_file", &seahowl::io::populate_environmental_conditions_from_file);
    m_io.def("get_environmental_model_from_file", &seahowl::io::get_environmental_model_from_file);
    m_io.def("populate_system_from_file", &seahowl::io::populate_system_from_file);

    // io/output_manager.h
    py::class_<seahowl::io::OutputManager, std::shared_ptr<seahowl::io::OutputManager>>(m_io, "OutputManager")
        .def(py::init<seahowl::core::System&>())
        .def("set_output_folder", &seahowl::io::OutputManager::set_output_folder)
        .def("initialize", &seahowl::io::OutputManager::initialize)
        .def("output_all", &seahowl::io::OutputManager::output_all)
        .def("create_new_csv", &seahowl::io::OutputManager::create_new_csv, py::return_value_policy::reference_internal)
        .def_readwrite("dt_output", &seahowl::io::OutputManager::dt_output)
        .def_readwrite("has_vtk", &seahowl::io::OutputManager::has_vtk)
        .def_readwrite("has_gui", &seahowl::io::OutputManager::has_gui)
        .def_readwrite("has_csv", &seahowl::io::OutputManager::has_csv);

    // io/write_csv.h
    py::class_<seahowl::io::CustomCSV, std::shared_ptr<seahowl::io::CustomCSV>>(m_io, "CustomCSV")
        .def(py::init<const std::string&>())
        .def("add_function",
             [](seahowl::io::CustomCSV& custom_csv, const std::string& name, py::object& pyfunction) {
                 bool added_function = false;
                 try {
                     std::function<std::vector<double>()> cfunction =
                         pyfunction.cast<std::function<std::vector<double>()>>();
                     auto vec = cfunction();
                     custom_csv.add_function(name, cfunction);
                     added_function = true;
                 } catch (const std::runtime_error& e) {
                     try {
                         std::function<double()> cfunction = pyfunction.cast<std::function<double()>>();
                         auto vec = cfunction();
                         custom_csv.add_function(name, cfunction);
                         added_function = true;
                     } catch (const std::runtime_error& e) {
                         seahowl::log(e.what(), "error");
                     }
                 }
                 if (!added_function) {
                     seahowl::log("Could not add function with header \"" + name + "\" to CustomCSV", "error");
                 }
             })
        .def("write_row", &seahowl::io::CustomCSV::write_row);
}
