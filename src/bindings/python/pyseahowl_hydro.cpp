#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/hydro/morison.h>
#include <seahowl/hydro/mooring_hydro.h>
#include <seahowl/hydro/floater_hydro.h>
#ifdef HAVE_HYDROCHRONO
    #include <seahowl/hydro/hydrochrono_adapter.h>
#endif

namespace py = pybind11;

void initialize_pyseahowl_hydro(py::module& m) {
    // submodule
    auto m_hydro = m.def_submodule("hydro", "Hydro submodule.");

#ifdef HAVE_HYDROCHRONO
    py::class_<seahowl::hydro::FloaterHydroChrono, std::shared_ptr<seahowl::hydro::FloaterHydroChrono>,
               seahowl::elasto::FloaterElasto>(m_hydro, "FloaterHydroChrono")
        .def(py::init<>())
        .def("initialize", &seahowl::hydro::FloaterHydroChrono::initialize)
        .def("set_h5_filepath", &seahowl::hydro::FloaterHydroChrono::set_h5_filepath)
        .def("set_waves", &seahowl::hydro::FloaterHydroChrono::set_waves);
#endif

    // hydro/morison.h
    py::class_<seahowl::hydro::HydroCoefficients, std::shared_ptr<seahowl::hydro::HydroCoefficients>>(
        m_hydro, "HydroCoefficients")
        .def(py::init<>())
        .def_readwrite("drag_normal", &seahowl::hydro::HydroCoefficients::drag_normal)
        .def_readwrite("drag_axial", &seahowl::hydro::HydroCoefficients::drag_axial)
        .def_readwrite("added_mass_normal", &seahowl::hydro::HydroCoefficients::added_mass_normal)
        .def_readwrite("added_mass_axial", &seahowl::hydro::HydroCoefficients::added_mass_axial)
        .def_readwrite("has_buoyancy", &seahowl::hydro::HydroCoefficients::has_buoyancy)
        .def_readwrite("has_inertia", &seahowl::hydro::HydroCoefficients::has_inertia);
    py::class_<seahowl::hydro::MorisonNode, std::shared_ptr<seahowl::hydro::MorisonNode>, seahowl::EntityDynamicEigen>(
        m_hydro, "MorisonNode")
        .def(py::init<>())
        .def("compute_fluid_loads", &seahowl::hydro::MorisonNode::compute_fluid_loads)
        .def_readwrite("load", &seahowl::hydro::MorisonNode::load)
        .def_readwrite("diameter", &seahowl::hydro::MorisonNode::diameter)
        .def_readwrite("coefficients", &seahowl::hydro::MorisonNode::coefficients);
    py::class_<seahowl::hydro::MorisonElement, std::shared_ptr<seahowl::hydro::MorisonElement>>(m_hydro,
                                                                                                "MorisonElement")
        .def(py::init<const seahowl::hydro::MorisonNode&, const seahowl::hydro::MorisonNode&>())
        .def("get_load", &seahowl::hydro::MorisonElement::get_load)
        .def("get_position", &seahowl::hydro::MorisonElement::get_position)
        .def("get_rotation", &seahowl::hydro::MorisonElement::get_rotation)
        .def_property_readonly("node1", [](seahowl::hydro::MorisonElement& element) { return &element.node1; })
        .def_property_readonly("node2", [](seahowl::hydro::MorisonElement& element) { return &element.node2; })
        .def_readwrite("length", &seahowl::hydro::MorisonElement::length);
    py::class_<seahowl::hydro::MorisonPlate, std::shared_ptr<seahowl::hydro::MorisonPlate>,
               seahowl::EntityDynamicEigen>(m_hydro, "MorisonPlate")
        .def(py::init<>())
        .def("compute_fluid_loads", &seahowl::hydro::MorisonPlate::compute_fluid_loads)
        .def_readwrite("load", &seahowl::hydro::MorisonPlate::load)
        .def_readwrite("diameter", &seahowl::hydro::MorisonPlate::diameter)
        .def_readwrite("drag_coefficient", &seahowl::hydro::MorisonPlate::drag_coefficient)
        .def_readwrite("reverse_direction", &seahowl::hydro::MorisonPlate::reverse_direction);

    // hydro/mooring_hydro.h
    py::class_<seahowl::hydro::MooringHydro, std::shared_ptr<seahowl::hydro::MooringHydro>>(m_hydro, "MooringHydro")
        .def(py::init<>())
        .def_readwrite("discretization_fractions", &seahowl::hydro::MooringHydro::discretization_fractions)
        .def_readwrite("diameter", &seahowl::hydro::MooringHydro::diameter)
        .def_readwrite("coefficients", &seahowl::hydro::MooringHydro::coefficients)
        .def_readwrite("loads", &seahowl::hydro::MooringHydro::loads)
        .def_readonly("nodes", &seahowl::hydro::MooringHydro::nodes)
        .def_readonly("elements", &seahowl::hydro::MooringHydro::elements)
        .def("build", &seahowl::hydro::MooringHydro::build)
        .def("compute_fluid_loads", &seahowl::hydro::MooringHydro::compute_fluid_loads);

    py::class_<seahowl::hydro::MooringSystemHydro, std::shared_ptr<seahowl::hydro::MooringSystemHydro>>(
        m_hydro, "MooringSystemHydro")
        .def(py::init<>())
        .def_readonly("moorings", &seahowl::hydro::MooringSystemHydro::moorings)
        .def("build", &seahowl::hydro::MooringSystemHydro::build)
        .def("compute_fluid_loads", &seahowl::hydro::MooringSystemHydro::compute_fluid_loads);

    // hydro/floater_hydro.h
    py::class_<seahowl::hydro::FloaterHydro, std::shared_ptr<seahowl::hydro::FloaterHydro>>(m_hydro, "FloaterHydro")
        .def(py::init<>())
        .def_property_readonly(
            "mooring_system", [](seahowl::hydro::FloaterHydro& floater) { return floater.mooring_system.get(); },
            py::return_value_policy::reference_internal)
        .def("compute_fluid_loads", &seahowl::hydro::FloaterHydro::compute_fluid_loads);
}
