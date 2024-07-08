#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <seahowl/hydro/morison.h>
#include <seahowl/env/fluid_models.h>
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
        .def_readwrite("drag_tangent", &seahowl::hydro::HydroCoefficients::drag_tangent)
        .def_readwrite("added_mass_normal", &seahowl::hydro::HydroCoefficients::added_mass_normal)
        .def_readwrite("added_mass_tangent", &seahowl::hydro::HydroCoefficients::added_mass_tangent)
        .def_readwrite("has_buoyancy", &seahowl::hydro::HydroCoefficients::has_buoyancy)
        .def_readwrite("has_inertia", &seahowl::hydro::HydroCoefficients::has_inertia);
    py::class_<seahowl::hydro::MorisonNode, std::shared_ptr<seahowl::hydro::MorisonNode>, seahowl::EntityDynamicEigen>(
        m_hydro, "MorisonNode")
        .def(py::init<>())
        .def("compute_loads", &seahowl::hydro::MorisonNode::compute_loads)
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
}
