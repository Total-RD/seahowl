#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

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
}
