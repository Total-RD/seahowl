API Reference
=============

This section provides detailed API documentation for SEAHOWL.

.. toctree::
   :maxdepth: 2

   cpp
   python

Overview
--------

SEAHOWL provides two API interfaces:

**C++ API**
   The native C++ interface provides full access to all SEAHOWL functionality.
   Documentation is generated via Doxygen.

**Python API**
   Python bindings via pybind11 expose the core functionality for scripting
   and rapid prototyping.

Module Organization
-------------------

Both APIs follow the same module structure:

.. list-table::
   :widths: 20 80
   :header-rows: 1

   * - Module
     - Description
   * - ``core``
     - Simulation orchestration, turbine components (Simulation, System, Turbine, Blade, Tower, Rotor)
   * - ``elasto``
     - Structural dynamics via Project Chrono (beams, rigid bodies, constraints)
   * - ``aero``
     - Aerodynamics (aero components, BEMT solver, AeroDyn interface, airfoil data)
   * - ``hydro``
     - Hydrodynamics (hydro components, Morison equation, HydroChrono interface)
   * - ``env``
     - Environmental models (wind, waves, soil)
   * - ``servo``
     - Control systems (DISCON interface, actuators)
   * - ``io``
     - Input/output (JSON parsing, CSV/VTK output)
   * - ``commons``
     - Shared utilities and base classes

Quick Links
-----------

- :doc:`cpp` - C++ API documentation (Doxygen)
- :doc:`python` - Python API documentation (autosummary)
