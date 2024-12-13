############
Installation
############


******************
Installing SEAHOWL
******************


Quick install
=============

After installing the `prerequisites`_ (and optional `dependencies`_, if any), clone the SEAHOWL repository:

.. code-block:: bash

   git clone https://github.com/Total-RD/seahowl
   cd seahowl

Make a directory where the library will be built:

.. code-block:: bash

   mkdir build
   cd build

Use cmake to configure :

.. code-block:: bash

   cmake .. --preset "full"
   make


.. _prerequisites:

Prerequisites
=============

Non-optional prerequisites include Eigen3 (linear algebra library), nlohmann-json (JSON reader), spdlog (logging library) and Project Chrono (multibody and finite elements library).

The development versions of Eigen3, nlohmann-json and spdlog can be easily installed on Ubuntu as follows:

.. code-block:: bash

   sudo apt install libeigen3-dev nlohmann-json3-dev libspdlog-dev

The supported version of Project Chrono for SEAHOWL is 8.0.0. To install used the following commands:

.. code-block:: bash

   cd external
   ./dep-install.sh -d chrono

This script build the Chrono library.


.. _dependencies:

Dependencies
============

Core dependencies
-----------------

- Chrono (8.0.0): https://github.com/projectchrono/chrono
- nlohmann-json (v3.10.5): https://github.com/nlohmann/json
- spdlog (v1.12.0): https://github.com/gabime/spdlog


Optional dependencies
---------------------

Physics
^^^^^^^

- AeroDyn: https://github.com/Total-RD/openfast4seahowl
- InflowWind: https://github.com/Total-RD/openfast4seahowl
- HydroChrono (v0.2.4): https://github.com/NREL/HydroChrono

Documentation
^^^^^^^^^^^^^

- Doxygen (Release_1_8_20): https://github.com/doxygen/doxygen
- Graphviz (7.0.4): https://graphviz.org/
- Sphinx (v5.3.0): https://github.com/sphinx-doc/sphinx

Visualization
^^^^^^^^^^^^^

- VTK (v9.2.0): https://gitlab.kitware.com/vtk/vtk
- Irrlicht (1.8.4): https://irrlicht.sourceforge.io/

Tests
^^^^^

- GoogleTest (release-1.12.1): https://github.com/google/googletest

Python bindings
^^^^^^^^^^^^^^^

- pybind11 (Version 2.10.4): https://github.com/pybind/pybind11




***************************
Installing optional modules
***************************

Python bindings
===============

Add pybind11 to your current python installation.

.. code-block:: bash

   pip install pybind11

Go into your build directory and enable python bindings option:

.. code-block:: bash

   cd build
   cmake .. -DSEAHOWL_ENABLE_PYTHON=ON
   make

Test your installation by opening a terminal (in the build directory):

.. code-block:: python

   import seahowl
   system_elasto = seahowl.elasto.SystemElastoChrono()
   system_aero = seahowl.aero.SystemAero()
   system_core = seahowl.core.System(system_elasto, system_aero)

You can add the build directory to your `PYTHONPATH` in order to use pyseahowl anywhere.
Adding the following line to your .bashrc (or equivalent file for your favorite terminal) will ensure that pyseahowl will be usable everytime you open a new terminal:

.. code-block:: bash

   export PYTHONPATH=/path/to/your/seahowl/build/directory:$PYTHONPATH
