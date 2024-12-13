############
Installation
############


******************
Installing SEAHOWL
******************


VCPKG Install
=============

This is the recommended approach for installing SEAHOWL. Clone the repository:

.. code-block:: bash

   git clone https://github.com/Total-RD/seahowl
   cd seahowl

Set up vcpkg on your environment.
Build and install SEAHOWL (and its dependencies) in a ``./build`` folder:

.. code-block:: bash

   cmake --preset full
   cmake --build --preset full

Some dependencies might be needed on some architectures for the build to go through. For example, on Ubuntu (select as needed):

.. code-block:: bash

  sudo apt-get build-essential  # essential tools for building packages
  sudo apt-get install pkg-config  # dependency of vcpkg
  sudo apt-get install autoconf automake autoconf-archive  # for python vcpkg
  sudo apt-get install libgl1-mesa-dev libxxf86vm-dev libglut-dev  # for irrlicht vcpkg
  sudo apt-get install gfortran  # for OpenFAST vcpkg
  sudo apt-get install vtk9  # if VTK is enabled as a dependency


Bash Install
==============

Non-optional prerequisites include Eigen3 (linear algebra library), nlohmann-json (JSON reader), spdlog (logging library) and Project Chrono (multibody and finite elements library).

The development versions of Eigen3, nlohmann-json and spdlog can be easily installed on Ubuntu as follows:

.. code-block:: bash

   sudo apt install libeigen3-dev nlohmann-json3-dev libspdlog-dev

Project Chrono can be installed automatically into an ``./install`` folder using the following commands:

.. code-block:: bash

   cd external
   ./external/bash/dep-install.sh -d chrono

Other optional dependencies for a more complete install can be easily installed on Ubuntu as follows:

.. code-block:: bash

   sudo apt install pybind11-dev libblas-dev liblapack-dev libirrlicht-dev vtk9

Once this is done, the usual cmake process can be used to build the process. First create a build folder:

.. code-block:: bash

   mkdir build
   cd build

Then build the project:

.. code-block:: bash

   cmake .. -DCMAKE_INSTALL_PREFIX=../install
   make


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

   pip install pybind11[global]

Go into your build directory and enable python bindings option:

.. code-block:: bash

   cd build
   cmake .. -DSEAHOWL_ENABLE_PYTHON=ON
   make

Test your installation by opening a terminal (in the build directory):

.. code-block:: python

   import seahowl

You can add the build directory to your `PYTHONPATH` in order to use seahowl from anywhere in Python.
Adding the following line to your .bashrc (or equivalent file for your favorite terminal) will ensure that seahowl will be found everytime you open a new terminal:

.. code-block:: bash

   export PYTHONPATH=/path/to/your/seahowl/build/directory:$PYTHONPATH
