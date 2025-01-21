# Installation

Clone the SEAHOWL repository:

```bash
git clone https://github.com/Total-RD/seahowl
cd seahowl
```

From here, there are three main ways to install SEAHOWL:

* [VCPKG install](#vcpkg-install) (recommended approach)
* [Bash install](#bash-install) (more involved process)
* Manual install (see [Dependencies](#dependencies) section to install them independently)

For additional (optional) modules that require extra post-install steps, see [Optional modules](#optional-modules) section.


## VCPKG install

This is the recommended approach for building and installing SEAHOWL.

### Installing vcpkg

#### Linux

First clone the vcpkg repository where you want, and then set it up as follows:

```bash
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
export VCPKG_ROOT=$(pwd)
```

For convenience, you can add the environment variables for vcpkg to your `.bashrc` file (adapt to your shell if needed, for example `.zshrc` for zsh) so it will be found when you open new terminals. To do so, run the following from within the vcpkg folder:

```bash
echo "export VCPKG_ROOT=$(pwd)" >> ~/.bashrc
```

If you have issues with installing vcpkg, see [here](external/vcpkg/README.md) or refer to the official vcpkg documentation for setting it up on your environment.

#### Windows

First clone the vcpkg repository where you want, and then set it up as follows:

```bash
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.bat
set VCPKG_ROOT=$(pwd)
```
For convenience, you can add the environment variables for vcpkg to your user environment variables so it will be found when you open new terminals.

If you have issues with installing vcpkg, see [here](external/vcpkg/README.md) or refer to the official vcpkg documentation for setting it up on your environment.

### Installing SEAHOWL

#### Linux

From within the `seahowl` folder that you cloned earlier, build and install SEAHOWL and its dependencies in a ``build`` folder as follows:

```bash
cmake --preset full
cmake --build build
```

Other presets are available, such as ``minimal`` for a minimal install with only essential dependencies, ``full_viz`` for an install including in situ visualization, ``full_vtk`` for an install with VTK output feature.

#### Windows
From within the `seahowl` folder that you cloned earlier, build and install SEAHOWL and its dependencies in a ``build`` folder as follows:

> **Note:** Ensure that Visual Studio is installed on your environment.

```bash
cmake -G "Visual Studio 17 2022" --preset full
MSBuild.exe ./build/SEAHOWL.sln /p:Configuration=Release
```

Other presets are available, such as ``minimal`` for a minimal install with only essential dependencies, ``full_viz`` for an install including in situ visualization, ``full_vtk`` for an install with VTK output feature.

### Installing a debug version of SEAHOWL


Adding a ``/debug`` suffix (e.g. ``full/debug``) to the presets will instead install a Debug version of the preset into the folder ``build/debug``. For a debug installation, the process becomes:

#### Linux

```bash
cmake --preset full/debug
cmake --build build/debug
```
#### Windows

```bash
cmake -G "Visual Studio 17 2022" --preset full/debug
MSBuild.exe ./build/SEAHOWL.sln /p:Configuration=Debug
```

### Troubleshooting

#### Linux

Extra system dependencies might be needed on some architectures for the build to go through. For example, some of these system packages will be required on Ubuntu (select as needed):

```bash
sudo apt build-essential cmake  # essential tools for building packages
sudo apt install pkg-config  # dependency of vcpkg
sudo apt install autoconf automake autoconf-archive  # for python vcpkg
sudo apt install libgl1-mesa-dev libxxf86vm-dev libglut-dev  # for irrlicht vcpkg
sudo apt install gfortran  # for OpenFAST vcpkg
```

Additionally, VTK was purposely omitted in the automated vcpkg installation process due to its compilation time, number of dependencies, and overall size.
It therefore has to be installed manually if the VTK dependency is activated in SEAHOWL. On Ubuntu, it can simply be added with:

```bash
sudo apt install libvtk9-dev  # if VTK is enabled as a dependency
```

## Bash install

> **Note:** Only for Linux environment.

### Installing dependencies

If going through the bash install process, several non-optional prerequisites must be installed on your system, including: Eigen3 (linear algebra library), nlohmann-json (JSON reader), spdlog (logging library) and Project Chrono (multibody and finite elements library). The development versions of Eigen3, nlohmann-json and spdlog can be easily installed on Ubuntu as follows:

```bash
sudo apt install libeigen3-dev nlohmann-json3-dev libspdlog-dev
```

Other optional system dependencies for a more complete install can be easily installed on Ubuntu as follows:

```bash
sudo apt install pybind11-dev  # for Python bindings
sudo apt libirrlicht-dev  # for in situ visualization capabilities
sudo apt gfortran libblas-dev liblapack-dev  # for OpenFAST modules (AeroDyn, InflowWind, etc)
sudo apt libvtk9-dev  # for VTK output feature
```

Dependencies that cannot be typically handled through the package manager of the system (e.g. Chrono, HydroChrono, etc.) are automatically installed into an ``install`` folder using the following command:

```bash
cd external/bash
./dep-install.sh
cd ../..
```
See [here](external/bash/README.md) for more details about the bash install process and commands.

### Installing SEAHOWL

Once the dependencies have been installed, the usual cmake workflow can be used to build SEAHOWL. Create a build folder:

```bash
mkdir build
cd build
```

Build the project:

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
make
```


## Dependencies

### Core dependencies

- Chrono (8.0.0): https://github.com/projectchrono/chrono
- nlohmann-json (v3.10.5): https://github.com/nlohmann/json
- spdlog (v1.12.0): https://github.com/gabime/spdlog


### Optional dependencies

#### Physics

- AeroDyn: https://github.com/Total-RD/openfast4seahowl
- InflowWind: https://github.com/Total-RD/openfast4seahowl
- HydroChrono (v0.2.4): https://github.com/NREL/HydroChrono

#### Documentation

- Doxygen (Release_1_8_20): https://github.com/doxygen/doxygen
- Graphviz (7.0.4): https://graphviz.org/
- Sphinx (v5.3.0): https://github.com/sphinx-doc/sphinx

#### Visualization

- VTK (v9.2.0): https://gitlab.kitware.com/vtk/vtk
- Irrlicht (1.8.4): https://irrlicht.sourceforge.io/

#### Tests

- GoogleTest (release-1.12.1): https://github.com/google/googletest

#### Python bindings

- pybind11 (Version 2.10.4): https://github.com/pybind/pybind11


## Optional modules

### Python bindings

Build SEAHOWL with ``SEAHOWL_ENABLE_PYTHON`` as ``ON``. SEAHOWL use a embeded Python in a virtual environement in the directory ``__env__``, to activate this virtual environement :

#### Linux

```bash
 source source __env__/bin/activate
```
You can then add the build directory to your ``PYTHONPATH`` so that the Python executable used to build the bindings can ``import seahowl`` from anywhere.
Adding the following line to your .bashrc (or equivalent file for your favorite terminal) ensures that seahowl will be found everytime you open a new terminal:

```bash
export PYTHONPATH=/path/to/your/seahowl/build/directory:$PYTHONPATH
```
#### Windows

```bash
 .__env__/Script/activate
```
You can then add the build directory to your ``PYTHONPATH`` so that the Python executable used to build the bindings can ``import seahowl`` from anywhere.

```bash
set PYTHONPATH=/path/to/your/seahowl/build/directory;%PYTHONPATH%
```
For convenience, you can add the environment variables PYTHONPATH to your user environment variables.

### Documentation

#### Linux

To build the Doxygen documentation, the following needs to be installed on your environment:
```bash
sudo apt install doxygen graphviz
```
#### Windows

To install Doxygen on a Windows environment, you can follow these steps:

- Step 1: Download Doxygen
    Go to the Doxygen download page (https://www.doxygen.nl/download.html).
    Download the Windows installer
- Step 2: Install Doxygen
    Run the downloaded installer.
    Follow the installation instructions. You can choose the default options.
- Step 3: Add Doxygen to the System Path
- Step 4: Install Graphviz (Optional, but recommended for generating diagrams)
    Go to the Graphviz download page.
    Download the Windows installer (graphviz-<version>.msi).
    Run the downloaded installer and follow the installation instructions.
    Add the Graphviz bin directory to the system path (similar to how you added Doxygen).

### Delivery

To create a delivery version of SEAHOWL, you can use the automatic install process, which installs the necessary files into the `install` directory.

#### Linux

```bash
cd build
ninja install
```
#### Windows

```bash
cd build
cmake --install . --config Release
```
