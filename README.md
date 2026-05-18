# SEAHOWL: Servo-Elasto-Aero-Hydro Offshore Wind Lab

SEAHOWL is a time-domain multi-physics simulation framework written in C++ and primarily developed for wind turbine analysis.

Main features of SEAHOWL:

- **Monolithic coupling of structural dynamics** through Project Chrono for multibody and finite element problems
- **Partitioned coupling for multi-physics** interaction for easy switching between individual physics solver
- **Modular by design**, easily adaptable for innovative or non-conventional aeroelastic applications
- **Extensive Python bindings** for controlling the simulation workflow and interact with subcomponents at runtime


## Installation

- For precompiled binaries (including Python bindings), check the Releases section of this project on GitHub to see if a zipped version is available for your OS.
- For the full installation and compilation process, refer to [INSTALL.md](INSTALL.md).


## Repository Structure

```
seahowl/
├── src/                          # Source files (.cpp)
│   ├── core/                     # Turbine components, simulation orchestration
│   ├── elasto/                   # Elastodynamics (multibody, finite elements, etc)
│   ├── fluid/
│   │   ├── aero/                 # Aerodynamics (BEMT, actuator disk, etc)
│   │   └── hydro/                # Hydrodynamics (Morison, potential flow, etc)
│   ├── env/                      # Environmental models (wind, waves, soil, etc)
│   ├── servo/                    # Servodynamics (control systems, DISCON)
│   ├── io/                       # I/O, configuration, output management
│   ├── commons/                  # Shared utilities and base classes
│   └── bindings/python/          # pybind11 Python bindings
│
├── include/seahowl/              # Header files (.h) - mirrors src/ structure
│
├── data/                         # Reference turbine configurations
│   └── IEA15MW/                  # IEA 15MW reference turbine
│       ├── onshore/
│       ├── monopile/
│       └── floating/
│
├── tests/
│   ├── unit_tests/               # C++ unit tests
│   └── non_regression/           # Python regression tests
│
├── examples/
│   ├── cpp/                      # C++ examples
│   └── python/                   # Python examples
│
└── external/                     # External dependencies
```


## Usage

### Using the driver

For example, if you are in the root directory of this repository and compiled the SEAHOWL driver in a `build` folder, you can run:

```bash
./build/seahowl_driver ./data/IEA15MW/main_onshore.json
```

An `output` folder containing all the outputs will be automatically created.

> [!NOTE]
> Copy the libdiscon.so (or dll) library for ROSCO >2.8.x into location define in 
> block **controller** of **turbine.json** file. 
> Usually into subdirectory **controller** of the test case.

### Using Python bindings

If you compiled the Python bindings and added them to your `PYTHONPATH`, you can use SEAHOWL as follows:

```python
import seahowl

# make simulation object
simulation = seahowl.core.Simulation()
simulation.populate_from_file("data/IEA15MW/main_onshore.json")
simulation.initialize_from_config()

# simulation loop
while simulation.system_core.get_time() < simulation.duration:
    simulation.step()
```

#### Adding sensors

[scripts/sensors.py](scripts/sensors.py) provides sensor classes to measure physical quantities at any spanwise location along an FEA component (`fraction=0` at root/bottom, `fraction=1` at tip/top). Sensors are callable and return a `numpy` array. They integrate naturally with the CSV output system:

```python
import seahowl
import sys
sys.path.append("scripts")
import sensors

simulation = seahowl.core.Simulation()
simulation.populate_from_file("data/IEA15MW/onshore/main.json")
simulation.initialize_from_config()

turbine = simulation.system_core.turbines[0]

# create sensors on turbine components
tower_mid_pos   = sensors.PositionGaugeFEA(turbine.elasto.tower, fraction=0.5)
blade_root_mom  = sensors.MomentGaugeFEA(turbine.elasto.rna.rotor.blades[0], fraction=0.0)
tower_top_accel = sensors.AccelerometerFEA(turbine.elasto.tower, fraction=1.0)

# log to CSV — sensors are directly callable
mycsv = simulation.outputs.create_new_csv("sensors_output.csv")
mycsv.add_function("time [s]",              lambda: simulation.system_core.get_time())
mycsv.add_function("tower mid position [m]", tower_mid_pos)
mycsv.add_function("blade root moment [Nm]", blade_root_mom)
mycsv.add_function("tower top accel [m/s2]", tower_top_accel)

# simulation loop
while simulation.system_core.get_time() < simulation.duration:
    simulation.step()
```

Other examples of Python bindings usage are available in `examples/python/` from the root of SEAHOWL's repository.


## How to cite

Reference paper on SEAHOWL [available in open access here:](https://iopscience.iop.org/article/10.1088/1742-6596/2767/5/052051) de Lataillade *et al* 2024 *J. Phys.: Conf. Ser.* **2767** 052051.
