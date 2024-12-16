# SEAHOWL: Servo-Elasto-Aero-Hydro Offshore Wind Lab

SEAHOWL is a time domain multi-physics simulation framework for onshore, offshore, and floating wind turbines.

## Installation

Clone the SEAHOWL repository:

```bash
git clone https://github.com/Total-RD/seahowl
cd seahowl
```

Install SEAHOWL and all its dependencies in a `build` folder:

```bash
cmake --preset full
cmake --build build
```

For more details about the installation process, see [INSTALL.md](INSTALL.md).

## Usage

### Using the driver

For example, if you are in the root directory of this repository and compiled the SEAHOWL driver in a `build` folder, you can run:

```bash
./build/seahowl_driver ./data/IEA15MW/main.json
```

An `output` folder containing all the outputs will be automatically created.


### Using Python bindings

If you compiled the Python bindings and added them to your `PYTHONPATH`, you can use SEAHOWL as follows:

```python
import seahowl

# make simulation object
simulation = seahowl.core.Simulation()
simulation.populate_from_file("data/IEA15MW/main.json")
simulation.initialize_from_config()

# simulation loop
while simulation.system_core.get_time() < simulation.duration:
    simulation.step()
```

Other examples of Python bindings usage are available in [examples/python/](examples/python/)
