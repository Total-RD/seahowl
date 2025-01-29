# SEAHOWL: Servo-Elasto-Aero-Hydro Offshore Wind Lab

SEAHOWL is a time domain multi-physics simulation framework for onshore, offshore, and floating wind turbines.

## Usage

### Using the Driver

To run SEAHOWL using the driver, execute the following command:

```bash
./bin/seahowl_driver data/IEA15MW/main.json
```

An `output` folder containing all the outputs will be automatically created.

### Using Python

#### Python Version

Python version 3.11.10 is required.

Add the build directory to your `PYTHONPATH` so that the Python executable can import `seahowl` from anywhere.

##### Linux

Add the following line to your `.bashrc` (or equivalent file for your terminal) to ensure `seahowl` is found every time you open a new terminal:

```bash
export PYTHONPATH=/path/to/your/seahowl/build/directory:$PYTHONPATH
```

##### Windows

Add the build directory to your `PYTHONPATH` so that the Python executable can import `seahowl` from anywhere:

```bash
set PYTHONPATH=/path/to/your/seahowl/build/directory;%PYTHONPATH%
```

For convenience, you can add the `PYTHONPATH` environment variable to your user environment variables.

#### Example

To use SEAHOWL in Python, you can follow this example:

```python
import seahowl

# Create a simulation object
simulation = seahowl.core.Simulation()
simulation.populate_from_file("data/IEA15MW/main.json")
simulation.initialize_from_config()

# Simulation loop
while simulation.system_core.get_time() < simulation.duration:
    simulation.step()
```
