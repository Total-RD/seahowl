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

#### Python Virtual Environment

Initialize the Python environment by following the steps below.

##### Linux
```bash
cd bin
./init_pyseahowl.sh
```

##### Windows
```bash
cd bin
./init_pyseahowl.bat
```

#### Activating the Virtual Environment

SEAHOWL uses an embedded Python in a virtual environment located in the `__env__` directory. To activate this virtual environment, follow the instructions for your operating system:

##### Linux

```bash
cd bin
source __env__/bin/activate
```

Add the build directory to your `PYTHONPATH` so that the Python executable can import `seahowl` from anywhere. Add the following line to your `.bashrc` (or equivalent file for your terminal) to ensure `seahowl` is found every time you open a new terminal:

```bash
export PYTHONPATH=/path/to/your/seahowl/build/directory:$PYTHONPATH
```

##### Windows

```bash
cd bin
.__env__/Scripts/activate
```

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
