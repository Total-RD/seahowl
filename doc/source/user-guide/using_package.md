# Using SEAHOWL package

The precompiled SEAHOWL package can be found in the Releases section on GitHub as a zip file, if available for your OS. Otherwise, after successfully compiling SEAHOWL, it can be created with `ninja install` (if you compiled SEAHOWL through VCPKG) or `make install` (e.g. when using bash install process). It will create an `install` folder with all binaries and libraries at the root of the project.


## Using the Driver

To run SEAHOWL using the driver, execute the following command:

```bash
./bin/seahowl_driver data/IEA15MW/main_onshore.json
```

An `output` folder containing all the outputs will be automatically created.

## Using Python

SEAHOWL uses an embedded Python in a virtual environment located in the `.venv` directory. To activate this virtual environment, follow the instructions for your operating system:

##### Linux

```bash
source bin/init_pyseahowl.sh
```
##### Windows

If using Command Prompt (cmd):
```bash
./bin/init_pyseahowl.bat
```

If using PowerShell:
```bash
./bin/init_pyseahowl.ps1
```

#### Example

To use SEAHOWL in Python, you can follow this example:

```python
import seahowl

# Create a simulation object
simulation = seahowl.core.Simulation()
simulation.populate_from_file("data/IEA15MW/main_onshore.json")
simulation.initialize_from_config()

# Simulation loop
while simulation.system_core.get_time() < simulation.duration:
    simulation.step()
```
