# SEAHOWL: Servo-Elasto-Aero-Hydro Offshore Wind Lab

SEAHOWL is a time domain multi-physics simulation framework for onshore, offshore, and floating wind turbines.


## Usage

### Using the driver

For example, if you are in the root directory of this repository and compiled the SEAHOWL driver in a `./build` directory, you can run:

```bash
./build/seahowl_driver ./data/IEA15MW/main.json
```

An `output` folder containing all the outputs will be automatically created.


### Using python bindings

If you compiled the python bindings and added them to your `PYTHONPATH`, you can use SEAHOWL as follows:

```python
import pyseahowl

# make system
system_elasto = pyseahowl.elasto.SystemElastoChrono()
system_aero = pyseahowl.aero.SystemAero()
system_core = pyseahowl.core.System(system_elasto, system_aero)

# populate and initialize system from json file
filepath = "./data/IEA15MW/main.json"
pyseahowl.io.populate_system_from_json(filepath, system_core)
pyseahowl.io.initialize_system_from_json(filepath, system_core)

# run simulation loop
dt = 0.05
t_sim = 0.
while system_core.get_time() < 200:
    # make a time step
    system_core.prestep(t_sim, dt)
    system_core.step(dt)
    system_core.poststep(t_sim, dt)

    t_sim = system_core.get_time()

    # print info about turbine
    print("time: {t_sim:.3f}, rpm: {rpm:.3f}, pitch: {pitch:.3f}".format(
        t_sim=t_sim,
        rpm=system_core.turbines[0].rna.elasto.get_rpm(),
        pitch=system_core.turbines[0].rna.elasto.rotor.pitch_collective,
    ))
```
