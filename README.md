# SEAHOWL: Servo-Elasto-Aero-Hydro Offshore Wind Lab

SEAHOWL is a time domain multi-physics wind turbine simulation platform.

## Installation

### Compilation

From a directory `./build`:

```bash
cmake .. -DChrono_DIR=/path/to/your/chrono/cmake/build/directory
make
```

If compiling with AeroDyn:
```bash
cmake .. -DChrono_DIR=/path/to/your/chrono/cmake/build/directory -DAERODYN_LIBRARY_RELEASE=/path/to/your/libaerodyn_inflow_c_binding.so
make
```


#### Default CMAKE Options:

```cmake
option(SEAHOWL_ENABLE_IRRLICHT "Enable Irrlicht 3D visualization library" OFF)
option(SEAHOWL_ENABLE_TESTS "Enable tests" OFF)
option(SEAHOWL_ENABLE_DOC "Generate html documentation" OFF)
option(SEAHOWL_ENABLE_BUILD "Build library and drivers" ON)
option(SEAHOWL_ENABLE_PYTHON "Enable python binding" OFF)
option(SEAHOWL_ENABLE_EXAMPLES "Enable examples" ON)
option(SEAHOWL_ENABLE_ROSCO "Enable ROSCO controller" ON)
option(SEAHOWL_ENABLE_VTK "Enable VTK Library for output" OFF)
option(SEAHOWL_ENABLE_AERODYN "Enable AeroDyn module" OFF)
```


### Dependencies

#### Core

- Chrono (8.0.0): https://github.com/projectchrono/chrono
- nlohmann-json (v3.10.5): https://github.com/nlohmann/json
- ROSCO (v2.5.0): https://github.com/NREL/ROSCO
- AeroDyn: https://github.com/Total-RD/aerodyn4seahowl

#### Documentation

- Doxygen (Release_1_8_20): https://github.com/doxygen/doxygen
- Graphviz (7.0.4): https://graphviz.org/
- Sphinx (v5.3.0): https://github.com/sphinx-doc/sphinx

#### Visualization

- VTK (v9.2.0): https://gitlab.kitware.com/vtk/vtk
- Irrlicht (1.8.4): https://irrlicht.sourceforge.io/

#### Tests

- GoogleTest (release-1.12.1): https://github.com/google/googletest



## Usage

### Using the driver

```bash
[path/to/seahowl_driver] [path/to/main/file/json]
```

For example, if from this repository your compiled driver is in a `build` folder, you can run the given example:
```bash
./build/seahowl_driver ./data/IEA15MW_main.json
```


### Input files

The simulation of a wind turbine can be entirely piloted from input files without recompilation of the code needed.
For examples of input files, look at the `./data/` folder with files based on the reference IEA15MW turbine properties.


#### Main file (main.json)

The main JSON file pilots numerical options, environmental conditions, and points to turbine files used for the simulation.
It is a JSON dictionary containing:

- numerics: (dict)
  - **dt**: (float) the time stepping value for the simulation [s].
  - **t_end**: (float) the ending time of the simulation [s].
  - **aerodyn**: (bool) option to enable aerodyn module. In-house BEMT will be used if this option is disable.
- outputs: (dict)
  - **dt**: (float) the time stepping value for outputs of the simulation [s].
  - **VTK**: (bool) whether VTK will be part of outputs or not.
- environment: (dict)
  - **gravity**: (array of floats length 3) gravitational acceleration [m/s2].
  - **air_density**: density of air [kg/m3].
  - **wind**: options for wind model.
- turbines: (list of dict)
  - **translation**: (float array of length 3) translation of turbine in space [m].
  - **rotation**: rotation of turbine (yaw) [°].
  - **file**: file path of turbine file (relative to this file path).
  - **use_aerodyn**: whether to use AeroDyn or not for this turbine.
  - **file_aerodyn**: path to AeroDyn .dat input file (only used if use_aerodyn is true).
  - **file_inflowwind**: path to InflowWind .dat input file (only used if use_aerodyn is true).

#### Available Wind Models

- Wind ramp:
```json
{
  "type": "ramp",
  "options": {
    "reference_height": 150,
    "shear_coefficient": 0.12,
     "velocity_start": [12, 0, 0],
     "velocity_stop": [25, 0, 0],
     "time_start": 500,
     "time_stop": 1700   
  }
}
```

#### Turbine file (turbine.json)

The turbine JSON file pilots the discretization options of the blades and tower, as well as other general options.
For discretization of blades and tower, it is possible to either use an ordered array of floats between 0 and 1 (with 0 and 1 included in the array as bounds) corresponding to the normalized abscissa of the reference points or only one integer corresponding to the number of elements to use for discretization.
It is a JSON dictionary containing:
- blades: (dict)
  - **fpm**: (bool) whether to consider Fully-Populated Matrix (FPM) elements (6x6 material properties) or not.
  - **file_aerodyn**: file path of aerodyn file (relative to this file path), only used if aerodyn option is true in turbine file.
  - **discretization**: (dict)
    - **elasto**: (array of floats) discretization fractions (between 0 and 1) for elasto part of blade.
    - **aero**: (array of floats) discretization fractions (between 0 and 1) for aero part of blade.
  - **blades**: (list)
    - **file**: file path of blade file (relative to this file path).
    - **initial_pitch**: initial pitch of blade [°].
- rna: (dict)
  - **initial_pitch_collective**: initial collective pitch of blades [°].
  - **file**: file path of RNA file (relative to this file path).
- tower: (dict)
  - **discretization**: (dict)
    - **elasto**: (array of floats) discretization fractions (between 0 and 1) for elasto part of blade.
    - **aero**: (array of floats) discretization fractions (between 0 and 1) for aero part of blade.
  - **file**: file path of tower file (relative to this file path).
- controller: (dict)
  - **type**: (string) type of controller.
  - **options**: (dict) options of controller.


 #### Rotor-Nacelle Assembly file (rna.json)
 
 The Rotor-Nacelle Assembly (RNA) JSON file describes everything related to the rotor, nacelle, drivetrain, generator, gearbox.
 It is a JSON dictionary containing:
- **shaft**: (dict)
  - **tilt**: (float) tilt of shaft [°].
  - **distance_from_towertop**: distance of shaft frol towertop [m].
- **nacelle**: (dict)
  - **inertia**: (float) inertia of nacelle [kg.m2].
  - **mass**: (float) mass of nacelle [kg]
  - **CM**: (array of floats of length 3) center of mass offset from towertop [m].
  - **yaw_bearing_mass**: mass of yaw bearing [kg].
- **drivetrain**: (dict)
  - **generator_efficiency**: (float) generator efficiency [%].
  - **generator_inertia**: (float) generator inertia [kg.m2].
  - **gearbox_ratio**: (float) gearbox ratio [/].
  - **gearbox_efficiency**: (float) geabox efficiency [%].
- **hub**:
  - **radius**: (float) radius of hub [m].
  - **overhang**: (float) overhang of hub [m].
  - **inertia**: (float) inertia of hub [kg.m2].
  - **mass**: (float) mass of hub [kg].
  - **CM**: (float) offset of center of mass of hub [m].
- **precones**: (array of floats) precone of blades [°].


#### Blade file (blade.json)

The blade JSON file is a reference file that should be defined only once per blade type and not be changed by the user (unless the blade properties themselves change).
All the options that can change per simulation such as discretization should be defined in the turbine JSON file.
Only *reference* points are defined in this file, which does not have to be defined at the same coordinates as the chosen numerical discretization during the simulation.
It is a json dictionary containing:
- **damping_coefficients**: (array of floats of length 4) damping coefficients of blade.
- **reference_points**: (list of dict) list of reference points.
  - **coordinates**: (array of floats of length 3) coordinates of reference point (IEC standard) [m].
  - **twist**: (float) structural twist of blade at reference point [°]. 
  - **mass_matrix**: (6x6 matrix of floats) mass matrix of blade at reference point [kg].
  - **stiffness_matrix**: (6x6 matrix of floats) stiffness matrix of blade at reference point [N/m].
  - **chord**: (float) chord length of blade at reference point [m].
  - **airfoil_file**: (string) file path of airfoil file (relative to this file path).


#### Tower file (tower.json)

The tower JSON file is a reference file that should be defined only once per tower type and not be changed by the user (unless the tower properties themselves change).
All the options that can change per simulation such as discretization should be defined in the turbine JSON file.
Only *reference* points are defined in this file, which does not have to be defined at the same coordinates as the chosen numerical discretization during the simulation.
It is a json dictionary containing:
- **height**: (float) absolute height of towertop [m].
- **base_height**: (float) absolute height of towerbase [m].
- **damping_coefficients**: (array of floats of length 4) damping coefficients of blade.
- **reference_points**: (list of dict) list of reference points.
  - **fraction**: (float) normalized abscissa along tower (starting from base) of reference point.
  - **density**: (float) density of tower at reference point [kg/m3]. 
  - **stiffness_foreaft**: (float) fore-aft stiffness of tower at reference point [N/m].
  - **stiffness_sideside**: (float) side-side stiffness of tower at reference point [N/m].
  - **diameter**: (float) diameter of tower at reference point [m].
  - **drag_coefficient**: (float) drag coefficient of tower at reference point [/].
