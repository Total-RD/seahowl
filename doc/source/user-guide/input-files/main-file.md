# Main Input File

The main input file is the entry point for SEAHOWL simulations. It defines simulation parameters, output settings, environmental conditions, and the list of turbines.

## File Structure

```
main.json
│
├── numerics/                              # Simulation parameters
│   ├── dt: float                          # Time step [s]
│   ├── duration: float                    # Total simulation duration [s]
│   │
│   ├── statics/                           # Static equilibrium solver
│   │   ├── linear_step: bool              # Enable linear statics step
│   │   └── nonlinear_steps: int           # Number of nonlinear iterations
│   │
│   └── presimulation/                     # Pre-simulation phase
│       ├── dt: float                      # Pre-sim time step [s]
│       ├── duration: float                # Pre-sim duration [s]
│       ├── presetup: bool                 # Gradual activation of loads
│       └── fix_towers: bool               # Fix tower bases during pre-sim
│
├── outputs/                               # Output configuration
│   ├── dt: float                          # Output interval [s]
│   ├── folder: string                     # Output directory path
│   ├── vtk: bool                          # Enable VTK output
│   ├── log_level: string                  # Logging verbosity
│   └── gui: bool                          # Enable real-time visualization
│
├── environment/
│   └── file: string                       # Path to environment file
│
└── turbines[]                             # Array of turbines
    ├── file: string                       # Path to turbine file
    ├── translation: [x, y, z]             # Position offset [m]
    └── rotation: float                    # Z-axis rotation [deg]
```

## Field Reference

### numerics

Core simulation parameters.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `dt` | float | s | Simulation time step |
| `duration` | float | s | Total simulation duration |

### numerics.statics

Static equilibrium solver configuration. Run before dynamic simulation to find initial equilibrium.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `linear_step` | bool | - | Perform linear static analysis |
| `nonlinear_steps` | int | - | Number of nonlinear Newton iterations |

### numerics.presimulation

Optional pre-simulation phase for gradual load application.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `dt` | float | s | Time step during pre-simulation |
| `duration` | float | s | Duration of pre-simulation (0 = disabled) |
| `presetup` | bool | - | Gradually apply loads (mooring pretension, blade damping) |
| `fix_towers` | bool | - | Fix tower bases during pre-simulation |

### outputs

Output file configuration.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `dt` | float | s | Time interval for outputs (0 = all time steps) |
| `folder` | string | - | Directory for output files |
| `vtk` | bool | - | Generate VTK visualization files |
| `log_level` | string | - | Log verbosity (see below) |
| `gui` | bool | - | Enable real-time 3D visualization |

**Log levels** (from least to most verbose): `critical`, `error`, `warning`, `info`, `debug`, `trace`

### environment

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `file` | string | - | Path to environment file (relative to this file) |

### turbines[]

Array of turbine configurations. Each entry creates one turbine instance.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `file` | string | - | Path to turbine file (relative to this file) |
| `translation` | [float, float, float] | m | Turbine position offset |
| `rotation` | float | deg | Rotation about Z-axis |


## Usage Notes

### Pre-simulation Phase

Use pre-simulation (`presimulation.duration > 0`) to:
- Apply initial conditions during the duration of the presimulation
- Allow moorings to reach target length and equilibrium tension
- Achieve a smooth start to the main simulation

### Multiple Turbines

Multiple turbines can be simulated by adding entries to the `turbines` array:

```json
"turbines": [
  {
    "file": "./turbine.json",
    "translation": [0, 0, 0],
    "rotation": 0
  },
  {
    "file": "./turbine.json",
    "translation": [1000, 0, 0],
    "rotation": 0
  }
]
```
