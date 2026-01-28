# Turbine Input File

The turbine file defines the overall turbine configuration, including rotor type, discretization, controller settings, and references to component files.

## File Structure

```
turbine.json
│
├── aero/                                  # Aerodynamics configuration
│   ├── solver: string                     # "BEMT" or "AeroDyn"
│   └── options/                           # Solver-specific options
│
├── rotor/                                 # Rotor configuration
│   ├── type: string                       # "fea", "fpm", "rigid", or "disk"
│   ├── discretization/
│   │   ├── elasto: array                  # Structural discretization
│   │   └── aero: array                    # Aerodynamic discretization
│   ├── pitch_actuator_dynamics: bool      # Include pitch dynamics
│   └── blades[]                           # Blade definitions
│       ├── file: string                   # Path to blade file
│       ├── initial_pitch: float           # Initial pitch angle [deg]
│       └── precone: float                 # Precone angle [deg]
│
├── rna/                                   # Rotor-nacelle assembly
│   ├── file: string                       # Path to RNA file
│   ├── initial_yaw: float                 # Initial yaw angle [deg]
│   └── yaw_actuator_dynamics: bool        # Include yaw dynamics
│
├── tower/                                 # Tower configuration
│   ├── file: string                       # Path to tower file
│   ├── discretization/
│   │   ├── elasto: array                  # Structural discretization
│   │   └── aero: array                    # Aerodynamic discretization
│   └── options/
│       ├── use_MacCamyFuchs_correction: bool
│       └── use_Cd_correction: bool
│
├── controller/                            # Control system
│   ├── type: string                       # "", "DISCON", or "RPM"
│   └── options/                           # Controller-specific options
│
└── foundation/                            # (optional) Foundation type
    ├── type: string                       # "floater" or "monopile"
    └── file: string                       # Path to foundation file
```

## Field Reference

### aero

Aerodynamics solver configuration.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `solver` | string | - | Aerodynamics solver: `"BEMT"` or `"AeroDyn"` |

#### aero.options (BEMT)

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `hub_loss` | bool | - | Enable Prandtl hub loss correction |
| `tip_loss` | bool | - | Enable Prandtl tip loss correction |
| `tower_shadow` | bool | - | Enable tower shadow model |

#### aero.options (AeroDyn)

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `file` | string | - | Path to AeroDyn input file |

### rotor

Rotor configuration including blade type and discretization.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `type` | string | - | Rotor model: `"fea"`, `"fpm"`, `"rigid"`, `"disk"` |
| `pitch_actuator_dynamics` | bool | - | Include blade pitch actuator dynamics |

**Rotor types:**
- `fea` - Finite element blades with diagonal mass/stiffness matrices
- `fpm` - Fully-populated 6x6 mass/stiffness matrices (coupled modes)
- `rigid` - Rigid blades (no flexibility)
- `disk` - Actuator disk model (no individual blades)

### rotor.discretization

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `elasto` | array | - | Structural element discretization |
| `aero` | array | - | Aerodynamic element discretization |

Discretization can be specified as:
- Array of floats: explicit normalized positions `[0.0, 0.1, ..., 1.0]`
- Single integer in array: number of uniform elements `[49]`
- Empty array: discretization from blade input file

### rotor.blades[]

Array of blade definitions. Typically 3 entries for a standard turbine.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `file` | string | - | Path to blade properties file |
| `initial_pitch` | float | deg | Initial blade pitch angle |
| `precone` | float | deg | Blade precone angle (negative = upwind) |

### rna

Rotor-nacelle assembly configuration.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `file` | string | - | Path to RNA properties file |
| `initial_yaw` | float | deg | Initial nacelle yaw angle |
| `yaw_actuator_dynamics` | bool | - | Include yaw actuator dynamics |

### tower

Tower configuration.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `file` | string | - | Path to tower properties file (CSV or JSON) |

### tower.discretization

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `elasto` | array | - | Structural element discretization |
| `aero` | array | - | Aerodynamic element discretization |

Discretization can be specified as:
- Array of floats: explicit normalized positions `[0.0, 0.1, ..., 1.0]`
- Single integer in array: number of uniform elements `[49]`
- Empty array: discretization from tower input file

### tower.options

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `use_MacCamyFuchs_correction` | bool | - | Wave diffraction correction for large diameters |
| `use_Cd_correction` | bool | - | Reynolds number drag coefficient correction |

### controller

Control system configuration.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `type` | string | - | Controller type: `""` (none), `"DISCON"`, or `"RPM"` |

#### controller.options (DISCON)

For Bladed-style DLL controllers (e.g. ROSCO):

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `infile` | string | - | Path to controller input file (e.g., DISCON.IN) |
| `libfile` | string | - | Path to controller library (.so on Linux, .dll on Windows) |

#### controller.options (RPM)

Simple variable-torque controller targeting a maximum rotor speed:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `target_rpm` | float | RPM | Target maximum rotor speed |

### foundation (optional)

For offshore configurations with monopile or floating foundation.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `type` | string | - | Foundation type: `"floater"` or `"monopile"` |
| `file` | string | - | Path to foundation file |

#### monopile.options

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `use_MacCamyFuchs_correction` | bool | - | Wave diffraction correction for large diameters |
| `use_Cd_correction` | bool | - | Reynolds number drag coefficient correction |

##### monopile.options.discretization

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `elasto` | array | - | Structural element discretization |
| `aero` | array | - | Aerodynamic element discretization |

Discretization can be specified as:
- Array of floats: explicit normalized positions `[0.0, 0.1, ..., 1.0]`
- Single integer in array: number of uniform elements `[49]`
- Empty array: discretization from monopile input file


## Controller Examples

### No Controller

```json
"controller": {
  "type": ""
}
```

### DISCON Controller

```json
"controller": {
  "type": "DISCON",
  "options": {
    "infile": "../base/controller/DISCON.IN",
    "libfile": "../base/controller/libdiscon.so"
  }
}
```

### RPM Controller

```json
"controller": {
  "type": "RPM",
  "options": {
    "target_rpm": 7.56
  }
}
```
