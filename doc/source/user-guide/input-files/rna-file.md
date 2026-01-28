# RNA Input File

The RNA (Rotor-Nacelle Assembly) file defines properties of the hub, shaft, nacelle, and drivetrain.

## File Structure

```
rna.json
│
├── shaft/                                 # Shaft configuration
│   ├── tilt: float                        # Shaft tilt angle [deg]
│   └── distance_from_towertop: float      # Shaft height above tower top [m]
│
├── nacelle/                               # Nacelle properties
│   ├── position_from_towertop: [x, y, z]  # CoM offset from tower top [m]
│   ├── mass: float                        # Nacelle mass [kg]
│   ├── inertia: [[3x3]]                   # Inertia tensor [kg-m^2]
│   └── yaw_bearing_mass: float            # Yaw bearing mass [kg]
│
├── drivetrain/                            # Drivetrain properties
│   ├── generator_inertia: float           # Generator inertia [kg-m^2]
│   ├── generator_efficiency: float        # Generator efficiency [%]
│   ├── gearbox_ratio: float               # Gearbox ratio [-]
│   └── gearbox_efficiency: float          # Gearbox efficiency [%]
│
└── hub/                                   # Hub properties
    ├── radius: float                      # Hub radius [m]
    ├── overhang: float                    # Hub overhang from tower axis [m]
    ├── position_from_apex: [x, y, z]      # CoM offset from hub apex [m]
    ├── mass: float                        # Hub mass [kg]
    └── inertia: [[3x3]]                   # Inertia tensor [kg-m^2]
```

## Field Reference

### shaft

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `tilt` | float | deg | Shaft tilt angle (negative = upward tilt) |
| `distance_from_towertop` | float | m | Vertical distance from tower top to shaft centerline |

### nacelle

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `position_from_towertop` | [float, float, float] | m | Nacelle center of mass position relative to tower top |
| `mass` | float | kg | Total nacelle mass (excluding hub) |
| `inertia` | 3x3 matrix | kg-m^2 | Nacelle inertia tensor about CoM |
| `yaw_bearing_mass` | float | kg | Yaw bearing mass |

### drivetrain

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `generator_inertia` | float | kg-m^2 | Generator rotor inertia |
| `generator_efficiency` | float | % | Generator electrical efficiency |
| `gearbox_ratio` | float | - | Gearbox ratio (generator speed / rotor speed) |
| `gearbox_efficiency` | float | % | Gearbox mechanical efficiency |

### hub

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `radius` | float | m | Hub radius (blade root attachment) |
| `overhang` | float | m | Hub overhang from tower axis (negative = upwind) |
| `position_from_apex` | [float, float, float] | m | Hub center of mass offset from hub apex |
| `mass` | float | kg | Hub mass |
| `inertia` | 3x3 matrix | kg-m^2 | Hub inertia tensor about CoM |

## Example

```{literalinclude} ../../../../data/IEA15MW/base/rna.json
:language: json
:caption: IEA 15MW RNA properties
```

## Usage Notes

### Coordinate System

All positions are in the nacelle coordinate system:
- **X**: Downwind along shaft axis (positive toward hub)
- **Y**: Lateral (positive to port)
- **Z**: Upward

### Shaft Tilt

The shaft tilt angle is measured from x-y plane. Negative values tilt the rotor upward (typical for upwind turbines to provide tower clearance).

### Hub Overhang

Hub overhang is measured from the tower centerline to the blade root plane:
- Negative values indicate an upwind rotor
- Positive values indicate a downwind rotor

### Drivetrain Modeling

For direct-drive turbines (no gearbox):
- Set `gearbox_ratio` to `1.0`
- Set `gearbox_efficiency` to `100.0`

### Inertia Tensor Format

The inertia tensor is specified as a 3x3 matrix:

```json
"inertia": [
  [Ixx, Ixy, Ixz],
  [Ixy, Iyy, Iyz],
  [Ixz, Iyz, Izz]
]
```

For symmetric bodies, off-diagonal terms are often zero:

```json
"inertia": [
  [Ixx, 0.0, 0.0],
  [0.0, Iyy, 0.0],
  [0.0, 0.0, Izz]
]
```

For the hub, the inertia is often defined only about its local X axis:


```json
"inertia": [
  [Ixx, 0.0, 0.0],
  [0.0, 0.0, 0.0],
  [0.0, 0.0, 0.0]
]
```
