# Floater Input File

The floater file defines the floating platform properties, including mass, inertia, hydrodynamic characteristics, additional bodies, and mooring system.

## File Structure

```
floater.json
│
├── type: string                           # Hydrodynamics type ("HydroChrono")
├── options/
│   └── file: string                       # Path to .h5 hydro data file
│
├── position: [x, y, z]                    # Center of gravity [m]
├── mass: float                            # Total platform mass [kg]
├── inertia: [[3x3]]                       # Inertia tensor [kg-m^2]
├── damping_matrix: [[6x6]]                # Viscous damping matrix
│
├── bodies[]                               # Additional rigid bodies
│   ├── name: string                       # Body identifier
│   ├── position: [x, y, z]                # Body CoG position [m]
│   ├── mass: float                        # Body mass [kg]
│   └── inertia: [[3x3]]                   # Body inertia [kg-m^2]
│
└── moorings[]                             # Mooring lines
    ├── connected_body_name: string        # Body to attach fairlead
    ├── line_properties: string            # Path to line properties file
    ├── length: float                      # Unstretched line length [m]
    ├── discretization/
    │   ├── elasto: array                  # Structural discretization
    │   └── hydro: array                   # Hydrodynamic discretization
    ├── fairlead_position: [x, y, z]       # Fairlead attachment point [m]
    ├── relative_fairlead: bool            # Position relative to body
    ├── anchor_position: [x, y, z]         # Anchor attachment point [m]
    ├── relative_anchor: bool              # Position relative to body
    ├── rotation_axis: [x, y, z]           # Axis for line rotation
    └── rotation_angle: float              # Angle to rotate line [deg]
```

## Field Reference

### Main Properties

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `type` | string | - | Hydrodynamics model: `"HydroChrono"` |
| `position` | [float, float, float] | m | Floater center of gravity (CoG) position |
| `mass` | float | kg | Total floater mass |
| `inertia` | 3x3 matrix | kg-m^2 | Inertia tensor about CoG |
| `damping_matrix` | 6x6 matrix | various | Viscous (quadratic) damping matrix |

### options (HydroChrono)

| Field | Type | Description |
|-------|------|-------------|
| `file` | string | Path to HDF5 file containing BEM hydrodynamic data |

The `.h5` file contains radiation/diffraction coefficients from a BEM solver (WAMIT, Nemoh, Capytaine, etc.).

### bodies[]

Hydrodynamic bodies connected to the main platform. They must match the bodies defined in the HDF5 (`.h5`) hydrodynamic database:
1. Define each body in the `bodies[]` array
2. Ensure body names match thos in the `.h5` file if using HydroChrono
3. Ensure body positions match those from which hydrodynamic properties where calculated in the `.h5` file

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `name` | string | - | Unique identifier |
| `position` | [float, float, float] | m | Body position |
| `mass` | float | kg | Body mass |
| `inertia` | 3x3 matrix | kg-m^2 | Body inertia tensor about CoG |

### moorings[]

Mooring line definitions.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `connected_body_name` | string | - | Name of body where fairlead is attached |
| `line_properties` | string | - | Path to mooring line properties file |
| `length` | float | m | Unstretched mooring line length |
| `fairlead_position` | [float, float, float] | m | Fairlead attachment point |
| `relative_fairlead` | bool | - | If true, position is relative to connected body |
| `anchor_position` | [float, float, float] | m | Anchor attachment point |
| `relative_anchor` | bool | - | If true, position is relative to connected body |
| `rotation_axis` | [float, float, float] | - | Axis to rotate mooring line about |
| `rotation_angle` | float | deg | Angle to rotate mooring line |

### moorings[].discretization

| Field | Type | Description |
|-------|------|-------------|
| `elasto` | array | Structural element discretization |
| `hydro` | array | Hydrodynamic element discretization |

## Mooring Line Properties File

Each mooring line references a properties file:

```json
{
  "diameter": 0.0766,
  "mass_per_length": 113.35,
  "stiffness_axial": 7.536e8,
  "stiffness_bending": 0.0,
  "drag_coefficient_axial": 0.0,
  "drag_coefficient_normal": 2.0,
  "added_mass_coefficient_axial": 0.0,
  "added_mass_coefficient_normal": 1.0
}
```

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `diameter` | float | m | Equivalent hydrodynamic diameter |
| `mass_per_length` | float | kg/m | Mass per unit length |
| `stiffness_axial` | float | N | Axial stiffness EA (Young's modulus x area) |
| `stiffness_bending` | float | N-m^2 | Bending stiffness EI (typically 0 for chain/cable) |
| `drag_coefficient_axial` | float | - | Morison axial drag coefficient (Cd) |
| `drag_coefficient_normal` | float | - | Morison normal drag coefficient |
| `added_mass_coefficient_axial` | float | - | Morison axial added mass (Ca) |
| `added_mass_coefficient_normal` | float | - | Morison normal added mass |

## Usage Notes

### Mooring Line Rotation

The `rotation_axis` and `rotation_angle` fields allow defining multiple mooring lines from a template:

```json
// First line (reference)
{
  "fairlead_position": [-58.0, 0.0, 0.94],
  "anchor_position": [-837.60, 0.0, -200.0],
  "rotation_angle": 0.0
},
// Second line (rotated 120deg)
{
  "fairlead_position": [-58.0, 0.0, 0.94],
  "anchor_position": [-837.60, 0.0, -200.0],
  "rotation_axis": [0.0, 0.0, 1.0],
  "rotation_angle": 120.0
},
// Third line (rotated 240deg)
{
  "fairlead_position": [-58.0, 0.0, 0.94],
  "anchor_position": [-837.60, 0.0, -200.0],
  "rotation_axis": [0.0, 0.0, 1.0],
  "rotation_angle": 240.0
}
```

### HydroChrono Integration

When using `"type": "HydroChrono"`:
- SEAHOWL must be built with `SEAHOWL_ENABLE_HYDROCHRONO=ON`
- The `.h5` file must contain radiation/diffraction data for the platform geometry

### Multi-Body Floaters

For floaters with multiple bodies (e.g., semi-submersibles with separate columns):
1. Define each body in the `bodies[]` array
2. Reference body names when connecting mooring lines
3. Ensure body names match those in the `.h5` file if using HydroChrono

### Viscous Damping

The `damping_matrix` represents additional viscous damping not captured by radiation damping.
