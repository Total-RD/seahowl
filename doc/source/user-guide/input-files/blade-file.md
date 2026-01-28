# Blade Input File

The blade file defines structural and aerodynamic properties along the blade span, including 6x6 mass and stiffness matrices for each reference point.

## File Structure

```
blade.json
│
├── global_variables/                      # Default values for all points
│   ├── offset_gravity: [y, z]             # Gravity center offset [m]
│   ├── offset_elastic: [y, z]             # Elastic axis offset [m]
│   ├── damping_flapwise: float            # Flapwise Rayleigh damping [-]
│   ├── damping_edgewise: float            # Edgewise Rayleigh damping [-]
│   ├── damping_axial: float               # Axial Rayleigh damping [-]
│   ├── damping_torsion: float             # Torsional Rayleigh damping [-]
│   └── damping_mass: float                # Mass-proportional damping [-]
│
└── reference_points[]                     # Array of spanwise stations
    ├── coordinates: [x, y, z]             # Position (IEC standard) [m]
    ├── fraction: float                    # Normalized span (0=root, 1=tip) [-]
    ├── twist: float                       # Structural twist [deg]
    │
    ├── mass_matrix: [[6x6]]               # Mass matrix [kg/m, kg-m^2/m]
    ├── stiffness_matrix: [[6x6]]          # Stiffness matrix [(N/m)/m, ...]
    │
    ├── chord: float                       # Chord length [m]
    ├── airfoil_file: string               # Path to airfoil file
    └── offset_aero: [y, z]                # Aerodynamic center offset [m]
```

## Field Reference

### global_variables

Default values applied to all reference points (unless overridden at the point level).

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `offset_gravity` | [float, float] | m | Gravity center offset from reference axis [y, z] |
| `offset_elastic` | [float, float] | m | Elastic axis offset from reference axis [y, z] |
| `damping_flapwise` | float | - | Stiffness-proportional Rayleigh damping, flapwise |
| `damping_edgewise` | float | - | Stiffness-proportional Rayleigh damping, edgewise |
| `damping_axial` | float | - | Stiffness-proportional Rayleigh damping, axial |
| `damping_torsion` | float | - | Stiffness-proportional Rayleigh damping, torsion |
| `damping_mass` | float | - | Mass-proportional Rayleigh damping coefficient |

### reference_points[]

Array of blade stations defining properties along the span.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `coordinates` | [float, float, float] | m | Position (x: prebend, y: sweep, z: main axis) |
| `fraction` | float | - | Normalized spanwise position (0 = root, 1 = tip) |
| `twist` | float | deg | Structural twist angle |
| `mass_matrix` | 6x6 matrix | various | Sectional mass matrix per unit length |
| `stiffness_matrix` | 6x6 matrix | various | Sectional stiffness matrix per unit length |
| `chord` | float | m | Chord length |
| `airfoil_file` | string | - | Path to airfoil polar file (for built-in BEMT) |
| `offset_aero` | [float, float] | m | Aerodynamic center offset [x, y] |

## Mass and Stiffness Matrices

### Matrix Format

Both matrices are 6x6, representing coupled behavior in 6 degrees of freedom per unit length:

```
     [  ux    uy    uz    Rx    Ry    Rz  ]
ux   [ m11   m12   m13   m14   m15   m16 ]
uy   [ m21   m22   m23   m24   m25   m26 ]
uz   [ m31   m32   m33   m34   m35   m36 ]
Rx   [ m41   m42   m43   m44   m45   m46 ]
Ry   [ m51   m52   m53   m54   m55   m56 ]
Rz   [ m61   m62   m63   m64   m65   m66 ]
```

Where:
- `ux, uy, uz` - translations along x (edgewise), y (flapwise), z (spanwise)
- `Rx, Ry, Rz` - rotations about x (flapwise bending), y (edgewise bending), z (torsion)

### Mass Matrix Units

| Block | Unit |
|-------|------|
| Translation-translation (rows/cols 1-3) | kg/m |
| Translation-rotation (rows 1-3, cols 4-6) | kg-m/m |
| Rotation-translation (rows 4-6, cols 1-3) | kg-m/m |
| Rotation-rotation (rows/cols 4-6) | kg-m^2/m |

### Stiffness Matrix Units

| Block | Unit |
|-------|------|
| Translation-translation | (N/m)/m |
| Translation-rotation | (N/rad)/m |
| Rotation-translation | (N-m/m)/m |
| Rotation-rotation | (N-m/rad)/m |

### FEA vs FPM Rotor Types

- **FPM** (`"type": "fpm"`): Uses the full 6x6 matrices including off-diagonal coupling terms
- **FEA** (`"type": "fea"`): Uses only diagonal terms of the matrices

## Example

```json
{
  "global_variables": {
    "damping_flapwise": 0.00299005,
    "damping_edgewise": 0.00218775,
    "damping_axial": 0.00084171,
    "damping_torsion": 0.00084171,
    "damping_mass": 0.0,
    "offset_gravity": [0.0, 0.0],
    "offset_elastic": [0.0, 0.0]
  },
  "reference_points": [
    {
      "coordinates": [0.0, 0.0, 0.0],
      "twist": 15.59,
      "fraction": 0.0,
      "stiffness_matrix": [
        [6.74e9, 2.65e6, 0.0, 0.0, 0.0, 1.48e8],
        [2.65e6, 6.73e9, 0.0, 0.0, 0.0, 3.90e7],
        [0.0, 0.0, 4.61e10, -1.09e9, 1.88e7, 0.0],
        [0.0, 0.0, -1.09e9, 1.50e11, -2.26e7, 0.0],
        [0.0, 0.0, 1.88e7, -2.26e7, 1.50e11, 0.0],
        [1.48e8, 3.90e7, 0.0, 0.0, 0.0, 8.75e10]
      ],
      "mass_matrix": [
        [3127.4, 0.0, 0.0, 0.0, 0.0, 73.9],
        [0.0, 3127.4, 0.0, 0.0, 0.0, -0.2],
        [0.0, 0.0, 3127.4, -73.9, 0.2, 0.0],
        [0.0, 0.0, -73.9, 10168.0, 1.1, 0.0],
        [0.0, 0.0, 0.2, 1.1, 10166.3, 0.0],
        [73.9, -0.2, 0.0, 0.0, 0.0, 20334.3]
      ],
      "chord": 5.2,
      "airfoil_file": "airfoils/IEA-15-240-RWT_AeroDyn15_Polar_00.json",
      "offset_aero": [0.0, -0.024]
    }
  ]
}
```

## Airfoil File

Each reference point can specify an airfoil polar file for built-in BEMT:

```json
{
  "alpha": [-180, -170, ..., 170, 180],
  "Cl": [-0.5, -0.4, ..., 0.4, 0.5],
  "Cd": [0.02, 0.02, ..., 0.02, 0.02],
  "Cm": [0.0, 0.0, ..., 0.0, 0.0]
}
```

| Field | Type | Description |
|-------|------|-------------|
| `alpha` | array | Angle of attack values [deg] |
| `Cl` | array | Lift coefficient [-] |
| `Cd` | array | Drag coefficient [-] |
| `Cm` | array | Moment coefficient [-] |

## Usage Notes

### Blade Coordinate System

- **X**: Toward leading edge (chordwise)
- **Y**: Toward low-pressure side (flapwise)
- **Z**: Toward blade tip (spanwise)
- Origin at blade root
