# Tower Input File

The tower file defines the structural and hydrodynamic properties of the tower along its length. It can be specified in CSV or JSON format.

## File Structure (JSON)

```
tower.json
│
└── reference_points[]                     # Array of reference points
    ├── position_x: float                  # X coordinate [m]
    ├── position_y: float                  # Y coordinate [m]
    ├── position_z: float                  # Z coordinate (height) [m]
    │
    ├── diameter: float                    # Outer diameter [m]
    ├── thickness: float                   # Wall thickness [m]
    │
    ├── density: float                     # Material density [kg/m^3]
    ├── young_modulus: float               # Young's modulus [Pa]
    ├── poisson_ratio: float               # Poisson's ratio [-]
    │
    ├── drag_coefficient_axial: float      # Axial Morison Cd [-]
    ├── drag_coefficient_normal: float     # Normal Morison Cd [-]
    ├── added_mass_coefficient_axial: float    # Axial Morison Ca [-]
    ├── added_mass_coefficient_normal: float   # Normal Morison Ca [-]
    │
    ├── fill_density: float                # Inner fill density [kg/m^3]
    ├── buoyancy_factor: float             # Buoyancy factor (0-1) [-]
    │
    ├── damping_foreaft: float             # Fore-aft Rayleigh damping [-]
    ├── damping_sideside: float            # Side-side Rayleigh damping [-]
    ├── damping_axial: float               # Axial Rayleigh damping [-]
    ├── damping_torsion: float             # Torsional Rayleigh damping [-]
    └── damping_mass: float                # Mass-proportional damping [-]
```

## Field Reference

### Geometry

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `position_x` | float | m | X coordinate of reference point |
| `position_y` | float | m | Y coordinate of reference point |
| `position_z` | float | m | Z coordinate (height above base) |
| `diameter` | float | m | Outer diameter at this location |
| `thickness` | float | m | Wall thickness |

### Material Properties

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `density` | float | kg/m^3 | Material volumetric density |
| `young_modulus` | float | Pa | Young's modulus (elastic modulus) |
| `poisson_ratio` | float | - | Poisson's ratio (typically ~0.3 for steel) |

### Hydrodynamic Coefficients

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `drag_coefficient_axial` | float | - | Morison drag coefficient, axial direction |
| `drag_coefficient_normal` | float | - | Morison drag coefficient, normal direction |
| `added_mass_coefficient_axial` | float | - | Morison added mass (Ca), axial direction |
| `added_mass_coefficient_normal` | float | - | Morison added mass (Ca), normal direction |

### Additional Mass

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `fill_density` | float | kg/m^3 | Density of material filling the hollow section (0 if empty) |
| `buoyancy_factor` | float | - | 0.0 = no buoyancy, 1.0 = fully buoyant |

### Damping (Rayleigh)

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `damping_foreaft` | float | - | Stiffness-proportional damping, fore-aft bending |
| `damping_sideside` | float | - | Stiffness-proportional damping, side-side bending |
| `damping_axial` | float | - | Stiffness-proportional damping, axial direction |
| `damping_torsion` | float | - | Stiffness-proportional damping, torsion |
| `damping_mass` | float | - | Mass-proportional damping coefficient |

## CSV Format

The tower can also be defined as a CSV file with the same fields as column headers:

```csv
position_x,position_y,position_z,diameter,thickness,density,young_modulus,...
0.0,0.0,15.0,10.0,0.0399,7850,2.1e11,...
0.0,0.0,28.0,9.926,0.0399,7850,2.1e11,...
...
```

## Example (JSON)

```json
{
  "reference_points": [
    {
      "position_x": 0.0,
      "position_y": 0.0,
      "position_z": 15.0,
      "diameter": 10.0,
      "thickness": 0.0399,
      "density": 7850,
      "young_modulus": 2.1e11,
      "poisson_ratio": 0.3,
      "drag_coefficient_axial": 0.0,
      "drag_coefficient_normal": 1.0,
      "added_mass_coefficient_axial": 0.0,
      "added_mass_coefficient_normal": 1.0,
      "fill_density": 0.0,
      "buoyancy_factor": 0.0,
      "damping_foreaft": 0.01,
      "damping_sideside": 0.01,
      "damping_axial": 0.01,
      "damping_torsion": 0.01,
      "damping_mass": 0.0
    }
  ]
}
```

## Example (CSV)

```{literalinclude} ../../../../data/IEA15MW/onshore/tower.csv
:language: text
:lines: 1-5
:caption: IEA 15MW tower properties (truncated)
```

## Usage Notes

### Reference Points and Interpolation

- Reference points define properties at specific heights
- SEAHOWL linearly interpolates between points for the actual simulation discretization
- Include enough points to capture property variations (e.g., tapering diameter)

### Coordinate System

- First point is typically at the tower base
- Next points go from tower base to tower top
- Z-axis is along the tower main axis (positive upward)
- For monopile configurations, Z can extend below the mudline

### Rayleigh Damping

Where:
- `damping_mass` = alpha (mass-proportional coefficient)
- `damping_*` = beta (stiffness-proportional coefficients for each direction)
