# Input Files

SEAHOWL uses JSON files for configuration. This section documents each file type in detail.

## File Hierarchy

```
main.json                           # Simulation entry point
│
├── numerics/                       # Time stepping parameters
├── outputs/                        # Output configuration
├── environment/
│   └── file ──────────────────────► environment.json
│                                    ├── gravity
│                                    ├── wind/
│                                    ├── sea/
│                                    └── soil/
│
└── turbines[]
    └── file ──────────────────────► turbine.json
                                     │
                                     ├── aero/                  # Aerodynamics settings
                                     ├── rotor/
                                     │   └── blades[]/file ────► blade.json
                                     │                           ├── global_variables/
                                     │                           └── reference_points[]
                                     │                               └── airfoil_file ──► airfoil.json
                                     ├── rna/
                                     │   └── file ─────────────► rna.json
                                     │                           ├── shaft/
                                     │                           ├── nacelle/
                                     │                           ├── drivetrain/
                                     │                           └── hub/
                                     ├── tower/
                                     │   └── file ─────────────► tower.csv or tower.json
                                     │
                                     ├── controller/
                                     │   └── options/
                                     │       ├── infile ───────► DISCON.IN
                                     │       └── libfile ──────► libdiscon.so
                                     │
                                     └── foundation/  (optional)
                                         └── file ─────────────► floater.json
                                                                 ├── bodies[]
                                                                 └── moorings[]
                                                                     └── line_properties ──► mooring.json
```

## File Reference

| File | Description | Link |
|------|-------------|------|
| Main file | Top-level simulation configuration | [main-file.md](main-file.md) |
| Turbine file | Turbine components and discretization | [turbine-file.md](turbine-file.md) |
| Environment file | Wind, waves, current, soil conditions | [environment-file.md](environment-file.md) |
| RNA file | Rotor-nacelle assembly properties | [rna-file.md](rna-file.md) |
| Tower file | Tower geometry and material properties | [tower-file.md](tower-file.md) |
| Blade file | Blade structural and aerodynamic properties | [blade-file.md](blade-file.md) |
| Floater file | Floating platform and mooring system | [floater-file.md](floater-file.md) |

## General Conventions

### File Paths

All file paths in JSON files are **relative to the file containing the reference**:

```json
{
  "environment": {
    "file": "../env/env.json"    // Relative to main.json location
  }
}
```

### Arrays vs Integers for Discretization

Discretization can be specified in two ways:

```json
// Array: explicit normalized positions (0 = root, 1 = tip)
"elasto": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

// Integer: number of uniform elements
"elasto": [10]
```

### Optional vs Required Fields

- Required fields have no default and must be specified
- Optional fields show their default value in the documentation
- Nested objects are required if any of their children are required
