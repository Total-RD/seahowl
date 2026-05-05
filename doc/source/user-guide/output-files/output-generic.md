# Generic output Files

SEAHOWL generates output files during a simulation to record turbine response, environmental conditions, and user-defined quantities. Output files are written to a configurable output folder (default: `./output`).

## Output Configuration

Output settings are defined in the `outputs` section of the [main input file](../input-files/main-file.md), or set programmatically via the `OutputManager`.

| Setting | Type | Default | Description |
|---------|------|---------|-------------|
| `dt` | float | 0.0 | Output time interval [s]. A value of 0 writes every time step. |
| `folder` | string | `./output` | Directory where output files are created |
| `csv` | bool | `true` | Enable CSV output files |
| `vtk` | bool | `false` | Enable VTK visualization files (blades, tower, monopile) |
| `gui` | bool | `false` | Enable real-time 3D in-situ visualization |

### JSON Example

```json
"outputs": {
    "dt": 0.1,
    "folder": "./output",
    "vtk": false,
    "gui": false
}
```

### Python Example

```python
simulation = seahowl.core.Simulation()
simulation.outputs.dt_output = 0.1
simulation.outputs.has_csv = True
simulation.outputs.has_vtk = False
simulation.outputs.has_gui = True
simulation.outputs.set_output_folder("./output")
```

## Default Turbine Output

When CSV output is enabled, SEAHOWL automatically generates a `turbine{N}_output.csv` file for each turbine (where `N` is the 1-based turbine index). This file contains the main quantities of interest for wind turbine analysis.

### Columns

| Column | Unit | Description |
|--------|------|-------------|
| `time` | s | Simulation time |
| `wind speed hub x/y/z` | m/s | Wind velocity at hub height |
| `rpm` | - | Rotor rotational speed |
| `power` | W | Electrical power |
| `pitch collective` | rad | Collective blade pitch angle |
| `torque elec` | Nm | Electrical generator torque |
| `axial thrust x/y/z` | N | Rotor thrust force |
| `axial torque x/y/z` | Nm | Rotor torque |
| `rotor azimuth` | rad | Rotor azimuthal position |
| `tower base moment x/y/z` | Nm | Bending moment at tower base |
| `tower base force x/y/z` | N | Shear force at tower base |
| `tower top moment x/y/z` | Nm | Bending moment at tower top |
| `tower top force x/y/z` | N | Shear force at tower top |

For each blade (1 to N):

| Column | Unit | Description |
|--------|------|-------------|
| `blade{N} root moment x/y/z` | Nm | Blade root bending moment |
| `blade{N} azimuth` | rad | Blade azimuthal position |
| `blade{N} pitch` | rad | Individual blade pitch angle |

For floating turbines, additional columns are included:

| Column | Unit | Description |
|--------|------|-------------|
| `floater position x/y/z` | m | Floater center of mass position |
| `floater rotation x/y/z` | rad | Floater rotation (roll, pitch, yaw) |
| `fairlead{N} tension` | N | Mooring line tension at each fairlead |

For monopile foundations:

| Column | Unit | Description |
|--------|------|-------------|
| `monopile base moment x/y/z` | Nm | Moment at monopile base |
| `monopile base force x/y/z` | N | Force at monopile base |

All the outputs are in the local reference frame of each body.

### CSV Format

The output CSV uses comma-separated values with a header row. Column names include the unit in square brackets (e.g. `time [s]`). 3D vector quantities are expanded into three columns with `x`, `y`, `z` suffixes.

```
time [s],wind speed hub x [m/s],wind speed hub y [m/s],wind speed hub z [m/s],rpm [-],power [W],...
0.000000,12.000000,0.000000,0.000000,0.000000,0.000000,...
0.100000,11.998813,0.000000,0.000000,0.002163,0.000000,...
```
