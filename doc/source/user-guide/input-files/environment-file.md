# Environment Input File

The environment file defines atmospheric and oceanic conditions: wind, waves, current, and soil.

## File Structure

```
environment.json
│
├── gravity: [x, y, z]                     # Gravity vector [m/s^2]
├── ramp_start: float                      # Load ramp start time [s]
├── ramp_end: float                        # Load ramp end time [s]
│
├── wind/                                  # Wind conditions
│   ├── type: string                       # Wind model type
│   ├── air_density: float                 # Air density [kg/m^3]
│   └── options/                           # Type-specific options
│
├── sea/                                   # Sea state
│   ├── type: string                       # Wave model type
│   ├── water_density: float               # Water density [kg/m^3]
│   ├── mean_water_level: float            # Still water level [m]
│   ├── water_depth: float                 # Water depth [m]
│   └── options/                           # Type-specific options
│
└── soil/                                  # Soil/seabed model
    ├── type: string                       # Soil model type
    └── options/                           # Type-specific options
```

## Field Reference

### Global Fields

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `gravity` | [float, float, float] | m/s^2 | Gravity acceleration vector |
| `ramp_start` | float | s | Time to start ramping environmental loads |
| `ramp_end` | float | s | Time when loads reach full magnitude |

### wind

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `type` | string | - | Wind model: `"constant"`, `"ramp"`, `"inflowwind"` |
| `air_density` | float | kg/m^3 | Air density |

### sea

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `type` | string | - | Wave model: `"still"`, `"regular"`, `"irregular"` |
| `water_density` | float | kg/m^3 | Water density |
| `mean_water_level` | float | m | Still water level |
| `water_depth` | float | m | Water depth from seabed to MWL |

### soil

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `type` | string | - | Soil model: `"linear"`, `"fixed"` |

---

## Wind Types

### Constant Wind

Steady uniform wind with power-law shear profile.

```json
"wind": {
  "type": "constant",
  "air_density": 1.225,
  "options": {
    "reference_height": 150,
    "shear_coefficient": 0.12,
    "velocity": [12, 0, 0]
  }
}
```

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `reference_height` | float | m | Height at which velocity is specified |
| `shear_coefficient` | float | - | Power-law shear exponent |
| `velocity` | [float, float, float] | m/s | Wind velocity at reference height |

### Wind Ramp

Linear wind speed variation between two times.

```json
"wind": {
  "type": "ramp",
  "air_density": 1.225,
  "options": {
    "reference_height": 150,
    "shear_coefficient": 0.12,
    "velocity_start": [12, 0, 0],
    "velocity_end": [25, 0, 0],
    "time_start": 500,
    "time_end": 1700
  }
}
```

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `reference_height` | float | m | Height at which velocity is specified |
| `shear_coefficient` | float | - | Power-law shear exponent |
| `velocity_start` | [float, float, float] | m/s | Initial wind velocity |
| `velocity_end` | [float, float, float] | m/s | Final wind velocity |
| `time_start` | float | s | Time at which ramp begins |
| `time_end` | float | s | Time at which ramp ends |

### InflowWind

Integration with NREL InflowWind for turbulent wind fields (requires `SEAHOWL_ENABLE_INFLOWWIND`).

```json
"wind": {
  "type": "inflowwind",
  "options": {
    "file_inflowwind": "./aerodyn/IEA-15-240-RWT_InflowWind.dat",
    "zmin": 5.0
  }
}
```

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `file_inflowwind` | string | - | Path to InflowWind input file |
| `zmin` | float | m | Minimum z coordinate (returns zero wind below this) |

---

## Sea Types

### Still Water

No waves, calm sea surface.

```json
"sea": {
  "type": "still",
  "water_density": 1025,
  "mean_water_level": 0.0,
  "water_depth": 200.0
}
```

### Regular Waves

Monochromatic (single frequency) waves using Airy wave theory.

```json
"sea": {
  "type": "regular",
  "water_density": 1025,
  "mean_water_level": 0.0,
  "water_depth": 200.0,
  "options": {
    "wave_height": 6.0,
    "wave_period": 10.0,
    "wave_direction": 0.0
  }
}
```

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `wave_height` | float | m | Wave height (crest-to-trough) |
| `wave_period` | float | s | Wave period |
| `wave_direction` | float | deg | Direction waves travel toward |

### Irregular Waves

Random sea state using wave spectrum.

```json
"sea": {
  "type": "irregular",
  "water_density": 1025,
  "mean_water_level": 0.0,
  "water_depth": 200.0,
  "options": {
    "spectrum": "JONSWAP",
    "Hs": 6.0,
    "Tp": 10.0,
    "gamma": 3.3,
    "wave_direction": 0.0,
    "seed": 12345
  }
}
```

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `spectrum` | string | - | Spectrum type: `"JONSWAP"`, `"PM"` (Pierson-Moskowitz) |
| `Hs` | float | m | Significant wave height |
| `Tp` | float | s | Peak spectral period |
| `gamma` | float | - | JONSWAP peak enhancement factor (typically 1.0-3.3) |
| `wave_direction` | float | deg | Mean wave direction |
| `seed` | int | - | Random seed for reproducibility |

---

## Soil Types

### Linear Soil

Simple linear spring model for soil-mooring interaction.

```json
"soil": {
  "type": "linear",
  "options": {
    "stiffness_normal": 1e5,
    "stiffness_shear": 0.0,
    "soil_position": -200.0
  }
}
```

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `stiffness_normal` | float | N/m | Normal (vertical) stiffness |
| `stiffness_shear` | float | N/m | Shear (horizontal) stiffness |
| `soil_position` | float | m | Z-coordinate of soil surface |

---
