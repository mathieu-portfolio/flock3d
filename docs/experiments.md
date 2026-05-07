# Experiments and CSV analysis

`flock3d` exports sampled macroscopic metrics and provides a deterministic headless experiment runner for scripted comparisons. This document focuses on experiment mechanics: export modes, CSV philosophy, runner usage, presets, sweeps, and plotting scripts. Scenario model details and scenario-specific metrics live in [scenarios.md](scenarios.md).

## CSV philosophy

The project records **sampled aggregate metrics**, not per-frame agent dumps. A simulation can advance at a fixed timestep such as 120 Hz while metrics are sampled independently, for example at 5 Hz. With `fixed_dt = 1/120` and `sample_rate_hz = 5`, one CSV row is emitted every 0.2 seconds of simulation time.

This keeps files compact, makes interactive and headless runs comparable, and emphasizes scientific observables such as order, cohesion, dispersion, depth, altitude, and timing instead of raw trajectory volume.

Interactive CSV files are written under `outputs/` using timestamped filenames. The overlay shows whether recording is active, the selected export mode, the sample rate, and the current output filename.

## Export modes

- **SampledTimeSeries** (`sampled`): default mode. Writes one aggregate row per sample.
- **Summary** (`summary`): samples internally at the same cadence, then writes one aggregate row at the end of recording or at the end of a headless run.
- **FullTrajectory**: reserved for a future compact trajectory schema and intentionally not implemented yet.

The shared CSV schema contains run metadata, timing fields, collective-order metrics, spatial-hash diagnostics, optional scenario-specific metrics, and optional sweep metadata. See [scenarios.md](scenarios.md) for the meaning of scenario-specific columns such as altitude, depth, and order-loss metrics.

## Headless experiment runner

The `flock3d_experiment_runner` executable reuses the same scenario and simulation code as the interactive app without creating a raylib window. Runs are deterministic for the same scenario, seed, boid count, fixed timestep, duration, sample rate, export mode, preset, and swept parameter value.

Sampled time-series example:

```bash
./build/local/bin/flock3d_experiment_runner \
  --scenario ClassicBoids \
  --seed 123 \
  --boids 2048 \
  --duration 30 \
  --fixed-dt 0.008333 \
  --sample-rate 5 \
  --export-mode sampled \
  --output outputs/classic_seed123.csv
```

Summary example:

```bash
./build/local/bin/flock3d_experiment_runner \
  --scenario ClassicBoids \
  --seed 123 \
  --boids 2048 \
  --duration 30 \
  --fixed-dt 0.008333 \
  --sample-rate 5 \
  --export-mode summary \
  --output outputs/classic_seed123_summary.csv
```

## Presets

Presets apply scenario defaults and a curated parameter set before explicit CLI overrides. Options such as `--seed`, `--boids`, `--duration`, `--sample-rate`, `--export-mode`, `--output`, and `--sweep` are applied after the preset.

Available presets:

- BirdFlight: `bird_baseline`, `bird_low_lift`, `bird_high_gravity`, `bird_narrow_fov`, `bird_low_turn_rate`.
- FishSchool: `fish_baseline`, `fish_high_drag`, `fish_strong_current`, `fish_low_visibility`.
- NoiseExperiment: `noise_baseline`, `noise_low`, `noise_medium`, `noise_high`.

Preset example:

```bash
./build/local/bin/flock3d_experiment_runner \
  --preset bird_baseline \
  --duration 30 \
  --sample-rate 5 \
  --output outputs/bird_baseline.csv
```

## One-parameter sweeps

One-parameter sweeps use inclusive `start:end:step` ranges. Only one `--sweep` argument is supported for now, and each output row includes `sweep_parameter` and `sweep_value` metadata.

ClassicBoids perception-radius sweep:

```bash
./build/local/bin/flock3d_experiment_runner \
  --scenario ClassicBoids \
  --seed 123 \
  --boids 1024 \
  --duration 20 \
  --fixed-dt 0.008333 \
  --sample-rate 5 \
  --export-mode sampled \
  --sweep perception_radius=5:30:5 \
  --output outputs/perception_radius_sweep.csv
```

BirdFlight, FishSchool, and NoiseExperiment expose additional sweep parameters documented by scenario in [scenarios.md](scenarios.md). Useful starting points are:

- BirdFlight: `gravity`, `lift_strength`, `max_turn_rate`, `field_of_view_degrees`, `altitude_correction_strength`.
- FishSchool: `drag_coefficient`, `depth_correction_strength`, `target_depth`, `current_strength`.
- NoiseExperiment: `perception_noise_strength`, `steering_noise_strength`, with `velocity_noise_strength`, `noise_seed_offset`, and `noise_enabled` for targeted overrides.

## Plotting scripts

The repository includes small Python scripts for the final analysis step from simulation to CSV to plot. They intentionally keep plotting outside the C++ app and avoid notebooks, GUI code, seaborn, Parquet/HDF5, and complex statistics.

Install plotting dependencies:

```bash
python -m pip install pandas matplotlib
```

Plot any sampled metric against another CSV column:

```bash
python scripts/plot_metric.py \
  --input outputs/run.csv \
  --x simulation_time \
  --y polarization \
  --output outputs/polarization.png
```

Compare one-parameter sweep exports by grouping rows by a sweep column and plotting each value against the mean metric:

```bash
python scripts/compare_sweeps.py \
  --input outputs/noise_sweep.csv \
  --sweep-column sweep_value \
  --metric polarization \
  --output outputs/noise_vs_polarization.png
```

Both scripts check that requested columns exist and exit with an error listing the available columns when a CSV schema or CLI argument does not match.

## Generate example studies

The `scripts/run_*_study.sh` recipe scripts turn the runner and plotting tools into one-command, reproducible result generation for documentation and portfolio artifacts. Each script:

1. Changes to the repository root so relative paths are stable.
2. Checks that `pandas` and `matplotlib` are importable and prints the install command if they are missing.
3. Configures and builds the `flock3d_experiment_runner` target using `${CMAKE_PRESET:-debug}`.
4. Runs a deterministic summary-mode sweep with fixed seed, boid count, timestep, duration, and sample rate.
5. Writes the CSV and multiple scenario-relevant PNGs into the scenario-specific folder under `outputs/`, so each study can compare several metrics without filename conflicts.

Install plotting dependencies once if needed:

```bash
python -m pip install pandas matplotlib
```

Run the curated studies from the repository root:

```bash
./scripts/run_noise_study.sh
./scripts/run_birdflight_study.sh
./scripts/run_fishschool_study.sh
```

Use another CMake preset by setting `CMAKE_PRESET`, for example:

```bash
CMAKE_PRESET=release ./scripts/run_fishschool_study.sh
```

Expected outputs and interpretation:

| Study | Deterministic sweep | CSV output | Plot outputs | How to read it |
| --- | --- | --- | --- | --- |
| Noise | `steering_noise_strength=0:0.4:0.1` with `noise_baseline` | `outputs/noise/noise_steering_sweep.csv` | `outputs/noise/noise_strength_vs_polarization.png`<br>`outputs/noise/noise_strength_vs_order_loss.png`<br>`outputs/noise/noise_strength_vs_dispersion.png`<br>`outputs/noise/noise_strength_vs_cohesion.png` | Polarization and order loss capture degradation of collective order; dispersion and cohesion show spatial spreading and loss of grouping under steering noise. |
| BirdFlight FOV | `field_of_view_degrees=90:270:45` with `bird_baseline` | `outputs/birdflight/birdflight_fov_sweep.csv` | `outputs/birdflight/fov_vs_polarization.png`<br>`outputs/birdflight/fov_vs_dispersion.png`<br>`outputs/birdflight/fov_vs_average_neighbors.png`<br>`outputs/birdflight/fov_vs_stall_count.png` | Polarization, dispersion, accepted neighbors, and stalls measure how limited perception changes flock order and flight stability. |
| BirdFlight turn rate | `max_turn_rate=30:150:30` with `bird_baseline` | `outputs/birdflight/birdflight_turn_rate_sweep.csv` | `outputs/birdflight/turn_rate_vs_polarization.png`<br>`outputs/birdflight/turn_rate_vs_dispersion.png`<br>`outputs/birdflight/turn_rate_vs_cohesion.png`<br>`outputs/birdflight/turn_rate_vs_altitude_variance.png` | Polarization, dispersion, cohesion, and altitude variance measure how maneuverability constraints affect collective order and vertical stability. |
| FishSchool | `drag_coefficient=0:1:0.25` with `fish_baseline` | `outputs/fishschool/fishschool_drag_sweep.csv` | `outputs/fishschool/drag_vs_polarization.png`<br>`outputs/fishschool/drag_vs_cohesion.png`<br>`outputs/fishschool/drag_vs_average_speed.png`<br>`outputs/fishschool/drag_vs_depth_variance.png` | Polarization, cohesion, average speed, and depth variance show how drag affects school alignment, grouping, motion, and depth keeping. |

BirdFlight intentionally emphasizes perception (`field_of_view_degrees`) and maneuverability (`max_turn_rate`) constraints in the curated recipe; gravity remains available for targeted follow-up sweeps, but it is not the primary BirdFlight study question.

The scripts intentionally use `--export-mode summary` so each sweep value contributes one averaged row, which keeps artifacts small. Each metric plot is still produced by the same `scripts/compare_sweeps.py` helper, keeping multi-plot recipes easy to extend. Because sweep exports include `sweep_parameter`, the plot helper labels x-axes with names such as `steering_noise_strength`, `field_of_view_degrees`, `max_turn_rate`, and `drag_coefficient`.

## Suggested analysis workflow

1. Choose a scenario question from [scenarios.md](scenarios.md).
2. Run a short sampled export to inspect metric behavior over time.
3. Switch to `--export-mode summary` for parameter sweeps.
4. Plot the sweep with `scripts/compare_sweeps.py`.
5. Keep the seed, timestep, boid count, sample rate, and duration fixed when comparing only one parameter.
