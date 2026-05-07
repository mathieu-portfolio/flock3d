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
python3 -m pip install pandas matplotlib
```

Plot any sampled metric against another CSV column:

```bash
python3 scripts/plot_metric.py \
  --input outputs/run.csv \
  --x simulation_time \
  --y polarization \
  --output outputs/polarization.png
```

Compare one-parameter sweep exports by grouping rows by a sweep column and plotting each value against the mean metric:

```bash
python3 scripts/compare_sweeps.py \
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
5. Writes a CSV in `outputs/` and a PNG in `outputs/plots/`.

Install plotting dependencies once if needed:

```bash
python3 -m pip install pandas matplotlib
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

| Study | Deterministic sweep | CSV output | Plot output | How to read it |
| --- | --- | --- | --- | --- |
| Noise | `steering_noise_strength=0:0.4:0.1` with `noise_baseline` | `outputs/noise_steering_sweep.csv` | `outputs/plots/noise_strength_vs_polarization.png` | Polarization is the collective-order metric; a downward trend means injected steering perturbations are disrupting alignment. |
| BirdFlight | `gravity=6:14:2` with `bird_baseline` | `outputs/birdflight_gravity_sweep.csv` | `outputs/plots/gravity_vs_mean_altitude.png` | Mean altitude summarizes flight stability; falling altitude at high gravity shows when lift and altitude correction can no longer maintain the target band. |
| FishSchool | `drag_coefficient=0:1:0.25` with `fish_baseline` | `outputs/fishschool_drag_sweep.csv` | `outputs/plots/drag_vs_polarization.png` | Polarization shows school alignment; drag changes the damping regime and can either smooth or suppress coordinated motion depending on magnitude. |

The scripts intentionally use `--export-mode summary` so each sweep value contributes one averaged row, which keeps artifacts small and makes `scripts/compare_sweeps.py` the only plotting step required.

## Suggested analysis workflow

1. Choose a scenario question from [scenarios.md](scenarios.md).
2. Run a short sampled export to inspect metric behavior over time.
3. Switch to `--export-mode summary` for parameter sweeps.
4. Plot the sweep with `scripts/compare_sweeps.py`.
5. Keep the seed, timestep, boid count, sample rate, and duration fixed when comparing only one parameter.
