# Metrics and experiments

`flock3d` exports sampled macroscopic metrics and provides a headless experiment runner for deterministic scripted comparisons. This document keeps experiment-specific reference material out of the top-level project README.

## Sampled CSV export

`flock3d` exports **sampled macroscopic metrics** rather than per-frame state dumps by default. The simulation can advance at a fixed timestep such as 120 Hz while metrics are sampled at an independent rate such as 5 Hz. For example, with `fixed_dt = 1/120` and `sample_rate_hz = 5`, one CSV row is emitted every 0.2 seconds of simulation time.

This keeps experiment files compact, emphasizes collective-behavior observables, and avoids tying scientific data volume to rendering framerate.

CSV files are written under `outputs/` by the interactive app using timestamped filenames. The overlay shows whether recording is active, the selected export mode, the sample rate, and the current output filename.

Supported export modes:

- **SampledTimeSeries** (default): writes one row per sample containing scenario, seed, timestamp, git commit when available, sample rate, sample index, simulation time, boid count, polarization, cohesion, dispersion, average speed, average neighbors, nearest-neighbor distance, simulation update time, neighbor queries, spatial hash cell count, mean altitude, altitude variance, stall count, near-ground count, and optional sweep metadata.
- **Summary**: samples at the same independent cadence but writes one aggregate row at the end of recording or at the end of a headless run. The aggregate records mean polarization, mean cohesion, max dispersion, mean speed, mean neighbors, mean flight metrics, and total duration in the shared CSV schema; `SummaryAggregator` also retains max polarization for code-level consumers.
- **FullTrajectory**: declared as a placeholder for future full state export and intentionally not implemented yet.

## BirdFlight stability metrics

BirdFlight experiments intentionally use a compact stability metric set so runs are easy to compare:

- `mean_altitude`: the average `y` position of all birds in the sampled simulation step. Higher or lower values show whether the flock is holding its target altitude band.
- `altitude_variance`: the variance of bird altitude around `mean_altitude` in the sampled step. Lower values indicate a tighter, more level flock.
- `stall_count`: the number of birds whose speed is below the BirdFlight `min_speed` threshold in the sampled step. Larger values indicate more birds are losing stable forward flight.

These metrics are deterministic for a fixed seed, timestep, preset, and override set. They are written to sampled CSV exports and summary CSV exports alongside the existing collective metrics.

## Headless experiment runner

The `flock3d_experiment_runner` executable reuses the scenario and simulation code without creating a raylib window. Runs are deterministic for the same scenario, seed, boid count, fixed timestep, duration, sample rate, export mode, preset, and swept parameter value.

Example sampled time-series run:

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

Example summary run:

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

## One-parameter sweeps

One-parameter sweeps use inclusive `start:end:step` ranges. Only one `--sweep` argument is supported for now, and each output row includes `sweep_parameter` and `sweep_value` metadata.

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

./build/local/bin/flock3d_experiment_runner \
  --scenario ClassicBoids \
  --seed 123 \
  --boids 1024 \
  --duration 20 \
  --fixed-dt 0.008333 \
  --sample-rate 5 \
  --export-mode sampled \
  --sweep alignment_weight=0:3:0.5 \
  --output outputs/alignment_weight_sweep.csv
```

BirdFlight sweeps support at least `gravity`, `lift_strength`, `max_turn_rate`, `field_of_view_degrees`, and `altitude_correction_strength`.

## BirdFlight presets

The experiment runner supports `--preset <name>`. Presets apply defaults first, then explicit CLI options such as `--seed`, `--boids`, `--duration`, `--sample-rate`, `--export-mode`, `--output`, and `--sweep` are applied after the preset.

Available presets:

- `bird_baseline`: default BirdFlight stability baseline.
- `bird_low_lift`: reduced lift strength while leaving the baseline gravity and visibility model intact.
- `bird_high_gravity`: stronger gravity with baseline lift, field of view, and turn rate.
- `bird_narrow_fov`: narrower forward field of view for neighbor perception.
- `bird_low_turn_rate`: reduced maximum turn rate for slower heading correction.

Run a short sampled baseline export:

```bash
./build/local/bin/flock3d_experiment_runner \
  --preset bird_baseline \
  --duration 30 \
  --sample-rate 5 \
  --output outputs/bird_baseline.csv
```

Compare low lift with a summary export:

```bash
./build/local/bin/flock3d_experiment_runner \
  --preset bird_low_lift \
  --duration 60 \
  --export-mode summary \
  --output outputs/bird_low_lift_summary.csv
```

Sweep gravity after applying the high-gravity preset defaults:

```bash
./build/local/bin/flock3d_experiment_runner \
  --preset bird_high_gravity \
  --sweep gravity=8:14:1 \
  --duration 45 \
  --output outputs/bird_gravity_sweep.csv
```

Override run size and seed while keeping the narrow-FOV preset parameters:

```bash
./build/local/bin/flock3d_experiment_runner \
  --preset bird_narrow_fov \
  --seed 2401 \
  --boids 1024 \
  --duration 30 \
  --output outputs/bird_narrow_fov_seed2401.csv
```
