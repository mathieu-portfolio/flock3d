# Metrics and experiments

`flock3d` exports sampled macroscopic metrics and provides a headless experiment runner for deterministic scripted comparisons. This document keeps experiment-specific reference material out of the top-level project README.

## Sampled CSV export

`flock3d` exports **sampled macroscopic metrics** rather than per-frame state dumps by default. The simulation can advance at a fixed timestep such as 120 Hz while metrics are sampled at an independent rate such as 5 Hz. For example, with `fixed_dt = 1/120` and `sample_rate_hz = 5`, one CSV row is emitted every 0.2 seconds of simulation time.

This keeps experiment files compact, emphasizes collective-behavior observables, and avoids tying scientific data volume to rendering framerate.

CSV files are written under `outputs/` by the interactive app using timestamped filenames. The overlay shows whether recording is active, the selected export mode, the sample rate, and the current output filename.

Supported export modes:

- **SampledTimeSeries** (default): writes one row per sample containing scenario, seed, timestamp, git commit when available, sample rate, sample index, simulation time, boid count, polarization, cohesion, dispersion, average speed, average neighbors, nearest-neighbor distance, simulation update time, neighbor queries, spatial hash cell count, mean altitude, altitude variance, stall count, near-ground count, noise strength, order loss, and optional sweep metadata.
- **Summary**: samples at the same independent cadence but writes one aggregate row at the end of recording or at the end of a headless run. The aggregate records mean polarization, mean cohesion, max dispersion, mean speed, mean neighbors, mean flight/noise metrics, and total duration in the shared CSV schema; `SummaryAggregator` also retains max polarization for code-level consumers.
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

BirdFlight sweeps support at least `gravity`, `lift_strength`, `max_turn_rate`, `field_of_view_degrees`, and `altitude_correction_strength`. NoiseExperiment sweeps support `perception_noise_strength` and `steering_noise_strength`, with `velocity_noise_strength`, `noise_seed_offset`, and `noise_enabled` available for targeted overrides.


## Python CSV analysis tools

The repository includes small Python scripts for the final analysis step from simulation to CSV to plot. They intentionally keep plotting outside the C++ app and avoid notebooks, GUI code, seaborn, Parquet/HDF5, and complex statistics. Install the required Python packages with:

```bash
python3 -m pip install pandas matplotlib
```

Use `plot_metric.py` for a direct metric-over-time or metric-vs-metric plot from any sampled CSV export:

```bash
python3 scripts/plot_metric.py \
  --input outputs/run.csv \
  --x simulation_time \
  --y polarization \
  --output outputs/polarization.png
```

Use `compare_sweeps.py` for one-parameter sweep CSVs. It groups rows by the selected sweep column, computes the mean of the metric for each value, and plots the sweep value against that mean:

```bash
python3 scripts/compare_sweeps.py \
  --input outputs/noise_sweep.csv \
  --sweep-column sweep_value \
  --metric polarization \
  --output outputs/noise_vs_polarization.png
```

Both scripts check that requested columns exist and exit with an error listing the available columns when a CSV schema or CLI argument does not match. Useful starter comparisons include:

- Noise strength vs. polarization: sweep `steering_noise_strength` or `perception_noise_strength`, then plot mean `polarization`.
- Gravity vs. altitude variance: sweep BirdFlight `gravity`, then plot mean `altitude_variance`.
- Drag vs. cohesion: sweep FishSchool `drag_coefficient`, then plot mean `cohesion`.

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

## NoiseExperiment order robustness

NoiseExperiment studies how noisy local information and actuator response degrade collective order. It reuses the shared ClassicBoids separation/alignment/cohesion update, then applies deterministic perturbations controlled by the simulation seed and `noise_seed_offset`:

- `perception_noise_strength` perturbs perceived neighbor offsets and alignment directions before the flocking sums are evaluated.
- `steering_noise_strength` perturbs each boid's final steering acceleration before force clamping.
- `velocity_noise_strength` optionally perturbs velocity after integration, followed by the usual speed clamp.
- `noise_enabled` can disable the perturbation path; with all strengths at zero, matched ClassicBoids parameters produce the same trajectory.

The core macroscopic metrics remain the main observables: `polarization`, `dispersion`, `cohesion`, and `average_neighbors`. CSV exports also include `noise_strength` and `order_loss`, where `order_loss = 1 - polarization`, so summary sweeps can directly compare noise strength vs. polarization.

Presets are `noise_baseline`, `noise_low`, `noise_medium`, and `noise_high`. A suggested experiment is a one-parameter summary sweep of noise strength against polarization:

```bash
./build/local/bin/flock3d_experiment_runner \
  --preset noise_baseline \
  --seed 7901 \
  --boids 1024 \
  --duration 45 \
  --fixed-dt 0.008333 \
  --sample-rate 5 \
  --export-mode summary \
  --sweep steering_noise_strength=0:0.5:0.05 \
  --output outputs/noise_steering_vs_polarization.csv
```

Repeat the same command with `--sweep perception_noise_strength=0:0.5:0.05` to compare perception noise against steering noise while keeping the initial seed and timestep fixed.

## FishSchool experiments

FishSchool uses the same separation/alignment/cohesion observables as ClassicBoids, with additional resistive-medium parameters for underwater-style motion:

- `drag_coefficient` dampens velocity every simulation step.
- `buoyancy_strength` adds positive-`y` acceleration.
- `target_depth`, `depth_band`, and `depth_correction_strength` keep the school near a preferred depth band.
- `current_strength` and `current_direction` add an optional constant current influence.
- `max_turn_rate` limits heading changes for smoother turns.

FishSchool exports include `mean_depth`, `depth_variance`, and `average_speed` alongside polarization, cohesion, dispersion, neighbor, and timing metrics. Presets are `fish_baseline`, `fish_high_drag`, `fish_strong_current`, and `fish_low_visibility`; CLI values and sweeps still apply after preset defaults.

Example commands:

```bash
./build/local/bin/flock3d_experiment_runner \
  --preset fish_baseline \
  --duration 30 \
  --sample-rate 5 \
  --output outputs/fish_baseline.csv

./build/local/bin/flock3d_experiment_runner \
  --preset fish_baseline \
  --sweep drag_coefficient=0.1:0.9:0.2 \
  --duration 45 \
  --export-mode summary \
  --output outputs/fish_drag_sweep.csv

./build/local/bin/flock3d_experiment_runner \
  --preset fish_strong_current \
  --sweep current_strength=0:4:1 \
  --seed 3109 \
  --boids 1024 \
  --output outputs/fish_current_sweep.csv
```
