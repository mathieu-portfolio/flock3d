# Benchmarking flock3d

These benchmark executables are for performance diagnosis before optimization. They are intentionally separate from the experiment runner and scientific CSV exports: experiments answer behavior questions, while benchmarks show where single-threaded time is spent.

## Why the benchmark sizes are small

Current simulations can become slower over time because boids cluster, increasing local neighbor density and the cost of candidate filtering and steering updates. The focused benchmarks therefore use boid counts below 512 by default (`64`, `128`, `256`, and `384`). This keeps runs practical while still making time-series slowdown visible.

Each benchmark runs every scenario for a fixed wall-clock duration, samples at a regular cadence, and prints one CSV row per sample window. Comparing early, middle, and late rows helps identify whether degradation is caused by clustering, spatial hash behavior, neighbor filtering, metrics, model logic, or deterministic noise generation.

## Build

Use a release build for representative timings. The dedicated build preset compiles the focused benchmark targets without rebuilding the app or tests:

```bash
cmake --preset release
cmake --build --preset benchmark
```

Use `benchmark-ninja` with the `release-ninja` configure preset when you prefer Ninja:

```bash
cmake --preset release-ninja
cmake --build --preset benchmark-ninja
```

The executables are written to `build/release/bin/` for the default release preset.

## Common options

All focused benchmarks accept the same lightweight options:

```bash
--duration seconds   # timed duration per scenario, default 30
--sample seconds     # CSV sample window length, default 5
--warmup seconds     # untimed warm-up per scenario, default 1
```

CSV is printed to stdout so output is easy to redirect. Progress bars are printed to stderr and are automatically disabled unless stderr is a terminal, so the helper script can redirect stdout to CSV files while still showing progress in your terminal.

Example progress display:

```text
[##########----------]  50% | simulation_update | ClassicBoids | 256 boids | 15.0s / 30s
```

## Suggested first commands

The helper script builds the selected benchmark targets with a release preset and writes clean CSV files under `outputs/benchmarks/`:

```bash
scripts/run_benchmark.sh
```

Run one focused benchmark by name when you only need a single CSV:

```bash
scripts/run_benchmark.sh simulation_update
scripts/run_benchmark.sh spatial_hash
scripts/run_benchmark.sh metrics
scripts/run_benchmark.sh noise
```

For a quick smoke run while changing benchmark code, shorten the duration and sample window:

```bash
scripts/run_benchmark.sh --duration 0.2 --sample 0.1 --warmup 0 simulation_update
```

Use `--preset`, `--output-dir`, or the `FLOCK3D_BENCHMARK_*` environment variables when you need a different build preset, destination directory, or default timing options. Arguments after `--` are forwarded directly to the benchmark executable:

```bash
scripts/run_benchmark.sh --preset release-ninja --output-dir outputs/benchmarks/nightly all -- --duration 10
```

## Plot benchmark output

Install the same lightweight plotting dependencies used by the experiment scripts:

```bash
python -m pip install pandas matplotlib
```

Write a dependency-free latest-sample summary for benchmark CSV files already present under `outputs/benchmarks/`:

```bash
scripts/summarize_benchmarks.py --markdown
```

Generate plots for benchmark CSV files already present under `outputs/benchmarks/`:

```bash
scripts/plot_benchmarks.sh
```

The plotting wrapper writes summary CSV files and PNGs under `outputs/benchmarks/plots/` by default. It creates latest-sample scaling plots, elapsed-time plots for timing metrics, and spatial-hash diagnostic plots for candidate counts and cell occupancy. Use `--input-dir`, `--output-dir`, `--format`, or `--benchmarks` when plotting a specific run or benchmark family:

```bash
scripts/plot_benchmarks.sh --input-dir outputs/benchmarks/nightly --output-dir outputs/benchmarks/nightly_plots
scripts/plot_benchmarks.sh --benchmarks simulation_update,spatial_hash --format svg
```

You can also ask the benchmark runner to plot immediately after it writes CSV files:

```bash
scripts/run_benchmark.sh --duration 1 --sample 0.25 --summary --plot all
scripts/run_benchmark.sh --plot-arg --format --plot-arg svg spatial_hash
```

You can still run the executables directly if you need shell-specific redirection:

```bash
mkdir -p outputs/benchmarks

./build/release/bin/flock3d_simulation_update_benchmark \
  > outputs/benchmarks/simulation_update.csv

./build/release/bin/flock3d_spatial_hash_benchmark \
  > outputs/benchmarks/spatial_hash.csv

./build/release/bin/flock3d_metrics_benchmark \
  > outputs/benchmarks/metrics.csv

./build/release/bin/flock3d_noise_benchmark \
  > outputs/benchmarks/noise.csv
```

## Benchmark targets

### `flock3d_simulation_update_benchmark`

Measures total `BoidSimulation::update` cost with metrics disabled for these models:

- `ClassicBoids`
- `BirdFlight`
- `FishSchool`
- `NoiseExperiment`

Columns:

```text
scenario,model,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms
```

### `flock3d_spatial_hash_benchmark`

Measures spatial hash rebuild, spatial neighbor query, and naive neighbor counting separately while a deterministic ClassicBoids simulation evolves. It reports density/radius scenarios:

- `low_density`
- `baseline_density`
- `high_density`
- `small_radius`
- `large_radius`

Columns:

```text
scenario,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_rebuild_ms,min_rebuild_ms,max_rebuild_ms,mean_spatial_query_ms,min_spatial_query_ms,max_spatial_query_ms,mean_naive_query_ms,min_naive_query_ms,max_naive_query_ms,candidates_per_query,effective_neighbors_per_query,naive_neighbors_per_query,occupied_cell_count,max_cell_occupancy,average_cell_occupancy,count_mismatches
```

Use `candidates_per_query`, `effective_neighbors_per_query`, `occupied_cell_count`, `max_cell_occupancy`, and `average_cell_occupancy` to see whether late-run slowdown corresponds to clustering or worsening candidate density. `count_mismatches` should remain `0`; it compares spatial neighbor counts to naive counts for the same positions.

### `flock3d_metrics_benchmark`

Measures ClassicBoids update cost with and without a metrics pointer:

- `no_metrics`
- `metrics_pointer`

Columns:

```text
scenario,metric_mode,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms
```

The code currently exposes a null/non-null metrics pointer rather than separate basic/full metric levels, so this benchmark isolates the overhead of enabling the existing metrics path.

### `flock3d_noise_benchmark`

Measures `NoiseExperiment` update cost for deterministic noise combinations:

- `zero_noise`
- `steering_noise`
- `perception_noise`
- `velocity_noise`
- `all_noise`

Columns:

```text
scenario,noise_mode,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms,steering_noise_strength,perception_noise_strength,velocity_noise_strength
```

This benchmark helps decide whether deterministic noise generation should be optimized before model or spatial-hash changes.

## Interpreting time-series rows

Rows with the same scenario/model/mode and boid count are ordered by `sample_index`. Inspect how `mean_update_ms`, `max_update_ms`, candidate counts, effective neighbors, and cell occupancy change from early to late windows. If update time rises with effective neighbors and max occupancy, clustering and neighbor filtering are likely first optimization targets. If rebuild time grows independently of neighbor counts, focus on spatial hash construction. If the metrics or noise benchmark shows a large delta from its baseline mode, optimize that path before parallelizing.
