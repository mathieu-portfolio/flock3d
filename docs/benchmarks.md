# Benchmarking flock3d

These benchmark executables are for performance diagnosis before optimization. They are intentionally separate from the experiment runner and scientific CSV exports: experiments answer behavior questions, while benchmarks show where single-threaded time is spent.

## Why the benchmark sizes are small

Current simulations can become slower over time because boids cluster, increasing local neighbor density and the cost of candidate filtering and steering updates. The focused benchmarks therefore use boid counts below 512 by default (`64`, `128`, `256`, and `384`). This keeps runs practical while still making time-series slowdown visible.

Focused benchmarks advance fixed simulation ticks as fast as the CPU allows, sample at a regular simulated-time cadence, and print one CSV row per simulated sample window. The configured duration answers “how expensive is simulating X seconds of behavior?” rather than forcing each scenario to run for X wall-clock seconds. Comparing early, middle, and late rows helps identify whether degradation is caused by clustering, spatial hash behavior, neighbor filtering, metrics, model logic, or deterministic noise generation.

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

The focused benchmarks accept the same lightweight simulated-time options:

```bash
--duration seconds   # simulated duration per scenario, default 20
--sample seconds     # simulated CSV sample window length, default 5
--warmup seconds     # simulated warm-up per scenario, default 1
```

CSV is printed to stdout so output is easy to redirect. Progress bars are printed to stderr and are automatically disabled unless stderr is a terminal, so the helper script can redirect stdout to CSV files while still showing simulated-time progress in your terminal.

Example progress display:

```text
[##########----------]  50% | simulation_update | ClassicBoids | 256 boids | 15.0s / 20s
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

Measures total `BoidSimulation::update` cost for these models while collecting the neighbor diagnostics needed to compare bounded and unbounded neighbor processing:

- `ClassicBoids`
- `BirdFlight`
- `FishSchool`
- `NoiseExperiment`

The benchmark runs four neighbor modes for each model and boid count:

- `fixed_radius_uncapped`: classic metric-neighbor behavior. Every visible boid inside the fixed perception radius contributes to alignment, cohesion, and separation. This is the fixed-radius baseline and remains available for regression comparisons.
- `fixed_radius_closest_k`: fixed metric perception radius, followed by deterministic closest-K topological selection. Candidates are ordered by squared distance with boid index as the tie-breaker, and only `max_selected_neighbors` are used for steering.
- `adaptive_radius_closest_k`: first queries a broad perception range, estimates local density, adapts the effective radius, then applies closest-K selection. The radius formula is `base_perception_radius * sqrt(target_neighbor_count / max(local_candidate_count, 1))`, clamped between `min_perception_radius` and `max_perception_radius`. Dense neighborhoods shrink toward the minimum; sparse neighborhoods expand toward the maximum.
- `cell_aggregate_social`: uses exact individual boids only for close-range separation, then uses cached occupied-cell centroids for cohesion and cached occupied-cell average velocities for alignment. Each social cell is weighted by its boid count after excluding the current boid from its own cell. This keeps collision/near-overlap avoidance exact while reducing dense-cluster social steering from per-boid iteration to per-cell iteration. It differs from closest-K because closest-K still scans and ranks individual candidates for all social forces; cell aggregates replace social individual neighbors with local cell summaries. The first implementation uses a fixed social perception radius and optional model FOV filtering for bird/fish social cells; distance falloff and aggregate-specific FOV weighting are future work.

Adaptive perception changes which neighbors are sensed before topological selection. Adaptive separation is intentionally **not** part of this refactor; separation continues to use the selected-neighbor list and the existing fixed `separation_radius` gate. In `cell_aggregate_social`, separation uses its own exact fixed `separation_radius` query and 360° perception, independent of the aggregate social query.

Columns:

```text
scenario,model,neighbor_mode,adaptive_perception_enabled,boid_count,elapsed_seconds,sample_index,iterations_in_sample,base_perception_radius,effective_radius_mean,selected_neighbors_mean,candidates_per_query,effective_neighbors_per_query,exact_separation_neighbors_mean,aggregate_cells_used_mean,social_weight_sum_mean,polarization,flock_spread,nearest_neighbor_distance,average_speed,mean_update_ms,min_update_ms,max_update_ms
```

To compare all neighbor modes directly, including `cell_aggregate_social`, run:

```bash
scripts/run_benchmark.sh simulation_update
```

For a quick local comparison while tuning neighbor parameters, shorten the run:

```bash
scripts/run_benchmark.sh --duration 0.5 --sample 0.25 --warmup 0 simulation_update
```


### `flock3d_simulation_ticks_benchmark`

Runs deterministic accelerated simulation benchmarks that answer “how long does this machine take to simulate X seconds of boids?” instead of running for X wall-clock seconds. It advances `BoidSimulation::update(dt)` directly in a tight loop, independent of rendering, input polling, progress bars, and the application fixed-timestep accumulator. Warm-up ticks are executed first and excluded from all measured statistics.

Defaults:

- boid counts: `128`, `256`, `512`, `1024`
- fixed `dt`: `1/60` seconds
- warm-up: `5` simulated seconds
- measured duration: `25` simulated seconds
- repetitions per count: `1`
- fixed seed: `12345` plus the boid count, reused for each repetition of that count

Columns:

```text
scenario,boid_count,repetition,seed,dt,warmup_seconds,warmup_ticks,simulated_seconds,measured_ticks,total_wall_seconds,average_ms_per_tick,p50_ms_per_tick,p95_ms_per_tick,max_ms_per_tick,ticks_per_second,real_time_factor
```

Run the default accelerated benchmark through the helper script:

```bash
scripts/run_benchmark.sh simulation_ticks
```

Run a smoke test that simulates `0.5` warm-up seconds plus `1.0` measured seconds at 60 Hz for two boid counts:

```bash
scripts/run_benchmark.sh simulation_ticks -- --counts 128,256 --warmup 0.5 --measured 1 --dt 0.0166666667 --repetitions 1
```

The legacy `--duration` option is accepted as an alias for `--measured` when forwarded by `scripts/run_benchmark.sh`; `--sample` is accepted and ignored because this benchmark emits compact one-row summaries per repetition rather than time-series sample windows.

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

Rows with the same scenario/model/mode and boid count are ordered by `sample_index`, and `elapsed_seconds` is simulated elapsed time. Inspect how `mean_update_ms`, `max_update_ms`, candidate counts, effective neighbors, exact separation neighbors, aggregate cells used, social weight, flock spread (`flock_spread`), nearest-neighbor distance, polarization, and average speed change from early to late windows. If update time rises with effective neighbors and max occupancy, clustering and neighbor filtering are likely first optimization targets. If rebuild time grows independently of neighbor counts, focus on spatial hash construction. If the metrics or noise benchmark shows a large delta from its baseline mode, optimize that path before parallelizing.
