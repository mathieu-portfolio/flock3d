# Benchmarking flock3d

These benchmark executables are for performance diagnosis before optimization. They are intentionally separate from the experiment runner and scientific CSV exports: experiments answer behavior questions, while benchmarks show where single-threaded time is spent.

## CPU thread-scaling focus

Current threaded measurements show useful scaling only up to a conservative number of workers: 2 threads can help around 512 boids, 4 threads is the best default from roughly 1024 through 10k boids, and 8/16 threads are not default yet because they have remained slower than 4 on recent runs. The shrinking gap at larger boid counts suggests fixed scheduling overhead matters less as flocks grow, while memory/cache bandwidth, spatial-hash access locality, false sharing, or serial bookkeeping may become the next bottleneck. GPU work remains out of scope until the CPU path has enough diagnostics to show whether thread scheduling, load imbalance, cache contention, serial metrics, or bandwidth is limiting scaling.

The simulation update path now uses a reusable CPU executor for threaded updates instead of creating worker threads every update, reuses per-worker scratch buffers for neighbor candidates and aggregate-cell work, and keeps `thread_count=1` on the serial path. `thread_count=0` selects a conservative automatic policy based on current benchmark evidence: fewer than 512 boids uses 1 thread, around 512 boids uses 2 threads, and 1024 or more boids uses 4 threads, capped at 4 for now. Manual thread counts are still respected for benchmark investigations.

## Benchmark sizes

Current simulations can become slower over time because boids cluster, increasing local neighbor density and the cost of candidate filtering and steering updates. The focused benchmarks now use compact defaults for quick local iteration: one boid count (`512`), one worker count (`1`), a one-second simulated duration, and a short warm-up. Pass explicit filters when you need a focused matrix, or pass `--full-matrix` to restore the older broad timing window, thread-count, and boid-count defaults (`512`, `1024`, `2048`, `5096`, and `10192`). Hardware concurrency is **not** appended automatically; pass `--hardware-threads` when you want to include this machine's `std::thread::hardware_concurrency()` value.

Focused benchmarks advance deterministic fixed simulation ticks as fast as the CPU allows, sample at a regular simulated-time cadence, and print one CSV row per simulated sample window. The configured duration answers “how expensive is simulating X seconds of behavior?” rather than forcing each scenario to run for X wall-clock seconds. CSV output therefore makes simulated time (`elapsed_seconds`/`simulated_seconds` and `simulated_ticks`) separate from measured CPU time (`sample_wall_seconds`/`total_wall_seconds`). Prefer the benchmark-specific primary timing column (`mean_update_ms`, `mean_ns_per_tick`, or `mean_spatial_query_ns_per_tick`) when comparing runs; legacy millisecond columns remain where they make the compact CSV easier to read. Comparing early, middle, and late rows helps identify whether degradation is caused by clustering, spatial hash behavior, neighbor filtering, metrics, model logic, or deterministic noise generation.

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

The focused benchmarks accept the same lightweight simulated-time and filtering options:

```bash
--models BirdFlight,FishSchool      # model/scenario filters where the target supports model selection
--modes adaptive_radius_closest_k   # benchmark mode filters; names are validated per target
--counts 512,1024,2048              # comma-separated boid counts; default 512
--threads 1,2,4,8                   # comma-separated CPU worker counts; default 1
--hardware-threads                  # append std::thread::hardware_concurrency() explicitly
--duration seconds                  # simulated duration per scenario; default 1
--sample seconds                    # simulated CSV sample window length; default 1
--warmup seconds                    # simulated warm-up per scenario; default 0.25
--seed integer                      # random seed override for each fresh simulation
--chunk-size boids                  # optional deterministic dynamic chunk size; 0 keeps one contiguous range per worker
--full-matrix                       # restore the older broad duration/sample/warmup/count/thread defaults
```

Common simulation-parameter overrides are also available when they map directly to `SimulationParameters`:

```bash
--world-size 60                     # world half extent
--neighbor-radius 5                 # metric neighbor radius
--perception-radius 5               # base/min/max perception radius override
--separation-radius 1.5
--max-speed 8
--max-force 10
--max-selected-neighbors 48
--target-neighbor-count 32
--adaptive-perception on            # accepts on/off, true/false, or 1/0
```

Invalid list entries, booleans, model names, or mode names print usage and fail instead of silently changing the run matrix.

CSV is printed to stdout so output is easy to redirect. Progress bars are printed to stderr and are automatically disabled unless stderr is a terminal, so the helper script can redirect stdout to CSV files while still showing simulated-time progress in your terminal. If you are running from an IDE, task runner, CI log, or another captured output pane where stderr is not detected as a terminal, set `FLOCK3D_BENCHMARK_PROGRESS=always` to force the progress bar; use `FLOCK3D_BENCHMARK_PROGRESS=never` to silence it.

Example progress display:

```text
[##########----------]  50% | simulation_update | ClassicBoids | 256 boids | 15.0 simulated s / 20 simulated s
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
scripts/run_benchmark.sh aggregate_social
```

For a quick smoke run while changing benchmark code, keep the matrix tiny and shorten the duration and sample window:

```bash
scripts/run_benchmark.sh --duration 0.2 --sample 0.1 --warmup 0 simulation_update -- --models ClassicBoids --modes adaptive_radius_closest_k --counts 128 --threads 1
```

For a thread-scaling run, explicitly request only the worker counts and boid counts you want to compare:

```bash
scripts/run_benchmark.sh simulation_update -- --models ClassicBoids --modes adaptive_radius_closest_k --counts 512,1024,2048 --threads 1,2,4,8 --duration 2 --sample 0.5
```

For a focused model/mode comparison, combine model and mode filters:

```bash
scripts/run_benchmark.sh simulation_update -- --models BirdFlight,FishSchool --modes adaptive_radius_closest_k,aggregate_social --counts 1024 --threads 1,4
```

For an exhaustive run comparable to the older broad local matrix, opt in explicitly:

```bash
scripts/run_benchmark.sh simulation_update -- --full-matrix
```

For a simulation-parameter override run, pass only overrides that correspond to existing simulation parameters:

```bash
scripts/run_benchmark.sh aggregate_social -- --models ClassicBoids --modes cell_aggregate_social_adaptive_radius --counts 1024 --threads 1 --seed 12345 --world-size 60 --neighbor-radius 5 --separation-radius 1.5 --max-speed 8 --max-force 10 --max-selected-neighbors 48 --target-neighbor-count 32 --adaptive-perception on --chunk-size 256
```

Use `--preset`, `--output-dir`, or the `FLOCK3D_BENCHMARK_*` environment variables when you need a different build preset, destination directory, or default timing options. Arguments after `--` are forwarded directly to the benchmark executable:

```bash
scripts/run_benchmark.sh --preset release-ninja --output-dir outputs/benchmarks/nightly all -- --duration 10
scripts/run_benchmark.sh simulation_update -- --threads 1,2,4,8 --counts 128,256 --duration 2 --sample 0.5
scripts/run_benchmark.sh simulation_update -- --hardware-threads --threads 1,2,4
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

The plotting wrapper writes summary CSV files and PNGs under `outputs/benchmarks/plots/` by default. It creates latest-sample scaling plots, simulated-time plots for each target's available timing metrics, and diagnostic plots for candidate counts, visited-cell counts, topology truncation, aggregate-cell work, and cell occupancy. Use `--input-dir`, `--output-dir`, `--format`, or `--benchmarks` when plotting a specific run or benchmark family:

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

./build/release/bin/flock3d_aggregate_social_benchmark \
  > outputs/benchmarks/aggregate_social.csv
```

## Which benchmark should I run?

- Use `simulation_update` for concise end-to-end model update timing across the main models and non-aggregate neighbor modes.
- Use `aggregate_social` for visibility-aware `cell_aggregate_social` cost and behavior diagnostics.
- Use `spatial_hash` for rebuild/query/cell occupancy questions.
- Use `metrics` for the cost of collecting `SimulationMetrics`.
- Use `noise` for deterministic noise-mode overhead.
- Use `simulation_ticks` for compact fixed-tick throughput summaries over larger counts.

## Benchmark targets

### `flock3d_simulation_update_benchmark`

Purpose: concise end-to-end `BoidSimulation::update` timing for the main simulation models. Run this benchmark when the question is “which model or neighbor-mode family is slower overall?” and not when you need detailed spatial-hash, metrics, noise, or aggregate-social diagnostics.

Models:

- `ClassicBoids`
- `BirdFlight`
- `FishSchool`
- `NoiseExperiment`

Neighbor modes:

- `fixed_radius_uncapped`
- `fixed_radius_closest_k`
- `adaptive_radius_closest_k`
- `aggregate_social`

Columns:

```text
scenario,model,neighbor_mode,boid_count,thread_count,worker_count_effective,boids_per_worker_mean,boids_per_worker_min,boids_per_worker_max,chunk_size,elapsed_seconds,sample_index,iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms,update_parallel_ms,integration_parallel_ms,serial_metrics_ms,speedup_vs_single_thread,random_seed,world_half_extent,neighbor_radius,separation_radius,max_speed,max_force,max_selected_neighbors,target_neighbor_count,adaptive_perception_enabled
```

Run it when you want a compact model/mode timing comparison:

```bash
scripts/run_benchmark.sh simulation_update
```

For a quick local comparison while tuning neighbor parameters, shorten the run:

```bash
scripts/run_benchmark.sh --duration 0.5 --sample 0.25 --warmup 0 simulation_update -- --threads 1,2,4
```

Run a scaling comparison for the currently tracked CPU sizes and modes by making the matrix explicit:

```bash
scripts/run_benchmark.sh simulation_update -- --models ClassicBoids,BirdFlight,FishSchool,NoiseExperiment --modes fixed_radius_uncapped,fixed_radius_closest_k,adaptive_radius_closest_k,aggregate_social --threads 1,2,4,8,16 --counts 512,1024,2048,5096,10192
```

Threaded rows are generated from fresh simulations with the same seed and parameters. Use `thread_count=1` as the deterministic baseline, then compare `mean_update_ms`, `min_update_ms`, `max_update_ms`, `worker_count_effective`, `boids_per_worker_*`, and `speedup_vs_single_thread` across worker counts. Empty `update_parallel_ms`, `integration_parallel_ms`, and `serial_metrics_ms` cells mean the benchmark timed the full update as a single unit for minimum measurement overhead. Try `--chunk-size 64` or `--chunk-size 128` to test whether smaller deterministic chunks help clustered flocks; if `boids_per_worker_min/max` are balanced but speedup flattens or regresses as worker count rises, suspect memory/cache bandwidth, false sharing, spatial-hash locality, or serial phases rather than simple range imbalance.



### `flock3d_aggregate_social_benchmark`

Purpose: isolate the cost and behavior impact of visibility-aware `cell_aggregate_social` steering. Run this benchmark when the question is “what does aggregate social steering cost, and how do FOV and adaptive social radius change aggregate-cell work and flock behavior?”

Compared modes:

- `cell_aggregate_social_no_fov`
- `cell_aggregate_social_fov`
- `cell_aggregate_social_adaptive_radius`
- `cell_aggregate_social_fov_adaptive_radius`

Columns:

```text
scenario,aggregate_social_mode,boid_count,thread_count,elapsed_seconds,sample_index,iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms,aggregate_social_enabled,social_fov_enabled,adaptive_social_radius_enabled,visible_aggregate_cells_mean,rejected_aggregate_cells_mean,aggregate_cells_used_mean,aggregate_query_radius_mean,aggregate_query_radius_min,aggregate_query_radius_max,exact_separation_neighbors_mean,exact_separation_neighbors_max,social_weight_sum_mean,flock_spread,polarization,random_seed,world_half_extent,neighbor_radius,separation_radius,max_speed,max_force,max_selected_neighbors,target_neighbor_count,adaptive_perception_enabled
```

Run it when tuning aggregate social visibility or adaptive-radius behavior:

```bash
scripts/run_benchmark.sh aggregate_social -- --modes cell_aggregate_social_no_fov,cell_aggregate_social_adaptive_radius --counts 512,1024 --threads 1,4
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
scenario,boid_count,repetition,seed,dt,warmup_seconds,warmup_ticks,simulated_seconds,measured_ticks,total_wall_seconds,average_ms_per_tick,p50_ms_per_tick,p95_ms_per_tick,p99_ms_per_tick,max_ms_per_tick,ticks_per_second,real_time_factor,ticks_in_sample,simulated_ticks,wall_seconds,mean_ns_per_tick,updates_per_second
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
scenario,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_rebuild_ms,min_rebuild_ms,max_rebuild_ms,mean_spatial_query_ms,min_spatial_query_ms,max_spatial_query_ms,mean_naive_query_ms,min_naive_query_ms,max_naive_query_ms,candidates_per_query,visited_cells_per_query,effective_neighbors_per_query,naive_neighbors_per_query,occupied_cell_count,max_cell_occupancy,average_cell_occupancy,count_mismatches,simulated_seconds,simulated_ticks,ticks_in_sample,sample_wall_seconds,mean_rebuild_ns_per_tick,p50_rebuild_ms,p95_rebuild_ms,p99_rebuild_ms,mean_spatial_query_ns_per_tick,p50_spatial_query_ms,p95_spatial_query_ms,p99_spatial_query_ms,mean_naive_query_ns_per_tick,p50_naive_query_ms,p95_naive_query_ms,p99_naive_query_ms,ticks_per_second,updates_per_second,real_time_factor
```

Use `candidates_per_query`, `visited_cells_per_query`, `effective_neighbors_per_query`, `occupied_cell_count`, `max_cell_occupancy`, and `average_cell_occupancy` to see whether late-run slowdown corresponds to clustering, larger query footprints, or worsening candidate density. `count_mismatches` should remain `0`; it compares spatial neighbor counts to naive counts for the same positions.

### `flock3d_metrics_benchmark`

Measures ClassicBoids update cost with and without a metrics pointer:

- `no_metrics`
- `metrics_pointer`

Columns:

```text
scenario,metric_mode,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms,simulated_seconds,simulated_ticks,ticks_in_sample,sample_wall_seconds,mean_ns_per_tick,p50_update_ms,p95_update_ms,p99_update_ms,ticks_per_second,updates_per_second,real_time_factor
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
scenario,noise_mode,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms,steering_noise_strength,perception_noise_strength,velocity_noise_strength,simulated_seconds,simulated_ticks,ticks_in_sample,sample_wall_seconds,mean_ns_per_tick,p50_update_ms,p95_update_ms,p99_update_ms,ticks_per_second,updates_per_second,real_time_factor
```

This benchmark helps decide whether deterministic noise generation should be optimized before model or spatial-hash changes.

## Interpreting time-series rows

Rows with the same scenario/model/mode and boid count are ordered by `sample_index`, and `elapsed_seconds` is simulated elapsed time kept for CSV compatibility. Use `simulated_seconds`/`simulated_ticks` where present for simulation progress, `ticks_in_sample`/`iterations_in_sample` for the deterministic work size, `sample_wall_seconds` where present for measured CPU elapsed time, the target-specific primary timing column for per-window cost, and `p50_*`/`p95_*` or throughput columns where those are part of the target schema. Inspect how tick cost, p95 latency, candidate counts, effective neighbors, exact separation neighbors, aggregate cells used, social weight, flock spread (`flock_spread`), nearest-neighbor distance, polarization, and average speed change from early to late windows in the benchmark targets that expose those diagnostics. If update time rises with effective neighbors and max occupancy, clustering and neighbor filtering are likely first optimization targets. If rebuild time grows independently of neighbor counts, focus on spatial hash construction. If the metrics or noise benchmark shows a large delta from its baseline mode, optimize that path before parallelizing.
