# Architecture

`flock3d` is organized so the interactive renderer and the headless experiment runner exercise the same simulation code. The goal is to keep the portfolio demo visually compelling while making the scientific path deterministic and testable.

## Repository map

```text
include/flock3d/
  math/        Lightweight Vector3 helpers.
  sim/         Public simulation and scenario headers.
src/
  app/         Window ownership, camera setup, input, and fixed-timestep loop.
  sim/         Core boid simulation, scenario definitions, metrics, spatial hash.
  render/      raylib drawing and overlay rendering.
  experiment/  CSV export and deterministic headless experiment runner.
tests/         Catch2 simulation, controls, experiment, and spatial-hash tests.
scripts/       Build/run/test helpers plus CSV plotting and study recipes.
```

## Separation of concerns

- **Simulation** owns boid state, scenario parameters, force evaluation, integration, metrics, and neighbor search.
- **Application code** owns window lifetime, camera movement, UI controls, scenario switching, and CSV recording triggers.
- **Rendering** reads simulation state and draws velocity-oriented boids plus the optional overlay.
- **Experiment code** runs the same simulation step loop without opening a window, then exports sampled or summarized metrics.

This split keeps visual iteration, deterministic studies, and automated tests from depending on each other unnecessarily.

## Scenario dispatch

Scenarios share a common parameter structure and public simulation interface. The implemented scenario set currently includes:

- `ClassicBoids` for baseline separation, alignment, and cohesion.
- `BirdFlight` for flight-like constraints such as lift, gravity, altitude correction, turn limits, and field of view.
- `FishSchool` for drag, buoyancy, target-depth behavior, smooth turning, and current influence.
- `NoiseExperiment` for deterministic perturbations in perception, steering, and velocity.

Placeholder scenario names reserve future model space without complicating the implemented scenarios.

## Spatial indexing

Production simulation updates still use the existing uniform 3D `SpatialHash3D` rather than all-pairs checks. Boids are inserted into cells derived from position and queried by nearby cells, which keeps local-interaction scans compact for the flock sizes targeted by the renderer and study scripts.

An experimental `SpatialGrid3D` now lives beside the hash as a sparse sorted grid with contiguous entries, cell ranges, and aggregates. It is not selected by the simulation by default yet; the first phase is equivalence testing and side-by-side benchmarking before any migration decision changes behavior.

## Deterministic stepping

The app advances the simulation with a fixed timestep, and the experiment runner exposes the same `fixed_dt` as a command-line option. Interactive input is polled once per rendered frame into a command queue; queued commands are drained and applied at frame or fixed-tick boundaries before simulation updates. This keeps event polling decoupled from stepping and prevents scenario, seed, boid-count, pause, and tuning changes from mutating simulation state mid-step. Determinism depends on keeping the scenario, seed, boid count, timestep, duration, export cadence, preset, and swept parameter values identical between runs.

## Data flow

```text
Scenario defaults + preset + overrides
        ↓
Simulation state initialization
        ↓
Frame event polling → queued control commands
        ↓
Frame/tick boundary command application
        ↓
Fixed-timestep update loop
        ↓
Metrics sampling
        ↓
CSV export or live overlay
        ↓
Python plotting scripts
```

## CPU update threading

`BoidSimulation` keeps the spatial hash build as a single pre-update phase, then treats the completed hash, boid positions, velocities, and cell aggregates as read-only while update workers evaluate per-boid steering. Each worker owns its temporary neighbor and aggregate buffers and writes only the acceleration, velocity, and position slot for the boid indices in its deterministic contiguous range. `thread_count = 1` keeps the serial path for reproducible debugging and low-overhead small runs; higher explicit values split the same ordered index space across CPU worker threads without changing neighbor ordering or using shared random generators.

`thread_count = 0` requests benchmark-informed automatic worker selection. The current deterministic policy uses 1 worker below 512 boids, 2 workers for 512-1023 boids, and 4 workers at 1024 or more boids, then clamps the automatic result to at least 1, no more than reported hardware concurrency when available, and no more than 4. Explicit `thread_count > 0` values remain manual overrides and keep the existing validation/clamping behavior; 8+ workers are still available for experiments but are not selected automatically because recent default-path benchmarks do not justify them yet. `BoidSimulation::effective_thread_count()` reports the worker count actually used after auto selection and clamping.

Deterministic noise remains a pure function of boid index, channel, seed offset, and simulation step, so it is independent of worker scheduling. Metrics collection currently uses the serial update path because `SimulationMetrics` is an aggregate recorder; use `metrics = nullptr` for the fully threaded benchmark path.

Use `thread_count = 0` for the default CPU policy, or choose explicit counts such as `1`, `2`, `4`, and `8` when benchmarking a target machine. Small flocks or diagnostic runs can be faster with one worker because dispatch overhead may dominate. Larger CPU-only headless updates should benefit from more workers until memory bandwidth, spatial-query cost, or scheduling overhead becomes the bottleneck.

GPU acceleration remains out of scope: the current architecture is intentionally CPU-first, keeps raylib rendering separate from simulation updates, and relies on deterministic CPU data structures and tests. Adding GPU compute would require a separate data layout, synchronization model, and validation strategy rather than a small update-phase parallelization.
