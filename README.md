# flock3d

`flock3d` is a modern C++20 real-time 3D collective-behavior simulation focused on clean architecture, deterministic simulation, spatial partitioning, and performance-oriented design. The v2 foundation keeps the classic 3D boids model intact while adding scientific scenario definitions, reproducible seeds, quantitative flock metrics, the physically constrained Bird Flight scenario, Fish School resistive-medium flocking, and a deterministic Noise Experiment for collective-order robustness so future predator/prey, obstacle, and leadership models can be isolated as scenarios instead of ad-hoc flags.

## Features

- **Modern C++20** with small, focused translation units.
- **Modular CMake + FetchContent** for cross-platform dependency management.
- **raylib** rendering with a free 3D camera, compact debug overlay, FPS counter, and velocity-oriented boid tetrahedrons.
- **Catch2** tests for spatial hashing, wrapping, and fixed-timestep behavior.
- **Deterministic fixed timestep** simulation loop at 120 Hz.
- **Simulation/rendering separation** so flock behavior can evolve without coupling to drawing code.
- **Data-oriented SoA storage** for boid positions, velocities, and accelerations.
- **Runtime-tunable flock parameters** for separation, alignment, cohesion, perception radius, max speed, boid count, and Bird Flight constraint sweeps such as gravity, turn rate, field of view, and altitude correction, plus Fish School sweeps such as drag, depth correction, target depth, and current strength, and Noise Experiment sweeps such as perception and steering noise.
- **Scenario definitions** with display text, default simulation parameters, environment settings, constraints, behavior settings, metric settings, and deterministic seeds.
- **Reproducible seed handling** so resetting a scenario with the same seed recreates the same initial boid positions and velocities.
- **Lightweight metrics instrumentation** for simulation step time, render time, spatial hash occupancy, neighbor-query behavior, collective-behavior order parameters, and Bird Flight altitude/stall observables, plus Fish School depth/speed observables and Noise Experiment order-loss observables.
- **Sampled CSV metrics export** for reproducible experiments without per-frame dumps.
- **Headless experiment runner** (`flock3d_experiment_runner`) for deterministic scripted runs and one-parameter sweeps without opening a raylib window.
- **Configurable directional boid scale** via simulation parameters.
- **Uniform 3D spatial hash** for allocation-light neighbor queries during the simulation step.
- **No ECS framework** and no unnecessary inheritance.

## Build instructions

### Prerequisites

- CMake 3.24 or newer
- A C++20 compiler:
  - MSVC 2022+
  - Clang 14+
  - GCC 11+
- `VCPKG_ROOT` set to a vcpkg checkout for the provided presets
- Ninja is optional and only needed when using the `debug-ninja` or `release-ninja` presets

### Configure, build, and test

```bash
./scripts/build.sh
./scripts/tests.sh
```

The scripts use the `debug` preset by default. Pass another preset name as the first argument, or set `CMAKE_PRESET`, to use a different preset:

```bash
./scripts/build.sh release
CMAKE_PRESET=debug-vs2022 ./scripts/tests.sh
```

### Run

```bash
./scripts/run.sh
```

Pass a preset name before `--` to run a different build, and pass any app arguments after `--`:

```bash
./scripts/run.sh release --
```

On Windows, the executable is typically located at:

```powershell
.\build\debug\Debug\flock3d.exe
```

## Architecture overview

```text
include/flock3d/
  sim/      Public simulation headers for flock3d::core consumers.
  math/     Public lightweight Vector3 helpers used by simulation code.
src/
  app/         Window ownership, camera setup, and the fixed-timestep main loop.
  sim/         Core simulation implementation and CMake target definition.
  render/      raylib rendering implementation and CMake target definition.
  experiment/  CSV metrics export and the headless experiment runner.
tests/         Catch2 coverage and test target definition.
```

CMake target ownership is intentionally local to each subdirectory: the root `CMakeLists.txt` configures the project, shared modules, and dependency fetching, while `src/sim`, `src/render`, `src/app`, and `tests` each declare the targets they own. Public core headers live under `include/flock3d` so external consumers can include stable paths such as:

```cpp
#include <flock3d/sim/BoidSimulation.hpp>
```

The simulation owns the boid data in a structure-of-arrays layout:

```cpp
std::vector<Vector3> positions;
std::vector<Vector3> velocities;
std::vector<Vector3> accelerations;
```

This keeps the initial implementation simple while preserving a path toward cache-friendly update passes, batched neighbor queries, and future SIMD-friendly code.

## v2 scientific scenarios

The scenario system is deliberately plain-data oriented. `ScenarioType` selects a `ScenarioDefinition`, and the factory applies scenario defaults to `BoidSimulation`. **Classic Boids** is the unconstrained baseline. **Bird Flight** adds scenario-specific perception and flight limits. **Fish School** reuses the shared separation/alignment/cohesion rules inside a resistive underwater-style medium with drag, depth preference, smooth turning, and optional current. **Noise Experiment** also reuses the shared flocking rules, then adds deterministic perception, steering, and velocity perturbations so controlled noise sweeps can quantify how collective order degrades. The remaining scenarios expose distinct names, descriptions, parameters, and seeds while reusing classic boid steering until their models are implemented.

Current scenarios:

- **Classic Boids**: baseline Reynolds-style separation, alignment, and cohesion in a wrapped 3D world.
- **Bird Flight**: gravity-aware flocking with upward lift, altitude hold, minimum airspeed, climb-rate limits, turn-rate limits, and a forward field of view.
- **Fish School**: underwater-style schooling with lower default speeds, velocity drag, buoyancy, target-depth correction, smooth turn-rate limiting, and optional constant current flow.
- **Predator-Prey**: placeholder for future predator/prey roles and pursuit/evasion.
- **Obstacle Avoidance**: placeholder for future obstacle fields and avoidance responses.
- **Leadership**: placeholder for future informed-leader experiments.
- **Noise Experiment**: collective-order robustness study with deterministic noisy perception, steering acceleration, and optional post-integration velocity perturbations.

Each scenario stores its default seed. Switching scenarios applies its default parameters and resets the simulation from that scenario seed. Pressing reset recreates the current scenario state with the current seed; pressing the randomize-seed control assigns a new seed and resets immediately.


### Bird Flight constraints

Bird Flight is intentionally simple rather than aerodynamically realistic. It is meant to study how gravity, limited perception, and limited maneuverability change flock stability relative to Classic Boids. The scenario keeps the existing separation, alignment, and cohesion rules, then adds these constraints through `SimulationParameters`:

- `gravity`: downward acceleration applied every fixed simulation step.
- `lift_strength`: baseline upward acceleration that can counter gravity.
- `altitude_target` and `altitude_band`: define a preferred vertical band.
- `altitude_correction_strength`: proportional upward/downward acceleration applied only outside the altitude band.
- `min_speed`: minimum sustained velocity magnitude used as a simple stall-prevention constraint.
- `max_climb_rate`: cap on vertical velocity.
- `max_turn_rate`: maximum heading change in degrees per second.
- `field_of_view_degrees`: forward cone used to ignore neighbors behind or outside the bird's perception.

The same model runs in the interactive viewer, CSV recorder, headless runner, and one-parameter sweep path.

### Noise Experiment robustness

Noise Experiment is meant to answer: how much noisy perception or steering can a flock tolerate before macroscopic order breaks down? It keeps Classic Boids separation, alignment, and cohesion as the baseline, then applies deterministic noise controlled by these `SimulationParameters`:

- `perception_noise_strength`: perturbs perceived neighbor offsets and alignment directions before separation/alignment/cohesion are computed.
- `steering_noise_strength`: perturbs the final steering acceleration before the force clamp.
- `velocity_noise_strength`: optionally perturbs velocity after integration, then clamps to `max_speed`.
- `noise_seed_offset`: separates the deterministic noise stream from the initial-state seed.
- `noise_enabled`: toggles the perturbation path; zero strengths reproduce Classic Boids behavior for matched classic parameters.

Use the built-in collective metrics (`polarization`, `dispersion`, `cohesion`, and `average_neighbors`) plus `noise_strength` and `order_loss = 1 - polarization` to study noise strength vs. polarization. A typical run sweeps `perception_noise_strength` or `steering_noise_strength` and exports summary rows for each value:

```bash
./build/local/bin/flock3d_experiment_runner \
  --preset noise_baseline \
  --seed 7901 \
  --boids 1024 \
  --duration 45 \
  --fixed-dt 0.008333 \
  --sample-rate 5 \
  --export-mode summary \
  --sweep perception_noise_strength=0:0.5:0.05 \
  --output outputs/noise_perception_vs_polarization.csv
```

## What are boids?

Boids are a classic model of emergent flocking behavior. Each agent follows a few local steering rules:

1. **Separation**: avoid crowding nearby agents.
2. **Alignment**: steer toward the average heading of neighbors.
3. **Cohesion**: steer toward the center of nearby agents.

`flock3d` implements those local forces and exposes their weights at runtime so you can see how each rule changes the flock while watching the accompanying performance metrics.


## Spatial hash performance diagnostics

The debug overlay separates expected neighbor-query volume from spatial-hash efficiency. The query count should match the boid count during each simulation step because every boid performs exactly one neighbor query; a constant query count is therefore expected and is not by itself a lag signal.

The key metric is **candidates per query**: it counts how many boid entries the spatial hash had to test from visited cells before the boid rules accept effective neighbors. High candidate counts, high maximum cell occupancy, or very dense occupied cells point to a hash or parameter bottleneck even when the effective neighbor count remains modest. In 3D, increasing perception radius expands the visited cell volume rapidly, so large perception radii can multiply checked cells and candidates much faster than they would in a 2D setup.

Use these diagnostics first to distinguish render-bound frames from simulation-bound frames and to tune cell size, boid density, and perception radius. Scenario defaults, radius sweeps, and runtime radius controls keep `spatial_cell_size` synchronized to the effective query radius (`max(neighbor_radius, separation_radius)`) so a small perception-radius increase does not accidentally expand each 3D query from 27 visited cells to 125. Parallelization should come after these measurements identify the bottleneck, not before.

## Controls

- `W/A/S/D` + mouse: free camera controls through raylib.
- `P`: pause or resume fixed-timestep simulation updates.
- `R`: reset the simulation with the current boid count and seed.
- `N`: randomize the current scenario seed and reset.
- `,` / `.`: switch to the previous or next scenario, applying that scenario's default parameters and seed.
- `F1`: toggle the debug/metrics overlay.
- `Mouse wheel`: adjust camera movement speed.
- `Up` / `Down`: scroll the debug/metrics overlay when its content is taller than the window; hold `Shift` for larger jumps.
- `Home` / `End`: jump to the top or bottom of the debug/metrics overlay.
- `+` / `-` or keypad `+` / `-`: increase or decrease boid count in batches of 128.
- `Tab` / `Shift+Tab`: cycle the selected tunable parameter.
- `Left` / `Right` or `[` / `]`: decrease or increase the selected tunable parameter.
- `1`-`8`: select separation weight, alignment weight, cohesion weight, perception radius, separation radius, max speed, max force, or boid scale.
- `O`: start or stop CSV metrics recording under `outputs/`.
- `M`: cycle export mode when recording is stopped (`Summary`, `SampledTimeSeries`, placeholder `FullTrajectory`).
- `PageUp` / `PageDown`: increase or decrease the sampled metrics rate when recording is stopped.
- `Esc`: close the window.


## FishSchool resistive-medium model

FishSchool is a distinct underwater-style model for studying how drag, depth preference, and currents affect collective order and cohesion in 3D. It reuses shared separation, alignment, and cohesion, then applies FishSchool-specific constraints each step:

- `drag_coefficient`: velocity damping from medium resistance.
- `buoyancy_strength`: constant positive-`y` acceleration.
- `target_depth`: preferred `y` coordinate; default FishSchool values use negative `y` as deeper water.
- `depth_band`: tolerance around the target before correction is applied.
- `depth_correction_strength`: acceleration back toward the target-depth band.
- `current_strength`: optional constant current influence magnitude.
- `current_direction`: current vector direction before normalization.
- `max_turn_rate`: heading-change limit in degrees per second for smoother, slower motion than ClassicBoids.

FishSchool metrics are `mean_depth`, `depth_variance`, and the shared `average_speed`. These appear in the overlay when FishSchool is active and are included in sampled CSV and summary exports.

Example FishSchool experiments:

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


## Python CSV analysis

CSV exports are intended to feed lightweight offline analysis instead of adding plotting code to the C++ application. Install the Python plotting dependencies with:

```bash
python3 -m pip install pandas matplotlib
```

Plot any sampled metric against another CSV column, such as polarization over simulation time:

```bash
python3 scripts/plot_metric.py \
  --input outputs/run.csv \
  --x simulation_time \
  --y polarization \
  --output outputs/polarization.png
```

Compare one-parameter sweep exports by grouping rows by `sweep_value`, computing the mean metric for each sweep value, and plotting the result:

```bash
python3 scripts/compare_sweeps.py \
  --input outputs/noise_sweep.csv \
  --sweep-column sweep_value \
  --metric polarization \
  --output outputs/noise_vs_polarization.png
```

Suggested first analyses:

- Noise strength vs. polarization for NoiseExperiment order robustness.
- Gravity vs. altitude variance for BirdFlight stability.
- Drag vs. cohesion for FishSchool compactness.

## Metrics and experiments

`flock3d` records sampled macroscopic metrics rather than per-frame dumps. The interactive app writes CSV files under `outputs/`, and `flock3d_experiment_runner` provides deterministic headless runs, one-parameter sweeps, and BirdFlight and FishSchool presets for comparing flight-stability and resistive-medium schooling behavior. See [docs/experiments.md](docs/experiments.md) for the CSV schema, export modes, BirdFlight stability metrics, FishSchool depth metrics, preset lists, and example experiment commands.

## Debug overlay and metrics

The `F1` overlay is intentionally compact and rendered with raylib text primitives so it can stay visible during performance experiments. It refreshes cached text at a low rate, or immediately after input changes, to avoid unnecessary formatting work every frame.

Displayed metrics include:

- **Active scenario and seed**: the current scientific scenario name and the seed used for reproducible initialization.
- **FPS and frame time**: current render-loop throughput and frame duration.
- **Boid count**: active agents in the simulation after runtime adjustments.
- **Average neighbors per boid**: effective flock neighbors, excluding the boid itself, divided by boid count for the latest fixed simulation step.
- **Polarization / global alignment order**: magnitude of the average normalized velocity vector; values near `1.0` indicate aligned motion and values near `0.0` indicate disordered or opposing headings.
- **Cohesion**: average distance from each boid to the flock center of mass.
- **Dispersion**: root-mean-square distance from each boid to the flock center of mass.
- **Average speed**: mean velocity magnitude across all boids.
- **Nearest-neighbor average distance**: average nearest observed neighbor distance gathered during the existing spatial-hash neighbor queries.
- **Mean altitude / altitude variance**: vertical center and spread of the flock, useful for Bird Flight altitude-hold studies.
- **FishSchool depth metrics**: `mean_depth` and `depth_variance` when the FishSchool scenario is active.
- **Stall count**: boids below `min_speed` after the latest update.
- **Near-ground count**: boids at or below the simple `y <= 0` ground reference.
- **Cluster count**: reserved as a TODO metric until connectivity semantics are defined for scenario-specific models.
- **Simulation update time**: wall-clock duration of the most recent fixed simulation update.
- **Render time**: wall-clock duration of the most recent 3D draw pass.
- **Spatial hash cell count**: occupied cells after the latest hash rebuild.
- **Neighbor queries**: number of spatial neighbor queries issued by the latest fixed simulation step.
- **Current flocking parameters**: live values for separation, alignment, cohesion, perception radius, separation radius, max speed, max force, boid scale, gravity, max turn rate, field of view, altitude correction strength, and FishSchool medium controls such as drag, buoyancy, target depth, depth correction, and current strength.

## Screenshots

Screenshots and capture notes for the debug overlay and larger flock scenarios will be added here as the visual sandbox evolves.

## Roadmap

- Refine predator/prey, obstacle-avoidance, leadership, and noise behavior models.
- Add deterministic scenario fixtures for simulation regression tests.
- Define cluster-count connectivity semantics for scientific metrics.
- Implement full-trajectory export when a compact trajectory schema is defined.
- Expand the spatial hash to support reusable neighbor buffers.
- Add benchmarks for hash construction and simulation update passes.
- Explore instanced rendering for larger flocks.
