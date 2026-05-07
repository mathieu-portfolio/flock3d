# flock3d

Real-time 3D collective-behavior simulation in modern C++20.

`flock3d` is a portfolio-scale scientific graphics project: it renders hundreds to thousands of agents with raylib, advances them with a deterministic fixed-timestep simulation, and exports macroscopic flock metrics for repeatable experiments. The project starts from classic Reynolds boids, then layers scenario-specific constraints for flight, schooling, and noise-robustness studies without turning the simulator into a collection of ad-hoc flags.

## Why it is interesting

- **Visual**: velocity-oriented tetrahedron boids move through a wrapped 3D world with a free camera and compact metrics overlay.
- **Scientific**: scenarios expose reproducible seeds, controlled parameters, sampled CSV metrics, and deterministic headless runs.
- **Engineering**: the codebase keeps simulation, rendering, application, and experiment-runner concerns separated behind small CMake targets.

## Key features

- Modern **C++20** with CMake presets and FetchContent-based dependencies.
- **raylib** interactive renderer with free camera controls and a toggleable debug overlay.
- Deterministic **120 Hz fixed timestep** simulation loop.
- Structure-of-arrays boid storage for positions, velocities, and accelerations.
- Uniform 3D spatial hash for allocation-light neighbor queries.
- Scenario definitions for **Classic Boids**, **Bird Flight**, **Fish School**, **Noise Experiment**, and future scenario placeholders.
- Reproducible seed handling for resets, scenario changes, CSV recordings, and headless experiments.
- Sampled CSV metrics export and a deterministic `flock3d_experiment_runner` for scripted runs and one-parameter sweeps.
- Catch2 coverage for core simulation behavior.

## Quick start

### Prerequisites

- CMake 3.24+
- A C++20 compiler: MSVC 2022+, Clang 14+, or GCC 11+
- `VCPKG_ROOT` pointing to a vcpkg checkout for the provided presets
- Ninja for the `debug-ninja` and `release-ninja` presets

### Build, test, run

```bash
./scripts/build.sh
./scripts/tests.sh
./scripts/run.sh
```

Use a different preset by passing it to the helper scripts or setting `CMAKE_PRESET`:

```bash
./scripts/build.sh release
CMAKE_PRESET=debug-vs2022 ./scripts/tests.sh
./scripts/run.sh release --
```

On Windows, the default debug executable is typically:

```powershell
.\build\debug\Debug\flock3d.exe
```

## Screenshots and plots

Visual artifacts are intentionally kept as placeholders until the capture set is curated:

- `docs/assets/classic-boids.png` — ClassicBoids interactive renderer screenshot.
- `docs/assets/bird-altitude.png` — BirdFlight altitude-stability plot.
- `docs/assets/fish-drag-sweep.png` — FishSchool drag-vs-cohesion sweep.
- `docs/assets/noise-polarization.png` — NoiseExperiment order-loss sweep.

## Architecture summary

```text
include/flock3d/
  math/        Lightweight Vector3 helpers.
  sim/         Public simulation and scenario headers.
src/
  app/         Window ownership, camera setup, input, and fixed-timestep loop.
  sim/         Core boid simulation, scenario definitions, metrics, spatial hash.
  render/      raylib drawing and overlay rendering.
  experiment/  CSV export and deterministic headless experiment runner.
tests/         Catch2 simulation tests.
scripts/       Build/run/test helpers and CSV plotting utilities.
docs/          Scenario and experiment reference material.
```

The simulation stores boid state in separate vectors for positions, velocities, and accelerations. Rendering consumes simulation state but does not own update logic, and the headless experiment runner reuses the same scenario and simulation code without opening a window.

## Scenario overview

| Scenario | Status | Focus |
| --- | --- | --- |
| Classic Boids | Implemented | Baseline separation, alignment, and cohesion in a wrapped 3D world. |
| Bird Flight | Implemented | Gravity, lift, altitude hold, minimum speed, climb-rate, turn-rate, and field-of-view constraints. |
| Fish School | Implemented | Resistive-medium schooling with drag, buoyancy, target depth, smooth turning, and optional current. |
| Noise Experiment | Implemented | Deterministic perception, steering, and velocity perturbations for collective-order robustness studies. |
| Predator-Prey | Placeholder | Future pursuit/evasion roles and predator/prey metrics. |
| Obstacle Avoidance | Placeholder | Future obstacle fields and avoidance-response experiments. |
| Leadership | Placeholder | Future informed-leader and information-propagation experiments. |

See [docs/scenarios.md](docs/scenarios.md) for scenario purposes, parameters, metrics, and suggested experiments.

## Common controls

- `W/A/S/D` + mouse: free camera.
- `P`: pause or resume simulation updates.
- `R`: reset with the current seed.
- `N`: randomize seed and reset.
- `,` / `.`: switch scenario.
- `F1`: toggle the debug and metrics overlay.
- `+` / `-`: adjust boid count in batches.
- `Tab` / `Shift+Tab`: cycle the selected tunable parameter.
- `Left` / `Right` or `[` / `]`: adjust the selected parameter.
- `O`: start or stop CSV metrics recording.
- `M`: cycle export mode while recording is stopped.
- `Esc`: close the window.

## Detailed documentation

- [docs/scenarios.md](docs/scenarios.md): scenario reference, model parameters, scenario metrics, and suggested scientific questions.
- [docs/experiments.md](docs/experiments.md): CSV export modes, runner usage, presets, sweeps, and plotting scripts.

## Current status

`flock3d` is a working real-time sandbox plus deterministic experiment harness. ClassicBoids, BirdFlight, FishSchool, and NoiseExperiment have distinct simulation paths and exported metrics. PredatorPrey, ObstacleAvoidance, and Leadership currently reserve scenario names, seeds, and descriptions while reusing classic boid steering until their models are implemented.

## Roadmap

- Implement predator/prey, obstacle-avoidance, and leadership models.
- Add deterministic scenario fixtures for simulation regression tests.
- Define cluster-connectivity metrics for scenario-specific analysis.
- Add compact full-trajectory export once the schema is settled.
- Expand spatial-hash benchmarks and reusable neighbor buffers.
- Explore instanced rendering for larger flocks.
- Curate screenshots and reproducible plot artifacts for the portfolio README.
