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

## Spatial hash

Neighbor queries use a uniform 3D spatial hash rather than all-pairs checks. Boids are inserted into cells derived from position and queried by nearby cells, which keeps local-interaction scans compact for the flock sizes targeted by the renderer and study scripts.

## Deterministic stepping

The app advances the simulation with a fixed timestep, and the experiment runner exposes the same `fixed_dt` as a command-line option. Determinism depends on keeping the scenario, seed, boid count, timestep, duration, export cadence, preset, and swept parameter values identical between runs.

## Data flow

```text
Scenario defaults + preset + overrides
        ↓
Simulation state initialization
        ↓
Fixed-timestep update loop
        ↓
Metrics sampling
        ↓
CSV export or live overlay
        ↓
Python plotting scripts
```
