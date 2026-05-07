# flock3d

**A real-time C++20 flocking simulator and deterministic experimentation platform for 3D collective behavior.**

`flock3d` combines an interactive raylib renderer with a headless experiment runner, sampled CSV exports, and lightweight plotting scripts. It is built as both a polished systems-programming portfolio project and a small scientific sandbox for studying how local rules produce group-level motion.

## Visual overview

Visual assets are curated from `resources/` for interactive screenshots/GIFs and from `results/<study_name>/` for generated scientific plots. This checkout does not currently contain pushed image files in either location, so the README avoids hardcoding broken image links.

<p align="center">
  <em>Add screenshots to `resources/` and study plots to `results/<study_name>/`, then link the selected files here as the hero image and comparison gallery.</em>
</p>

## Why it is interesting

- **Simulation craft**: classic Reynolds-style boids are extended into bird-flight, fish-schooling, and noise-robustness scenarios without turning the model into a pile of ad-hoc switches.
- **Systems engineering**: rendering, simulation, scenario dispatch, experiment running, and plotting are separated into small CMake targets and scripts.
- **Scientific workflow**: deterministic seeds, fixed-timestep stepping, sampled metrics, one-parameter sweeps, and repeatable plots make visual behavior measurable.

## Key features

- Interactive **raylib** renderer with camera controls, scenario switching, live parameter tuning, and a compact metrics overlay.
- Deterministic **120 Hz fixed-timestep** simulation path shared by the app and headless runner.
- Structure-of-arrays boid state and a uniform 3D spatial hash for efficient neighbor queries.
- Implemented scenarios for **ClassicBoids**, **BirdFlight**, **FishSchool**, and **NoiseExperiment**.
- Sampled CSV export, summary export, curated study scripts, and Python plotting utilities.
- Catch2 tests for core simulation, experiment, controls, and spatial-hash behavior.

## Visual gallery

The gallery should link only files that exist in the repository. Use filename cues to choose representative assets:

| Role | Source | Filename cues |
| --- | --- | --- |
| Hero screenshot/GIF | `resources/` | `bird`, `fish`, `boid`, `overlay` |
| Scenario comparison | `resources/` | `bird`, `fish`, `noise`, `overlay` |
| Quantitative plot | `results/<study_name>/` | `plot`, `polarization`, `cohesion`, `altitude` |

Keep checked-in images small enough for GitHub to render quickly. Use generated plots for quantitative results and screenshots/GIFs for first-contact visual impact.

## Example results

Curated study scripts generate compact CSV summaries and publication-style PNG plots. Once plots are pushed under `results/<study_name>/`, select the clearest examples for README display:

| Study | What to show | Good filename cues |
| --- | --- | --- |
| Noise robustness | steering noise vs polarization/order loss | `noise`, `polarization`, `order_loss`, `plot` |
| Bird flight | field of view or turn rate vs altitude/order | `bird`, `altitude`, `polarization`, `plot` |
| Fish school | drag vs cohesion/speed/depth stability | `fish`, `drag`, `cohesion`, `depth`, `plot` |

The current scripts write reproducible artifacts under `outputs/<study_name>/`; see [docs/experiments.md](docs/experiments.md) for the full workflow and commands.

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

Use `CMAKE_PRESET=release` or pass a preset to the helper scripts when you want a non-default build.

## Scenario overview

| Scenario | Focus | Representative visual |
| --- | --- | --- |
| **ClassicBoids** | Baseline separation, alignment, and cohesion in a wrapped 3D world. | Use a broad flock screenshot or overlay capture. |
| **BirdFlight** | Flight-like constraints: gravity, lift, altitude hold, turn rate, and field of view. | Use `bird` screenshots or `altitude`/`polarization` plots. |
| **FishSchool** | Resistive-medium schooling with drag, buoyancy, depth keeping, and optional current. | Use `fish`, `drag`, `cohesion`, or `depth` plots. |
| **NoiseExperiment** | Deterministic sensing/steering/velocity perturbations for robustness studies. | Use `noise`, `polarization`, or `order_loss` plots. |

Future placeholders reserve space for predator-prey, obstacle-avoidance, and leadership experiments while the current implemented scenarios remain the main research surface. Detailed model notes live in [docs/scenarios.md](docs/scenarios.md).

## Architecture summary

```text
include/flock3d/  Public math and simulation headers
src/app/          Window, camera, controls, and fixed-timestep loop
src/sim/          Boid model, scenario definitions, metrics, spatial hash
src/render/       raylib boid drawing and overlay rendering
src/experiment/   Deterministic headless runner and CSV export
tests/            Catch2 regression coverage
scripts/          Build, test, run, study, and plotting helpers
```

The interactive app and experiment runner reuse the same simulation and scenario code. That keeps the project visually engaging for exploration while preserving a deterministic path for repeatable analysis.

## Experiment workflow

```text
simulate → export CSV → analyze → generate plots
```

Two common entry points:

```bash
./scripts/run_noise_study.sh
python scripts/plot_metric.py --input outputs/run.csv --x simulation_time --y polarization --output outputs/polarization.png
```

For presets, sweeps, export modes, and plotting details, see [docs/experiments.md](docs/experiments.md).

## Documentation

- [docs/scenarios.md](docs/scenarios.md): scenario purpose, parameters, metrics, and suggested scientific questions.
- [docs/experiments.md](docs/experiments.md): runner usage, CSV export modes, presets, sweeps, study scripts, and plotting workflow.
- [docs/architecture.md](docs/architecture.md): code organization, model dispatch, spatial hash, and deterministic stepping notes.

## Current status and roadmap

`flock3d` is a working real-time simulator plus deterministic experiment harness. ClassicBoids, BirdFlight, FishSchool, and NoiseExperiment have distinct simulation paths and exported metrics; the next polish pass is primarily about curating checked-in visuals.

Planned work:

- Curate screenshots/GIFs under `resources/` and representative plots under `results/<study_name>/`.
- Implement predator-prey, obstacle-avoidance, and leadership models.
- Add deterministic scenario fixtures for regression testing.
- Define cluster-connectivity metrics for richer scenario analysis.
- Explore compact full-trajectory export and instanced rendering for larger flocks.
