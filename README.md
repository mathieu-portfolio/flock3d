# flock3d

`flock3d` is a modern C++20 real-time 3D boids simulation focused on clean architecture, deterministic simulation, spatial partitioning, and performance-oriented design. The project starts deliberately small: it opens a raylib-powered 3D window, advances a fixed-timestep simulation at 120 Hz, and renders simple moving boids inside a wrapped world volume.

## Features

- **Modern C++20** with small, focused translation units.
- **Modular CMake + FetchContent** for cross-platform dependency management.
- **raylib** rendering with a free 3D camera and FPS counter.
- **Catch2** tests for spatial hashing, wrapping, and fixed-timestep behavior.
- **Deterministic fixed timestep** simulation loop at 120 Hz.
- **Simulation/rendering separation** so flock behavior can evolve without coupling to drawing code.
- **Data-oriented SoA storage** for boid positions, velocities, and accelerations.
- **Uniform 3D spatial hash** skeleton for future neighbor queries.
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
  app/      Window ownership, camera setup, and the fixed-timestep main loop.
  sim/      Core simulation implementation and CMake target definition.
  render/   raylib rendering implementation and CMake target definition.
tests/      Catch2 coverage and test target definition.
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

## What are boids?

Boids are a classic model of emergent flocking behavior. Each agent follows a few local steering rules:

1. **Separation**: avoid crowding nearby agents.
2. **Alignment**: steer toward the average heading of neighbors.
3. **Cohesion**: steer toward the center of nearby agents.

`flock3d` currently includes the simulation and spatial-partitioning skeleton needed for those rules, but the full flocking forces are intentionally left as focused stubs for the next iteration.

## Controls

- `W/A/S/D` + mouse: free camera controls through raylib.
- `Esc`: close the window.

## Roadmap

- Implement separation, alignment, and cohesion forces.
- Add deterministic scenario fixtures for simulation regression tests.
- Expand the spatial hash to support reusable neighbor buffers.
- Add benchmarks for hash construction and simulation update passes.
- Explore instanced rendering for larger flocks.
