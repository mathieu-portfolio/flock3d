# flock3d

`flock3d` is a modern C++20 real-time 3D boids simulation focused on clean architecture, deterministic simulation, spatial partitioning, and performance-oriented design. The project starts deliberately small: it opens a raylib-powered 3D window, advances a fixed-timestep simulation at 120 Hz, and renders simple moving boids inside a wrapped world volume.

## Features

- **Modern C++20** with small, focused translation units.
- **CMake + FetchContent** for cross-platform dependency management.
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
- Ninja is recommended for the provided presets

### Configure, build, and test

```bash
cmake --preset default
cmake --build --preset default
ctest --preset default
```

### Run

```bash
./build/default/flock3d
```

On Windows, the executable is typically located at:

```powershell
.\build\default\flock3d.exe
```

## Architecture overview

```text
src/
  app/      Window ownership, camera setup, and the fixed-timestep main loop.
  sim/      Deterministic boid data, world wrapping, and spatial hashing.
  render/   raylib rendering for boids and world bounds.
  math/     Lightweight Vector3 helpers used by simulation code.
tests/      Catch2 coverage for core simulation behavior.
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
