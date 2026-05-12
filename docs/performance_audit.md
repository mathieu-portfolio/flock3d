# Performance audit: real-time large flocks

This report audits the current CPU-first `flock3d` architecture against smooth large-flock goals for the raylib app. It is based on the benchmark harnesses, current simulation/rendering code paths, and the observed benchmark trends summarized for recent runs. Conclusions below call out whether they are benchmark-backed or code-review hypotheses; no optimization changes are proposed as implemented work here.

## Targets and budgets

A 60 FPS frame has **16.67 ms** of wall-clock time. A practical laptop target should not allocate all of that to simulation, because raylib drawing, camera/input, overlay text, OS scheduling, and occasional fixed-timestep catch-up need headroom.

| Mode | Hard budget | Practical budget | Notes |
| --- | ---: | ---: | --- |
| 60 FPS rendering only | 16.67 ms/frame | 10-14 ms render+app | Leaves room for input and jitter. |
| 60 Hz simulation, one tick per rendered frame | 16.67 ms/tick | 6-10 ms average, <=12-14 ms p95 | Required if sim and render run on the same thread without catch-up pressure. |
| 120 Hz simulation with 60 FPS rendering | 8.33 ms/tick, two ticks/frame | 3-5 ms/tick | Two updates plus rendering must still fit a frame. |
| 240 Hz simulation stretch | 4.17 ms/tick | 1.5-2.5 ms/tick | Only realistic for smaller counts, reduced behavior, or GPU/compute approaches. |

The application currently uses a fixed-timestep accumulator that can execute multiple simulation updates per rendered frame, capped at eight updates. That prevents unbounded spiral-of-death behavior, but a large flock that exceeds the per-frame budget will either reduce visual frame rate or repeatedly hit the catch-up cap.

## Current code path summary

Each `BoidSimulation::update` performs these phases:

1. Begin metrics if enabled.
2. Recompute the effective query radius and rebuild the spatial hash.
3. Dispatch the selected model update.
4. In the common local-neighbor path, query the spatial hash per boid, filter by radius/FOV, select closest neighbors when bounded, accumulate separation/alignment/cohesion, then write acceleration.
5. Integrate velocity and position in a second parallel pass.
6. Record serial collective metrics when metrics are enabled.

The benchmark targets exercise this path without rendering and usually pass `metrics = nullptr` for the fully threaded path. The interactive app, however, passes a metrics pointer every simulation step, so app timing includes serial metric collection and metric bookkeeping that the default `simulation_update` benchmark intentionally excludes.

The renderer is immediate-mode style: every boid builds an orientation basis and draws a tetrahedron with four `DrawTriangle3D` calls, then draws the world bounds.

## Dominant costs by subsystem

### Simulation update

Benchmark-backed: per-boid local steering and neighbor work dominate the observed large-count update costs, and bounded/adaptive modes reduce workload growth relative to uncapped modes. Code-review hypothesis: bird/fish/noise model constraints are usually secondary at thousands of boids unless deterministic noise is enabled heavily, because candidate scanning, distance checks, FOV/radius filtering, and bounded-neighbor selection remain on the hot path.

For realistic real-time goals, use the bounded/adaptive or aggregate-social modes as the baseline. Uncapped fixed-radius mode should be treated as a diagnostic or small-count behavior mode, not the 10k real-time target.

### Neighbor and spatial queries

The spatial hash avoids all-pairs scaling, but it is still the central cost driver. With cell size synchronized to the maximum query radius, a normal local query visits a 3x3x3 cube of cells. Query cost therefore scales approximately with:

```text
boids * visited_cells + boids * candidates_per_query + selected_neighbor_processing
```

Benchmark-backed: late-window slowdowns correlate with spatial-query diagnostics such as occupancy and candidate counts in the benchmark output. Likely hypothesis: clustering is a major cause when dense cells are scanned repeatedly. Adaptive/bounded modes reduce selected-neighbor work, and aggregate social steering lowers social-neighbor detail, but exact local separation and candidate scans can still dominate dense regions.

### Memory layout and cache locality

Code-review hypothesis: the core boid arrays are reasonably cache-friendly structure-of-arrays vectors for positions, velocities, and accelerations, but the spatial hash is less cache-friendly. It uses `std::unordered_map<CellCoord, Cell>`, each `Cell` owns a vector of entries, and query loops perform many hash lookups while walking neighboring cell coordinates, so pointer chasing and scattered memory access are likely contributors at large counts.

Worker scratch buffers reserve capacity proportional to the full boid count for each worker. That prevents repeated allocation during the hot path, but it can become significant memory overhead at large counts and higher worker counts.

### Threading and synchronization

Benchmark-backed: threading helps medium and larger counts but shows diminishing returns at higher worker counts. Code-review factors that can flatten scaling include:

- spatial hash rebuild is serial;
- the update has at least two parallel dispatches per tick, one for steering and one for integration;
- the executor uses condition-variable wakeups and atomic chunk assignment;
- high worker counts increase synchronization and memory bandwidth pressure;
- contiguous static ranges are deterministic, but clustered flocks can still produce uneven work unless smaller chunks are used;
- metrics collection in the app is serial.

The current automatic policy caps at four workers, which matches the observed trend that 8/16 workers are not reliable wins yet.

### Rendering

Code-review risk, not yet render-benchmark-backed: rendering may become a dominant bottleneck before or near 10k boids in the interactive raylib app. The current renderer issues four 3D triangle draw calls per boid: 20,000 `DrawTriangle3D` calls for 5k boids and 40,000 for 10k, plus per-boid basis construction. Until a render-only benchmark exists, treat this as a high-probability risk rather than a measured limit.

If render-only measurements confirm this risk, large-count rendering should move toward batched or instanced meshes, lower-detail impostors/points for distance, or a GPU-side particle/mesh path.

### Benchmark instrumentation overhead

Default benchmark CSVs are suitable for trend tracking because diagnostics are compact. Diagnostics mode adds timing calls, worker summaries, and internal counters; aggregate-social diagnostics can even advance a separate diagnostic simulation with metrics enabled. Those modes are invaluable for attribution, but should not be used as the only estimate of app-frame cost. Conversely, default simulation benchmarks omit raylib rendering and often omit metrics, so they are optimistic relative to the app.

## Feasibility assessment

### 5,000 boids at 60 FPS

**Possible target, not demonstrated.** Benchmarks make 5k/60 worth pursuing with adaptive/bounded or aggregate-social behavior, reduced live metrics, and a thread count near the measured sweet spot. The full interactive target remains unproven until render-only and app-level p95 frame measurements show enough headroom.

Recommended 5k target envelope:

- simulation average <=8-10 ms/tick, p95 <=12 ms;
- render average <=4-6 ms/frame;
- no live full metrics every tick;
- bounded local neighbor count around 32-64;
- aggregate social for alignment/cohesion if exact local flocking is too expensive.

### 10,000 boids at 60 FPS

**Stretch target, not a current claim.** Benchmark observations already place heavier 10k scenarios at substantially higher update costs. Code review also flags linear draw-call growth as a rendering risk, but that part still needs a render-only benchmark. A credible 10k/60 target likely requires both CPU data-oriented simulation improvements and renderer batching/instancing; simply raising thread count is unlikely to solve serial rebuild, cache locality, candidate scans, and app/render overhead.

A credible 10k CPU-first target would require:

- a cache-friendly spatial grid or sorted cell ranges instead of `unordered_map` cells;
- reduced candidate scanning under clustering;
- one-pass or fused update/integration where correctness allows;
- stable worker chunking tuned by measured load balance;
- app metrics sampled at a lower rate;
- batched/instanced raylib rendering.

### Much larger scales

**Much larger than 10k is outside the comfortable range of the current CPU/raylib-immediate design.** With local interactions, larger scales need either stronger approximations or a GPU-oriented pipeline. CPU-only can go further with data-oriented grids, SIMD, LOD, and approximate aggregate fields, but tens or hundreds of thousands of visually smooth agents are better treated as a GPU compute/rendering project with a separate validation strategy.

## Highest-impact roadmap

### Small/local optimizations

1. Disable or downsample live metrics for large interactive runs; keep full metrics for experiments.
2. Use diagnostics to choose a default worker count and chunk size per count/mode, not a single high thread count.
3. Avoid diagnostics mode for headline performance numbers.
4. Reuse query buffers carefully and verify scratch capacity does not grow unnecessarily for high counts.
5. Keep bounded neighbor limits low enough that `nth_element`/sort and steering loops stay predictable.

These are useful, but they will not by themselves make 10k/60 robust.

### Data-oriented CPU improvements

1. Replace `unordered_map<CellCoord, Cell>` with a dense or sorted spatial grid for the wrapped world: compute cell indices, sort boid indices by cell, and store contiguous cell ranges.
2. Build cell aggregates in contiguous arrays and query by integer cell ranges without hash lookups.
3. Separate exact local separation from approximate social steering: exact within small separation cells, aggregate or bounded K for alignment/cohesion.
4. Track and cap worst-case candidate scans in clustered cells, not just selected-neighbor counts.
5. Consider SIMD-friendly distance/FOV filtering over contiguous candidate spans.
6. Evaluate fusing phases or double-buffering state to reduce repeated passes and synchronization.
7. Make metrics a sampled subsystem rather than per-tick app work for large counts.

These are the likely highest-impact changes for laptop CPU simulation.

### Rendering optimizations

1. Replace per-boid `DrawTriangle3D` calls with a reusable mesh and batched/instanced rendering where raylib support is sufficient.
2. Use lower-detail geometry, billboards, or points for distant boids.
3. Add frustum/distance culling for camera views that do not need every boid drawn at full detail.
4. Avoid rebuilding per-boid orientation geometry on the CPU when a shader/instance transform can do it.
5. Benchmark render-only frame time separately from simulation-only update time.

Rendering should be measured before claiming 5k or 10k interactive success, because headless simulation benchmarks do not cover that risk.

### GPU/compute-oriented approaches

GPU compute is not a small refactor of the current deterministic CPU path. It would require GPU-resident SoA buffers, GPU spatial binning or sort, compute kernels for neighbor/aggregate steering, synchronization with rendering buffers, and a tolerance-based validation strategy. It becomes the right approach when the target moves beyond 10k/60 with rich behavior, or when higher simulation rates are required at 10k+.

## Recommended validation plan

1. Run release `simulation_update` at 5k and 10k with `--diagnostics phases` for the candidate target modes: adaptive bounded and aggregate social.
2. Run `spatial_hash` at the same counts and world/radius settings to record candidate counts, visited cells, max occupancy, and late-window slowdown.
3. Run `metrics` at 5k and 10k to quantify live-app metric overhead.
4. Add or run a render-only benchmark that draws frozen boid states at 1k/5k/10k using the current renderer.
5. Measure the full app with overlay off and on, recording update ms, render ms, catch-up cap hits, and p95/p99 frame times.
6. Treat success as p95 frame time under 16.67 ms, not just average update time under 16.67 ms.

## Bottom line

The current architecture is on the right path for **several thousand CPU-simulated boids**, especially with adaptive/bounded neighbor selection and moderate threading. **5k/60 is a reasonable near-term target**, but should not be claimed until render-only and full-app p95 measurements pass. **10k/60 remains a stretch goal** until spatial queries become more data-oriented and rendering is batched or otherwise proven affordable. **Much larger scales** should be planned as approximate CPU LOD or GPU compute/rendering work rather than incremental tuning of the current `unordered_map` spatial hash and immediate-mode renderer.
