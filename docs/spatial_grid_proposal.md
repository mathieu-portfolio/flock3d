# Spatial Grid Proposal

## Purpose

`flock3d` currently uses `SpatialHash3D`, a uniform spatial hash backed by `std::unordered_map<CellCoord, Cell>`, to reduce neighbor searches from all-pairs scans to nearby-cell scans. Render-only benchmarks indicate that rendering is not the immediate bottleneck; simulation cost is more likely dominated by neighbor candidate scanning, memory locality, aggregate-cell traversal, and hash-table lookups. This document proposes a cache-friendly spatial grid that can be implemented beside the existing hash, validated against it, and benchmarked before any behavior switch.

Non-goals for this proposal:

- Do not implement the new grid yet.
- Do not change simulation behavior, tuning, rendering, or GPU paths.
- Do not remove `SpatialHash3D`.
- Do not assume the new grid is faster until equivalence tests and benchmarks prove it for the target workloads.

## Current spatial hash usage

### Data structure and API

`SpatialHash3D` stores occupied cells in an `std::unordered_map<CellCoord, Cell, CellCoordHash>`. Each `Cell` owns a `std::vector<Entry>` containing boid indices and positions plus a cached `CellAggregate` with count, position sum, velocity sum, centroid, and average velocity.

Current public behavior to preserve or explicitly replace:

- `cell_for(position)`: integer cell coordinates from `floor(position / cell_size)`.
- `insert(boid_index, position, velocity)`: appends the boid entry to that cell and incrementally updates the aggregate.
- `clear()`: removes all cells and entries.
- `query_neighbors(position, radius, result, diagnostics)`: visits every integer cell in a cube around the query cell, tests exact distance to each entry, and returns accepted boid indices.
- `query_cell_aggregates(position, radius, result, diagnostics)`: visits the same cube and returns occupied cell aggregates whose centroids are within the query radius.
- `query_visible_cell_aggregates(...)`: optionally culls cells by a conservative field-of-view check before aggregate lookup.
- `aggregate_for(coord)`: returns an aggregate pointer for tests and direct diagnostics.
- Occupancy diagnostics: `cell_count()`, `total_entries()`, `max_cell_occupancy()`, and `average_cell_occupancy()`.

### Simulation dependency map

Every new grid implementation should account for these call sites and behavioral assumptions before it is used in simulation:

| Area | Current dependency | Preservation requirement |
| --- | --- | --- |
| Rebuild | `BoidSimulation::update()` recreates the hash when effective query radius changes, rebuilds it once per tick, and records rebuild timing. | New grid must support full rebuild from the current `positions_` and `velocities_` once per simulation step, with timing comparable to existing diagnostics. |
| Fixed-radius neighbor queries | `update_shared_flocking()` calls `query_neighbors()` for each boid using `effective_query_radius()`. | Return the same set of boid indices for a query radius, including the querying boid when it lies inside its own query radius so current self-filtering remains valid. |
| Closest-K / bounded neighbor modes | `update_shared_flocking()` converts neighbor indices to `NeighborCandidate`, then `select_closest_neighbors()` sorts by distance and boid index after optional top-K pruning. | Candidate set equivalence matters more than raw candidate order for closest-K modes, but deterministic ordering must be available for modes that consume raw query results. |
| Adaptive radius modes | `compute_effective_perception_radius()` uses the local candidate count from the spatial query to shrink or expand perception radius. | Candidate counts must remain equivalent for the broad query radius, excluding only behavior-equivalent false positives/false negatives. |
| Field-of-view filtering | Bird/Fish modes filter individual neighbors after the query. Aggregate-social mode can pre-cull aggregate cells with `query_visible_cell_aggregates()`. | Individual neighbor queries should provide all metric candidates; aggregate queries need an equivalent conservative cell-FOV culling policy. |
| Aggregate-social queries | `update_cell_aggregate_social()` queries exact neighbors for separation, queries cell aggregates for alignment/cohesion, then subtracts the current boid from its own aggregate. | New grid must expose aggregate ranges and `CellAggregate` values with stable `CellCoord`, counts, sums, centroids, and averages. |
| Exact separation | Aggregate-social still performs exact per-boid neighbor scanning for separation radius before using aggregate social cells. | New grid must support exact small-radius per-boid scans without coupling them to aggregate-cell acceptance. |
| Diagnostics and metrics | Metrics record neighbor candidate counts, visited cells, aggregate candidate cells, rejected aggregate cells, spatial cell count, average occupancy, and max occupancy. | Diagnostics should map one-to-one where possible; if dense-grid lookups make “visited empty cells” cheaper but still counted, keep counts semantically comparable. |
| Benchmarks | `spatial_hash_benchmark` directly times rebuild/query and compares neighbor counts to naive all-pairs; aggregate and simulation benchmarks expose internal diagnostics. | Add side-by-side benchmark variants rather than replacing existing hash benchmark output immediately. |
| Tests | Spatial hash tests cover stable cell mapping, exact inclusion/exclusion, occupancy diagnostics, total entries, and aggregates. Simulation/experiment tests assume deterministic sampled metrics. | Add equivalence tests against `SpatialHash3D` first; update simulation tests only after intentionally selecting a backend. |
| Determinism | The architecture relies on a single rebuild before read-only parallel per-boid evaluation, deterministic boid index ranges, and deterministic selection tie-breaking by boid index. | Grid rebuild and query traversal must not introduce thread-race-dependent ordering or unordered container iteration into behavior-critical paths. |

## Proposed cache-friendly design

Add a new implementation, tentatively `SpatialGrid3D`, beside `SpatialHash3D`. Keep it private to the simulation module until tests and benchmarks stabilize the API. It should use the same `CellCoord`, `CellAggregate`, and `NeighborQueryDiagnostics` concepts where practical to make equivalence testing simple.

### Core representation

Recommended sparse sorted-grid layout:

```text
positions_/velocities_ remain owned by BoidSimulation

SpatialGrid3D
  cell_size: float
  entries: vector<GridEntry>              // one per boid, sorted by cell key then boid index
  cell_ranges: vector<CellRange>          // one per occupied cell, sorted by cell key
  aggregates: vector<CellAggregate>       // same order as cell_ranges
  scratch/counting buffers as needed      // reused across rebuilds
```

Suggested records:

```text
GridEntry
  CellCoord coord
  uint64_t key              // deterministic packed/sort key derived from coord
  size_t boid_index
  Vector3 position

CellRange
  CellCoord coord
  uint64_t key
  size_t begin              // inclusive offset into entries
  size_t end                // exclusive offset into entries
  size_t aggregate_index    // often identical to range index
```

The first implementation should favor a sparse sorted grid over a dense 3D array because current positions are world-space floats, worlds can be sparse, and wrapping is handled by simulation integration rather than by spatial indexing. Dense variants can be added later for known bounded worlds.

### Rebuild algorithm

A straightforward rebuild path is sufficient for the first version:

1. Resize `entries` to boid count.
2. For each boid index in ascending order:
   - Compute `coord = cell_for(position)` using the same floor semantics as `SpatialHash3D`.
   - Compute a deterministic `key` from integer coordinates.
   - Store `{coord, key, boid_index, position}`.
3. Stable-sort entries by `(key, coord.x, coord.y, coord.z, boid_index)`.
   - If the key is collision-free for supported coordinate ranges, the explicit coordinate tie-breakers are still useful documentation and safety.
   - Stable sorting is not strictly necessary if boid index is part of the comparator, but deterministic tie-breaking is mandatory.
4. Build contiguous `cell_ranges` by scanning sorted entries.
5. Build `aggregates` in the same scan using velocities from the simulation arrays.
6. Cache occupancy totals and maxima while building ranges.

This is more work per rebuild than appending to unordered-map buckets, but it improves query locality: each occupied cell is represented by one range into a contiguous entry array and one aggregate in a contiguous aggregate array.

### Cell key options

The grid needs deterministic lookup for integer coordinates. Acceptable approaches:

- **Packed bounded key**: If the world and cell size imply bounded coordinates, bias each coordinate and pack into a 64-bit integer. This is fastest but needs validation for world sizes and query radii.
- **Morton/Z-order key with signed-coordinate bias**: Improves spatial locality among nearby cells while preserving deterministic sort order. It requires clear coordinate-range limits.
- **Lexicographic key/comparator only**: Avoids packing constraints by sorting and binary-searching `(z, y, x)` or `(x, y, z)`. It may be slightly slower but safest for the first implementation.

Recommendation: start with a lexicographic comparator or a packed key plus coordinate fallback. Optimize key packing only after benchmarks show lookup overhead matters.

## Query behavior

### Fixed-radius neighbor queries

The new grid should preserve the current query algorithm’s semantics while changing storage:

1. Compute query center cell with `cell_for(position)`.
2. Compute `cell_radius = ceil(radius / cell_size)`.
3. Traverse integer cells in a deterministic nested-loop order matching the hash today: `z`, then `y`, then `x`.
4. For each cell coordinate:
   - Increment `visited_cells` even if no occupied range exists, preserving diagnostic meaning.
   - Locate the occupied range via binary search in `cell_ranges` or a compact side index.
   - Increment `candidates_tested` by range length.
   - Scan `entries[begin:end]` contiguously and perform exact squared-distance filtering.
   - Append accepted boid indices to the caller-provided vector.

For fixed-radius uncapped behavior, returning candidates in cell traversal order plus boid-index order inside each cell is deterministic and should usually match current insertion-order-in-cell behavior because the hash is rebuilt by ascending boid index. It may differ from current cross-cell behavior only if the unordered map leaks into ordering; the current query loops coordinates explicitly, so matching nested-loop traversal should preserve ordering.

### Closest-K and bounded neighbor modes

Current closest-K behavior is primarily determined by `select_closest_neighbors()`, which sorts by distance and breaks ties by boid index after optional pruning. Therefore the grid must ensure:

- It returns every candidate inside the broad query radius.
- It provides deterministic offsets and distances from the same positions used by the hash.
- It does not rely on unstable cell iteration for candidates with equal distance.

If future modes consume a bounded candidate stream directly, add an explicit deterministic final ordering step or a query option that emits `(distance_squared, boid_index)` sorted candidates.

### Adaptive radius modes

Adaptive perception currently uses the broad query candidate count to compute an effective perception radius. The grid can support this by:

- Running the same broad query using `effective_query_radius(parameters)`.
- Counting accepted broad-radius neighbors exactly as today.
- Applying the current adaptive radius computation outside the grid.

Optimization opportunity after equivalence: a count-only query could avoid storing every broad candidate when only the count is needed before a second pass. Do not introduce that until tests prove it does not change behavior.

### Aggregate-social queries

Aggregate queries should use `cell_ranges` and `aggregates` directly:

- Traverse query cells in the same deterministic order as neighbor queries.
- Count visited cells like the existing diagnostics.
- Locate each occupied cell range.
- Increment aggregate `candidates_tested` once per occupied cell considered.
- Filter by aggregate centroid distance to preserve current aggregate acceptance.
- Return `CellAggregate` values in traversal order.

For `query_visible_cell_aggregates()`, keep the current conservative cell field-of-view test based on cell center and bounding radius. A cell skipped by FOV culling currently does not increment `visited_cells`; the new grid should preserve that diagnostic convention unless metrics are intentionally revised.

### Exact separation queries

Aggregate-social mode still needs exact separation from individual boids. The same fixed-radius neighbor query can serve separation, but the implementation should consider a specialized exact-radius scan later:

- Query with `separation_radius` when exact separation is the only consumer.
- Reuse the same range scanning logic.
- Return boid indices or directly accumulate separation in a callback to avoid temporary vectors.

For the initial coexistence implementation, prefer API equivalence over callback micro-optimization.

### Occupancy diagnostics

The new grid can expose diagnostics from cached rebuild statistics:

- `cell_count = cell_ranges.size()`.
- `total_entries = entries.size()`.
- `max_cell_occupancy = max(range.end - range.begin)`.
- `average_cell_occupancy = entries.size() / cell_ranges.size()` or `0.0` for no cells.

Keep metric names stable at first, even if internal code uses “grid” rather than “hash”, to avoid breaking CSV consumers prematurely.

## Determinism plan

Determinism should be treated as an acceptance criterion, not a nice-to-have.

Required rules:

1. **Stable rebuild input order**: populate entries by ascending boid index.
2. **Stable cell ordering**: sort occupied cells by a documented total order over integer coordinates.
3. **Stable in-cell ordering**: sort entries within a cell by boid index.
4. **Stable query traversal**: use the same `z/y/x` nested loops as `SpatialHash3D` unless a behavior migration deliberately changes it.
5. **Stable tie-breaking**: preserve `select_closest_neighbors()` distance-then-boid-index ordering for bounded modes.
6. **No unordered iteration in behavior paths**: unordered containers may be used for optional lookup acceleration only if their iteration order never affects returned candidates, aggregates, metrics, or floating-point accumulation order.
7. **Parallel safety**: rebuild once before worker queries; query data is immutable during per-boid updates; worker scratch vectors remain per-worker as today.

Floating-point aggregate sums are order-sensitive. The grid should accumulate aggregates in ascending boid index within each cell to match current hash rebuild insertion order. If sorted-cell aggregation changes summation order for a cell, equivalence tests should catch drift in aggregate-social metrics.

## Wrapped-world handling

`BoidSimulation::wrap_position()` wraps positions into `[-world_half_extent, world_half_extent]` after integration, but spatial queries currently use ordinary Euclidean offsets and do not query periodic image cells across world boundaries. The new grid should initially preserve this behavior exactly:

- Use wrapped positions as stored by simulation.
- Do not add toroidal neighbor offsets.
- Do not mirror cells at boundaries.

A future wrapped-neighbor feature would need explicit behavior tests because it would change flocking, not just data layout.

## Dense vs. sparse variants

### Sparse sorted grid (recommended first)

Benefits:

- Memory scales with boid count and occupied cells.
- Works well for sparse worlds and arbitrary integer coordinates.
- Coexists naturally with current `CellCoord` API.
- Contiguous entries and aggregates improve query locality over hash buckets.

Costs:

- Rebuild requires sorting, typically `O(n log n)` unless replaced by counting/radix sorting.
- Cell lookup by binary search is `O(log occupied_cells)` unless a secondary index is added.
- Very dense small worlds may pay unnecessary lookup overhead compared with direct dense indexing.

### Dense bounded grid (possible later)

Benefits:

- Direct cell lookup by array index.
- Predictable memory access and no binary search.
- Rebuild can be `O(n + cell_count)` with counting offsets.

Costs:

- Memory scales with total world cells, not occupied cells.
- Requires robust coordinate bounds and resize behavior.
- Poor fit for large or sparse worlds.
- Wrapped boundaries need careful indexing semantics if behavior changes later.

Recommendation: implement sparse sorted first; add dense bounded only if benchmarks identify lookup cost as the remaining bottleneck for known bounded worlds.

## Expected trade-offs

| Dimension | Expected effect |
| --- | --- |
| Rebuild cost | Likely higher initially due to sorting. Can be reduced later with radix/counting sort, persistent buffers, or dense-grid prefix sums. |
| Query speed | Expected to improve for many workloads because entries, ranges, and aggregates are contiguous and branch/pointer chasing is lower than unordered-map buckets. |
| Memory use | More predictable contiguous allocations; may store duplicated coord/key per entry. Avoids many small bucket allocations and per-cell vectors. |
| Candidate scanning | Same candidate set, but scans are linear contiguous ranges. Candidate order can be made explicit and stable. |
| Aggregate queries | Should improve locality because aggregates are contiguous and range metadata is compact. |
| Diagnostics | Can preserve current counts; may add grid-specific counters later, such as binary searches, occupied ranges touched, and empty cells visited. |
| Wrapped worlds | Preserve current non-toroidal query semantics initially. True periodic queries are a separate behavior feature. |
| Dense worlds | Dense direct-index grid may eventually outperform sparse sorted lookup; do not start there unless memory bounds are guaranteed. |
| Sparse worlds | Sparse sorted grid avoids allocating empty world cells while still improving occupied-cell locality. |
| Large boid counts | Sorting cost grows, but query locality should matter more as candidate scans dominate. Benchmarks must cover large counts before switching defaults. |

## Equivalence and benchmark strategy

### Tests to add before simulation integration

1. **Cell mapping parity**: compare `cell_for()` for representative positive, negative, boundary, and large coordinates.
2. **Neighbor set parity**: build both structures from the same positions and compare sorted neighbor-index sets for many query positions/radii.
3. **Neighbor order parity where required**: for default fixed-radius modes, compare returned order against the hash for deterministic fixtures. If order differs but selected output is equivalent, document the accepted difference before using it in behavior paths.
4. **Diagnostics parity**: compare visited cells, candidates tested, cell count, total entries, max occupancy, and average occupancy.
5. **Aggregate parity**: compare `CellAggregate` count, sums, centroids, averages, and query results.
6. **Visible aggregate parity**: compare FOV-culled aggregate results and diagnostics for representative forward vectors and FOV angles.
7. **Simulation metric parity**: run short deterministic scenarios for all models and neighbor modes with both backends and compare positions/velocities/metrics within a documented tolerance.
8. **Parallel determinism parity**: repeat tests with serial and explicit worker counts, verifying backend choice does not introduce nondeterministic output.

### Benchmarks to add

- Add a grid variant to the existing spatial benchmark rather than replacing the hash benchmark.
- Measure rebuild time, query time, total tick time, candidates/query, visited cells/query, occupied cells, max occupancy, and average occupancy.
- Include sparse, clustered, and dense scenarios.
- Include fixed-radius, closest-K, adaptive radius, aggregate-social, and exact-separation-heavy workloads.
- Preserve existing CSV columns and append backend-specific columns where needed.

## Recommended implementation sequence

1. **Introduce type beside old hash**
   - Add `SpatialGrid3D` files with no simulation call-site changes.
   - Reuse `CellCoord`, `CellAggregate`, and `NeighborQueryDiagnostics` if possible.

2. **Build equivalence tests**
   - Create fixtures that build `SpatialHash3D` and `SpatialGrid3D` from identical boid arrays.
   - Compare cell mapping, exact neighbor sets, aggregate data, diagnostics, and FOV aggregate queries.

3. **Add benchmark comparison**
   - Extend `spatial_hash_benchmark` or add `spatial_grid_benchmark` with a `backend` column.
   - Keep old hash numbers available in the same run.

4. **Add an internal backend switch**
   - Add an experimental parameter or compile-time option only after parity tests pass.
   - Default remains `SpatialHash3D`.
   - Ensure metrics and CSV output remain stable.

5. **Run deterministic simulation comparisons**
   - Cover ClassicBoids, BirdFlight, FishSchool, NoiseExperiment, fixed radius, closest-K, adaptive radius, and aggregate-social modes.
   - Compare both single-thread and multi-thread deterministic paths.

6. **Enable optimized modes selectively**
   - Switch only the modes whose behavior is equivalent enough and whose benchmarks improve.
   - Keep fallback to `SpatialHash3D` until the grid has broad coverage and production confidence.

7. **Only then consider cleanup**
   - Rename metrics from “hash” to “spatial grid” in a compatibility-aware way.
   - Deprecate old paths only after benchmark history and tests show the new grid fully replaces them.

## Open questions

- Should the first grid use binary search per queried cell, or maintain a small deterministic lookup table from sorted cell key to range index?
- What coordinate range should a packed key support if world size becomes user-configurable beyond current presets?
- Are exact aggregate-social floating-point sums required to be bit-for-bit identical, or is a tolerance acceptable once backend switching is explicit?
- Should a future query API offer callback-based accumulation to remove temporary neighbor vectors, or should that wait until after backend parity?
- At what boid counts does sorting rebuild cost overtake unordered-map insertion cost on target hardware?
