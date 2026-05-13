# SpatialGrid3D query traversal profile

## Summary

The first contiguous `SpatialGrid3D` implementation only matched or slightly improved the old `SpatialHash3D` backend because its hot queries still performed one `std::lower_bound` for every cell in the visited query cube. With the current cell-size policy, a normal neighbor query visits a 3x3x3 cube, so the grid replaced 27 hash lookups with 27 binary searches. That preserves behavior, but it leaves a large fixed traversal cost in front of the contiguous candidate scan.

The highest-impact short-term optimization is to make sorted-grid traversal row-oriented instead of cell-oriented: perform one binary search per `(z, y)` row, then scan the contiguous occupied `x` ranges in that row. For the default 3x3x3 query cube this cuts grid lookups from 27 to 9 per query before candidate scanning begins, while preserving the existing z/y/x traversal order for occupied cells.

## Why the first contiguous grid was not much faster

1. **Cell lookup cost moved, but did not disappear.** `SpatialHash3D` pays for `unordered_map::find` per visited cell. The initial sorted grid paid for `std::lower_bound` per visited cell. For small query cubes and moderate occupied-cell counts, that binary-search overhead can be close to the hash lookup it replaced.
2. **Candidate sets are intentionally identical.** The grid and hash scan the same neighboring cells and test the same boid positions. Contiguous entries improve locality inside occupied cells, but they do not reduce the number of distance checks.
3. **Empty-cell traversal remained dominant in sparse scenes.** Low-density scenarios still visit the full query cube for every boid, but many visited cells are empty. A per-cell binary search makes those empty cells expensive.
4. **Aggregate queries still had lookup overhead.** Aggregate queries scan one aggregate per occupied cell rather than every boid entry, so lookup/traversal overhead is an even larger share of total time than it is for neighbor scans.
5. **Rebuild sorting is a separate trade-off.** A contiguous sorted grid gains query locality by sorting entries during rebuild. If query traversal is not significantly cheaper, rebuild overhead can hide the benefit.

## Implemented optimization

`SpatialGrid3D` now uses row-oriented traversal for exact neighbor and aggregate queries. Each `(z, y)` row does a single lower-bound lookup at `min_x`, then walks contiguous occupied ranges until `max_x`. Diagnostics now distinguish total semantic visited cells from physical lookup operations and occupied cells touched.

Expected impact:

- default 3x3x3 exact queries: 27 semantic cells, 9 grid binary searches;
- radius/cell-size ratio of 2: 125 semantic cells, 25 grid binary searches;
- aggregate queries benefit strongly because each occupied range directly maps to one contiguous aggregate record;
- dense rows benefit from streaming adjacent `CellRange`, `CellAggregate`, and entry ranges.

## Benchmark instrumentation

The `spatial_hash` benchmark now records additional per-query diagnostics for both backends:

- `cell_lookups_per_query`: physical lookup operations performed by the backend;
- `occupied_lookup_cells_per_query`: occupied cells/ranges touched by the query;
- aggregate query timing and analogous aggregate traversal counters;
- aggregate result count per query.

Use these fields to separate fixed traversal overhead from candidate scan work. If `cell_lookups_per_query` drops but query time does not, the next bottleneck is candidate scanning, distance math, result writes, or rebuild cost rather than binary search.


## Per-candidate memory/layout audit

The new benchmark evidence changes the likely bottleneck. Row-oriented lookup counters show that the fixed lookup term is mostly solved, and callback/direct traversal has reduced the old vector-materialization term. Remaining dense and high-radius regressions should therefore be treated first as per-candidate traversal bandwidth and loop-body cost rather than more empty-cell lookup overhead.

### Entry size and fields touched

`SpatialGrid3D::Entry` currently stores the sorted cell coordinate, boid index, position, and velocity in one array element. On a normal 64-bit layout this is expected to be 48 bytes: a 12-byte `CellCoord`, 4 bytes of padding before the 8-byte `std::size_t`, then 12-byte position and 12-byte velocity. `SpatialHash3D::Entry` stores only boid index and position, so the equivalent candidate record is expected to be 24 bytes.

Exact neighbor scans only need `position` for the distance test and `boid_index` for accepted callbacks. They do not need the entry's `coord`, because `CellRange` already identifies the cell span being scanned, and they do not need `velocity`, because velocity is only used while building per-cell aggregates. The current grid entry therefore streams roughly twice the bytes per candidate that the hash entry needs for the same exact scan.

Relevant hot-path field use:

- grid scan: range traversal reads `CellRange::coord.x`, `begin`, and `end`, then each candidate reads `Entry::position` and conditionally `Entry::boid_index`;
- hash scan: each occupied-cell vector scan reads `Entry::position` and conditionally `Entry::boid_index`;
- grid rebuild/aggregate construction reads `Entry::coord`, `position`, and `velocity`, but that is not the exact-query inner loop.

### Cache-line pressure

A 64-byte cache line fits two 24-byte hash entries with room for part of a third, but only one 48-byte grid entry with part of the next. In dense cells or high-radius queries, where many candidates are rejected after only a distance check, the grid can consume nearly one cache line per one to two candidates even though the loop body needs only 20 useful bytes (`position` plus eventual `boid_index`). Velocity alone adds 12 cold bytes per candidate to exact scans; the stored coordinate plus padding adds another 16 bytes.

The contiguous grid still has better spatial locality than many separately allocated per-cell vectors, and the row directory reduces lookup pressure. However, once lookup cost is amortized, the larger stride becomes visible: dense scans stream more memory, bring fewer useful candidates per line, and increase the chance that the exact distance loop is bandwidth or L1/L2 fill limited.

### Unnecessary loads during exact neighbor scan

The source only explicitly loads `entry.position` before the radius test and `entry.boid_index` after a hit, but cache fills operate at line granularity. Because coord and velocity are colocated with position, they ride along for every candidate touched by exact scanning. That makes them bandwidth-inflating fields even if the compiler does not emit scalar loads for them in the loop.

This matters most for dense/high-radius regressions because the accepted-neighbor count is not the only cost: rejected candidates still pull the full strided entry through cache to compute the squared distance. If the benchmark shows candidates per query rising while lookup counts stay low and materialization is reduced, this layout is a plausible primary cause.

### Query-only compact entry or split layout

A narrow query entry is the next contained design to test before broad SoA work:

```cpp
struct QueryEntry {
    std::size_t boid_index;
    Vector3 position;
};
```

`SpatialGrid3D` could keep sorted `QueryEntry` data for exact neighbor traversal and store aggregate-only inputs separately during rebuild, or it could split the current entry into parallel sorted arrays/spans for query data and aggregate data. Either version would reduce exact scan stride from about 48 bytes to about 24 bytes and remove coord/velocity from the dense traversal stream while preserving the existing row/range algorithm.

Recommended narrow next step: add a query-only compact-entry experiment behind the existing `SpatialGrid3D` benchmark path, keeping behavior and traversal order equivalent. Measure dense and high-radius cases with candidates/query, cell lookups/query, occupied cells/query, exact query time, aggregate query time, rebuild time, and p95/p99 update time. Do not combine this experiment with SIMD, dense-grid indexing, GPU work, or a broad SoA migration, because the goal is to isolate whether candidate record width is the remaining regression source.

## Highest-impact next optimizations before production switch

1. **Persist row starts or add a compact row directory.** Row-oriented traversal still binary-searches every `(z, y)` row. A secondary sorted row index from `(z, y)` to the first `CellRange` would replace row lower-bounds with one row lookup and a direct range scan.
2. **Specialize common query radii.** The default cell size equals the broad query radius, so most exact queries are fixed 3x3x3 traversals. Precomputed offsets or unrolled row traversal can reduce loop and coordinate-construction overhead.
3. **Avoid temporary neighbor vectors in hot simulation paths.** The current API materializes boid indices before model-specific filtering and accumulation. Callback/direct-accumulation query APIs would keep candidate scans contiguous and avoid result-vector writes for modes that do not need raw neighbor order.
4. **Split exact separation from broad social queries.** Aggregate-social mode still performs exact separation scans. A separation-specific traversal using the smaller separation radius can reduce candidate scans and row lookups.
5. **Improve rebuild cost after query wins are proven.** If row traversal makes queries clearly faster but total ticks remain rebuild-bound, replace comparison sort with radix/counting sort or a dense bounded grid for known bounded worlds.
6. **Consider structure-of-arrays entries for candidate locality.** Current contiguous entries are better than per-cell vectors, but distance checks still load coord, boid index, position, and velocity together. A query-only position/index layout can reduce cache bandwidth for exact neighbor scans.

## Production-switch gate

Do not switch simulation paths solely because the grid is contiguous. Switch only after benchmark runs show that:

- query timing improves after accounting for rebuild cost;
- aggregate query timing improves in aggregate-social workloads;
- cell lookup counters fall as expected;
- candidate counts and neighbor counts remain behavior-equivalent;
- dense and clustered scenarios do not regress p95/p99 update time.
