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
