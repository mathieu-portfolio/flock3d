# SpatialGrid3D experiment record

## Status: frozen experiment

`SpatialGrid3D` was explored as an experimental sparse sorted-grid backend beside the production `SpatialHash3D` implementation. The work was useful and evidence-driven: it validated behavioral parity goals, added benchmark diagnostics, and clarified where spatial-query time is actually spent in the current flocking workload.

**Stop/go result: stop for now.** `SpatialGrid3D` should not be switched into production simulation yet, and additional neighbor-grid optimization patches should not be prioritized for the current portfolio milestone.

## What was tested

The experiment focused on making a sparse sorted grid competitive with the existing hash backend without changing simulation behavior:

- row-directory / row-span lookup to reduce per-cell binary searches;
- callback/direct traversal to reduce temporary neighbor-vector materialization;
- compact query-entry layout analysis for candidate-scan bandwidth;
- benchmark diagnostics comparing lookup counts, occupied cells touched, candidates, exact query time, aggregate query time, rebuild time, and total update behavior.

## Conclusion

SpatialGrid3D successfully reduced fixed lookup overhead and improved sparse/small-radius query cases. However, after lookup and materialization costs were reduced, dense and large-radius workloads remained limited by candidate traversal and rebuild trade-offs. For this project's current workload, SpatialHash3D remains the better production backend.

The positive outcome is that the experiment narrowed the problem: the hash backend is not retained by inertia, but because the measured trade-offs favor it for dense exact-neighbor workloads. The grid also improved the benchmark vocabulary around lookup overhead, candidate traversal, aggregate timing, and rebuild cost, which remains valuable for future performance work.

## Production guidance

- Keep `SpatialHash3D` as the production simulation backend.
- Treat `SpatialGrid3D` as an experimental reference and profiling artifact, not the default runtime path.
- Do not spend near-term effort on more neighbor-grid backend patches unless a new workload or benchmark target changes the decision criteria.
- Preserve the benchmark data and diagnostics because they explain the decision and help future comparisons.

## Better next investments

For the current project goals, future work should prioritize:

1. simulation-level optimizations that reduce candidate work or repeated passes without changing the backend;
2. aggregate-social specialization, especially where it reduces exact-neighbor pressure while preserving behavior quality;
3. documentation, benchmark presentation, and portfolio-readable explanations of the measured trade-offs;
4. broader C++ learning and code-quality improvements that strengthen the project beyond this one backend experiment.

## Historical acceptance gate

The original production-switch gate required the grid to improve query timing after rebuild cost, improve aggregate-social timing, preserve behavior-equivalent candidate counts, and avoid dense/clustered p95/p99 regressions. The current evidence does not clear that gate, so the experiment is frozen rather than promoted.
