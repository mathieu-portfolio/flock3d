# SpatialGrid3D profile summary

## Status

This profile is frozen as the final record of the `SpatialGrid3D` experiment. The backend remains useful as an experimental comparison point, but it should not be switched into production simulation yet.

## Final benchmark conclusion

SpatialGrid3D successfully reduced fixed lookup overhead and improved sparse/small-radius query cases. However, after lookup and materialization costs were reduced, dense and large-radius workloads remained limited by candidate traversal and rebuild trade-offs. For this project's current workload, SpatialHash3D remains the better production backend.

The experiment was still successful: it showed which costs were real, which optimizations helped, and why the hash backend remains the measured production choice for exact-neighbor flocking.

## What the profiling clarified

- Row-oriented traversal reduced the original per-cell lookup problem by replacing many cell-level searches with row/span scans.
- Callback/direct traversal reduced temporary result materialization overhead in query-heavy paths.
- Additional diagnostics made it possible to separate fixed lookup cost from occupied-cell traversal, candidates tested, aggregate timing, and rebuild time.
- Dense and large-radius cases remained dominated by candidate traversal and rebuild trade-offs after lookup overhead was reduced.

## Decision

Freeze `SpatialGrid3D` neighbor-query optimization work for now. Do not add more backend patches for the current milestone, and do not promote the grid to the default simulation path.

Better next investments are simulation-level optimizations, aggregate-social specialization, documentation and benchmark presentation, and broader C++ learning/code-quality work.
