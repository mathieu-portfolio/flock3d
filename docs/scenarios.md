# Scenario reference

`flock3d` scenarios are plain-data definitions that select a simulation model, default seed, environment settings, behavior weights, constraints, and metric settings. Switching scenarios applies the scenario defaults and resets the simulation from the scenario seed. Resetting with the same seed recreates the same initial positions and velocities.

This document keeps model-level detail out of the top-level README. Experiment-runner syntax, CSV export modes, sweeps, presets, and plotting commands live in [experiments.md](experiments.md).

## Shared boid model

All implemented scenarios build on local flocking forces:

- **Separation**: steer away from nearby agents inside the separation radius.
- **Alignment**: steer toward the average heading of perceived neighbors.
- **Cohesion**: steer toward the local neighbor center.

Shared parameters include `boid_count`, `world_half_extent`, `neighbor_radius`, `separation_radius`, `separation_weight`, `alignment_weight`, `cohesion_weight`, `max_speed`, `max_force`, `boid_scale`, `spatial_cell_size`, and `random_seed`.

Shared metrics include `polarization`, `cohesion`, `dispersion`, `average_speed`, `average_neighbors`, `nearest_neighbor_distance`, `simulation_update_time`, `neighbor_queries`, and `spatial_hash_cell_count`.

## ClassicBoids

### Purpose

ClassicBoids is the baseline Reynolds-style 3D flocking scenario. It is the reference point for checking whether later scenario constraints improve, preserve, or degrade collective order.

### Parameters

- `separation_weight`, `alignment_weight`, `cohesion_weight`: relative influence of the three local steering rules.
- `neighbor_radius`: perception radius for alignment and cohesion.
- `separation_radius`: closer-range avoidance radius.
- `max_speed` and `max_force`: velocity and steering-force caps.
- `world_half_extent`: size of the wrapped 3D domain.
- `random_seed`: reproducible initial position and velocity seed.

### Metrics

- `polarization`: global velocity alignment order.
- `cohesion`: average distance to the flock center of mass.
- `dispersion`: root-mean-square distance to the flock center of mass.
- `average_neighbors`: accepted neighbors per boid from spatial-hash queries.
- `nearest_neighbor_distance`: average nearest observed neighbor distance.

### Suggested experiments

- Sweep `perception_radius` to find the transition between fragmented and globally aligned motion.
- Sweep `alignment_weight` and compare mean `polarization`.
- Increase `boid_count` while watching simulation update time and spatial-hash candidate diagnostics.

## BirdFlight

![Birds](/resources/birds.png)

### Purpose

BirdFlight studies how simple flight constraints change flock stability relative to ClassicBoids. It is intentionally not an aerodynamic model; it is a controlled scenario for gravity, lift, perception, and maneuverability tradeoffs.

### Parameters

- `gravity`: downward acceleration applied every fixed simulation step.
- `lift_strength`: baseline upward acceleration that can counter gravity.
- `altitude_target`: preferred vertical center.
- `altitude_band`: tolerated vertical band around the target before correction is applied.
- `altitude_correction_strength`: proportional acceleration back toward the altitude band.
- `min_speed`: simple minimum airspeed / stall-prevention threshold.
- `max_climb_rate`: vertical velocity cap.
- `max_turn_rate`: maximum heading change in degrees per second.
- `field_of_view_degrees`: forward perception cone for ignoring neighbors behind or outside view.
- Shared flocking parameters still control separation, alignment, and cohesion.

### Metrics

- `mean_altitude`: average vertical position of the flock.
- `altitude_variance`: vertical spread around `mean_altitude`.
- `stall_count`: number of birds below `min_speed`.
- `near_ground_count`: number of birds at or below the simple `y <= 0` ground reference.
- Shared metrics such as `polarization`, `cohesion`, `dispersion`, and `average_neighbors` remain useful for comparing against ClassicBoids.

### Suggested experiments

- Sweep `field_of_view_degrees` and plot `polarization`, `dispersion`, `average_neighbors`, and `stall_count` to measure the cost of narrow forward perception.
- Sweep `max_turn_rate` and plot `polarization`, `dispersion`, `cohesion`, and `altitude_variance` to measure how maneuverability limits change collective order and vertical stability.
- Gravity remains useful as a secondary altitude-control sweep, but BirdFlight's curated study focuses on perception and maneuverability constraints.
- Compare `bird_baseline`, `bird_low_lift`, `bird_high_gravity`, `bird_narrow_fov`, and `bird_low_turn_rate` presets with summary exports.

## FishSchool

![Fish](/resources/fish.png)

### Purpose

FishSchool studies schooling inside a resistive underwater-style medium. It keeps shared separation/alignment/cohesion steering but adds drag, depth preference, smooth turning, buoyancy, and optional current.

### Parameters

- `drag_coefficient`: velocity damping from medium resistance.
- `buoyancy_strength`: constant positive-`y` acceleration.
- `target_depth`: preferred `y` coordinate; default values use negative `y` as deeper water.
- `depth_band`: tolerated range around `target_depth` before depth correction is applied.
- `depth_correction_strength`: acceleration back toward the target-depth band.
- `current_strength`: magnitude of an optional constant current influence.
- `current_direction`: direction of the current before normalization.
- `max_turn_rate`: heading-change limit for smoother, slower motion.
- FishSchool also adjusts shared defaults such as lower speed limits, larger neighbor radius, and stronger cohesion.

### Metrics

- `mean_depth`: average `y` position of the school.
- `depth_variance`: spread around `mean_depth`.
- `average_speed`: useful for seeing how drag and currents affect motion.
- Shared metrics such as `polarization`, `cohesion`, `dispersion`, and `average_neighbors` remain exported.

### Suggested experiments

- Sweep `drag_coefficient` and plot `polarization`, `cohesion`, `average_speed`, and `depth_variance` to measure how resistive damping affects alignment, grouping, speed, and depth keeping.
- Sweep `current_strength` to compare current-driven drift against polarization and dispersion.
- Compare `fish_baseline`, `fish_high_drag`, `fish_strong_current`, and `fish_low_visibility` presets.

## NoiseExperiment

![Noise](/resources/noise.png)

### Purpose

NoiseExperiment asks how much noisy local information or noisy actuation a flock can tolerate before macroscopic order breaks down. It uses the ClassicBoids force model, then applies deterministic perturbations controlled by the simulation seed and `noise_seed_offset`.

### Parameters

- `perception_noise_strength`: perturbs perceived neighbor offsets and alignment directions before the flocking sums are evaluated.
- `steering_noise_strength`: perturbs each boid's final steering acceleration before force clamping.
- `velocity_noise_strength`: optionally perturbs velocity after integration, followed by speed clamping.
- `noise_seed_offset`: separates the deterministic noise stream from the initial-state seed.
- `noise_enabled`: toggles the perturbation path. With all strengths at zero, matched ClassicBoids parameters reproduce the baseline behavior.

### Metrics

- `noise_strength`: combined noise strength reported for noise runs.
- `order_loss`: `1 - polarization`, useful for direct robustness comparisons.
- `polarization`, `dispersion`, `cohesion`, and `average_neighbors` are the primary macroscopic observables.

### Suggested experiments

- Sweep `steering_noise_strength` and plot `polarization`, `order_loss`, `dispersion`, and `cohesion` to measure collective-order degradation, loss of cohesion, and spatial spreading.
- Repeat with `perception_noise_strength` to compare sensing noise against actuation noise.
- Compare `noise_baseline`, `noise_low`, `noise_medium`, and `noise_high` presets.

## PredatorPrey placeholder

### Purpose

PredatorPrey reserves a scenario for future predator/prey roles, pursuit/evasion behavior, and role-specific observables. It currently reuses ClassicBoids steering with a distinct scenario name, seed, and slightly stronger separation defaults.

### Parameters

- Current: shared ClassicBoids parameters.
- Planned: predator count, prey count, pursuit gain, evasion gain, capture radius, predator speed multiplier, and role-specific perception ranges.

### Metrics

- Current: shared flock metrics.
- Planned: capture events, prey survival time, predator-prey distance, predator energy or effort, and prey cluster fragmentation.

### Suggested experiments

- After implementation, sweep predator count or pursuit gain against survival time.
- Compare prey polarization before and after predator introduction.

## ObstacleAvoidance placeholder

### Purpose

ObstacleAvoidance reserves a scenario for future obstacle fields and local avoidance responses. It currently reuses ClassicBoids steering with a distinct world extent and seed.

### Parameters

- Current: shared ClassicBoids parameters.
- Planned: obstacle count, obstacle radius distribution, avoidance lookahead, avoidance force, and static or moving obstacle layouts.

### Metrics

- Current: shared flock metrics.
- Planned: collision count, near-miss count, path deviation, mean clearance, and post-obstacle regrouping time.

### Suggested experiments

- After implementation, sweep obstacle density against collision count and polarization.
- Compare obstacle layouts by regrouping time and dispersion.

## Leadership placeholder

### Purpose

Leadership reserves a scenario for future informed-leader experiments, information propagation, and leader/follower heterogeneity. It currently reuses ClassicBoids steering with a distinct seed and altered alignment/cohesion defaults.

### Parameters

- Current: shared ClassicBoids parameters.
- Planned: leader fraction, leader target direction, leader confidence, follower responsiveness, information radius, and leader switching schedule.

### Metrics

- Current: shared flock metrics.
- Planned: target-heading error, convergence time, leader influence, follower lag, and consensus stability.

### Suggested experiments

- After implementation, sweep leader fraction against target-heading convergence time.
- Compare high-alignment and low-alignment regimes for information propagation.
