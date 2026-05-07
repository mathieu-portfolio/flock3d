#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/study_common.sh"

study_repo_root
study_require_python_deps
study_build_runner
runner="$(study_runner_path)"

mkdir -p outputs/plots

csv="outputs/birdflight_gravity_sweep.csv"
plot="outputs/plots/gravity_vs_mean_altitude.png"

"${runner}" \
    --preset bird_baseline \
    --seed 2401 \
    --boids 512 \
    --duration 20 \
    --fixed-dt 0.008333333333333333 \
    --sample-rate 5 \
    --export-mode summary \
    --sweep gravity=6:14:2 \
    --output "${csv}"

python3 scripts/compare_sweeps.py \
    --input "${csv}" \
    --sweep-column sweep_value \
    --metric mean_altitude \
    --output "${plot}"

printf 'BirdFlight study complete:\n  CSV:  %s\n  Plot: %s\n' "${csv}" "${plot}"
