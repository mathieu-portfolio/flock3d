#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/study_common.sh"

study_repo_root
study_require_python_deps
study_build_runner
runner="$(study_runner_path)"

mkdir -p outputs/plots

csv="outputs/noise_steering_sweep.csv"
plot="outputs/plots/noise_strength_vs_polarization.png"

"${runner}" \
    --preset noise_baseline \
    --seed 7901 \
    --boids 512 \
    --duration 20 \
    --fixed-dt 0.008333333333333333 \
    --sample-rate 5 \
    --export-mode summary \
    --sweep steering_noise_strength=0:0.4:0.1 \
    --output "${csv}"

python scripts/compare_sweeps.py \
    --input "${csv}" \
    --sweep-column sweep_value \
    --metric polarization \
    --output "${plot}"

printf 'Noise study complete:\n  CSV:  %s\n  Plot: %s\n' "${csv}" "${plot}"
