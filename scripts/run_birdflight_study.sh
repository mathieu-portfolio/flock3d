#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/study_common.sh"

study_repo_root
study_require_python_deps
study_build_runner
runner="$(study_runner_path)"

out_dir="outputs/birdflight"
mkdir -p "${out_dir}"

csv="${out_dir}/birdflight_gravity_sweep.csv"
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

study_plot_sweep_metric "${csv}" mean_altitude "${out_dir}/gravity_vs_mean_altitude.png"
study_plot_sweep_metric "${csv}" altitude_variance "${out_dir}/gravity_vs_altitude_variance.png"
study_plot_sweep_metric "${csv}" stall_count "${out_dir}/gravity_vs_stall_count.png"
study_plot_sweep_metric "${csv}" near_ground_count "${out_dir}/gravity_vs_near_ground_count.png"

printf 'BirdFlight study complete:\n  CSV:   %s\n  Plots: %s\n' "${csv}" "${out_dir}"
