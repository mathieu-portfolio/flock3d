#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/study_common.sh"

study_repo_root
study_require_python_deps
study_build_runner
runner="$(study_runner_path)"

out_dir="outputs/fishschool"
mkdir -p "${out_dir}"

csv="${out_dir}/fishschool_drag_sweep.csv"
"${runner}" \
    --preset fish_baseline \
    --seed 3109 \
    --boids 512 \
    --duration 20 \
    --fixed-dt 0.008333333333333333 \
    --sample-rate 5 \
    --export-mode summary \
    --sweep drag_coefficient=0:1:0.25 \
    --output "${csv}"

study_plot_sweep_metric "${csv}" polarization "${out_dir}/drag_vs_polarization.png"
study_plot_sweep_metric "${csv}" cohesion "${out_dir}/drag_vs_cohesion.png"
study_plot_sweep_metric "${csv}" average_speed "${out_dir}/drag_vs_average_speed.png"
study_plot_sweep_metric "${csv}" depth_variance "${out_dir}/drag_vs_depth_variance.png"

printf 'FishSchool study complete:\n  CSV:   %s\n  Plots: %s\n' "${csv}" "${out_dir}"
