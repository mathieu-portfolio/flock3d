#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/study_common.sh"

study_repo_root
study_require_python_deps
study_build_runner
runner="$(study_runner_path)"

out_dir="outputs/birdflight"
mkdir -p "${out_dir}"

fov_csv="${out_dir}/birdflight_fov_sweep.csv"
"${runner}" \
    --preset bird_baseline \
    --seed 2401 \
    --boids 512 \
    --duration 20 \
    --fixed-dt 0.008333333333333333 \
    --sample-rate 5 \
    --export-mode summary \
    --sweep field_of_view_degrees=90:270:45 \
    --output "${fov_csv}"

study_plot_sweep_metric "${fov_csv}" polarization "${out_dir}/fov_vs_polarization.png"
study_plot_sweep_metric "${fov_csv}" dispersion "${out_dir}/fov_vs_dispersion.png"
study_plot_sweep_metric "${fov_csv}" average_neighbors "${out_dir}/fov_vs_average_neighbors.png"
study_plot_sweep_metric "${fov_csv}" stall_count "${out_dir}/fov_vs_stall_count.png"

turn_rate_csv="${out_dir}/birdflight_turn_rate_sweep.csv"
"${runner}" \
    --preset bird_baseline \
    --seed 2401 \
    --boids 512 \
    --duration 20 \
    --fixed-dt 0.008333333333333333 \
    --sample-rate 5 \
    --export-mode summary \
    --sweep max_turn_rate=30:150:30 \
    --output "${turn_rate_csv}"

study_plot_sweep_metric "${turn_rate_csv}" polarization "${out_dir}/turn_rate_vs_polarization.png"
study_plot_sweep_metric "${turn_rate_csv}" dispersion "${out_dir}/turn_rate_vs_dispersion.png"
study_plot_sweep_metric "${turn_rate_csv}" cohesion "${out_dir}/turn_rate_vs_cohesion.png"
study_plot_sweep_metric "${turn_rate_csv}" altitude_variance "${out_dir}/turn_rate_vs_altitude_variance.png"

printf 'BirdFlight study complete:\n  CSVs:  %s\n         %s\n  Plots: %s\n' "${fov_csv}" "${turn_rate_csv}" "${out_dir}"
