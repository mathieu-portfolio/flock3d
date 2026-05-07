#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/study_common.sh"

study_repo_root
study_require_python_deps
study_build_runner
runner="$(study_runner_path)"

out_dir="outputs/noise"
mkdir -p "${out_dir}"

csv="${out_dir}/noise_steering_sweep.csv"
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

study_plot_sweep_metric "${csv}" polarization "${out_dir}/noise_strength_vs_polarization.png"
study_plot_sweep_metric "${csv}" order_loss "${out_dir}/noise_strength_vs_order_loss.png"
study_plot_sweep_metric "${csv}" dispersion "${out_dir}/noise_strength_vs_dispersion.png"
study_plot_sweep_metric "${csv}" cohesion "${out_dir}/noise_strength_vs_cohesion.png"

printf 'Noise study complete:\n  CSV:   %s\n  Plots: %s\n' "${csv}" "${out_dir}"
