#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
cd "${repo_root}"

preset="${FLOCK3D_BENCHMARK_PRESET:-${CMAKE_PRESET:-release}}"
output_dir="${FLOCK3D_BENCHMARK_OUTPUT_DIR:-outputs/benchmarks}"
duration="${FLOCK3D_BENCHMARK_DURATION:-}"
sample="${FLOCK3D_BENCHMARK_SAMPLE:-}"
warmup="${FLOCK3D_BENCHMARK_WARMUP:-}"
clean=false
skip_build=false
plot=false
summary=false
plot_args=()
selection="all"

benchmarks=(
    simulation_update
    spatial_hash
    metrics
    noise
    simulation_ticks
)

declare -A targets=(
    [simulation_update]=flock3d_simulation_update_benchmark
    [spatial_hash]=flock3d_spatial_hash_benchmark
    [metrics]=flock3d_metrics_benchmark
    [noise]=flock3d_noise_benchmark
    [simulation_ticks]=flock3d_simulation_ticks_benchmark
)

declare -A outputs=(
    [simulation_update]=simulation_update.csv
    [spatial_hash]=spatial_hash.csv
    [metrics]=metrics.csv
    [noise]=noise.csv
    [simulation_ticks]=simulation_ticks.csv
)

usage() {
    cat <<'USAGE'
Usage: scripts/run_benchmark.sh [options] [all|simulation_update|simulation_ticks|spatial_hash|metrics|noise] [-- benchmark-args...]

Build and run one or more focused benchmark executables, writing CSV files under
outputs/benchmarks/ by default.

Options:
  --preset PRESET       CMake preset to configure/build (default: release, or
                        FLOCK3D_BENCHMARK_PRESET/CMAKE_PRESET when set)
  --output-dir DIR      Directory for CSV output (default: outputs/benchmarks,
                        or FLOCK3D_BENCHMARK_OUTPUT_DIR when set)
  --duration SECONDS    Simulated duration per scenario, forwarded to benchmarks
  --sample SECONDS      Simulated CSV sample window length, forwarded to benchmarks
  --warmup SECONDS      Simulated warm-up per scenario, forwarded to benchmarks
  --clean               Remove build/<preset> before configuring
  --skip-build          Run existing benchmark binaries without rebuilding
  --summary             Write a compact latest-sample summary CSV after benchmarks run
  --plot                Generate benchmark plots after CSV files are written
  --plot-arg ARG        Extra argument forwarded to scripts/plot_benchmarks.py
  --list                Print benchmark names and exit
  -h, --help            Show this help

Environment defaults:
  FLOCK3D_BENCHMARK_PRESET, FLOCK3D_BENCHMARK_OUTPUT_DIR,
  FLOCK3D_BENCHMARK_DURATION, FLOCK3D_BENCHMARK_SAMPLE,
  FLOCK3D_BENCHMARK_WARMUP

Examples:
  scripts/run_benchmark.sh
  scripts/run_benchmark.sh --duration 0.2 --sample 0.1 --warmup 0 simulation_update
  scripts/run_benchmark.sh --preset release-ninja spatial_hash -- --duration 10
  scripts/run_benchmark.sh --duration 1 --sample 0.25 --summary --plot all
USAGE
}

list_benchmarks() {
    printf 'all\n'
    printf '%s\n' "${benchmarks[@]}"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --preset)
            preset="${2:?--preset requires a value}"
            shift 2
            ;;
        --output-dir)
            output_dir="${2:?--output-dir requires a value}"
            shift 2
            ;;
        --duration)
            duration="${2:?--duration requires a value}"
            shift 2
            ;;
        --sample)
            sample="${2:?--sample requires a value}"
            shift 2
            ;;
        --warmup)
            warmup="${2:?--warmup requires a value}"
            shift 2
            ;;
        --clean)
            clean=true
            shift
            ;;
        --skip-build)
            skip_build=true
            shift
            ;;
        --summary)
            summary=true
            shift
            ;;
        --plot)
            plot=true
            shift
            ;;
        --plot-arg)
            plot_args+=("${2:?--plot-arg requires a value}")
            shift 2
            ;;
        --list)
            list_benchmarks
            exit 0
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        all|simulation_update|simulation_ticks|spatial_hash|metrics|noise)
            selection="$1"
            shift
            ;;
        *)
            printf 'Unknown option or benchmark: %s\n\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

extra_args=("$@")
benchmark_args=()
[[ -n "${duration}" ]] && benchmark_args+=(--duration "${duration}")
[[ -n "${sample}" ]] && benchmark_args+=(--sample "${sample}")
[[ -n "${warmup}" ]] && benchmark_args+=(--warmup "${warmup}")
benchmark_args+=("${extra_args[@]}")

selected=()
if [[ "${selection}" == "all" ]]; then
    selected=("${benchmarks[@]}")
else
    selected=("${selection}")
fi

if [[ "${clean}" == true ]]; then
    rm -rf "build/${preset}"
fi

if [[ "${skip_build}" == false ]]; then
    cmake --preset "${preset}"
    for benchmark in "${selected[@]}"; do
        cmake --build --preset "${preset}" --target "${targets[${benchmark}]}"
    done
fi

mkdir -p "${output_dir}"

find_executable() {
    local target="$1"
    local candidate
    local candidates=(
        "build/${preset}/bin/${target}"
        "build/${preset}/bin/${target}.exe"
        "build/${preset}/Debug/${target}"
        "build/${preset}/Debug/${target}.exe"
        "build/${preset}/Release/${target}"
        "build/${preset}/Release/${target}.exe"
        "build/${preset}/benchmarks/${target}"
        "build/${preset}/benchmarks/${target}.exe"
    )

    for candidate in "${candidates[@]}"; do
        if [[ -f "${candidate}" && -x "${candidate}" ]]; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done

    candidate="$(find "build/${preset}" -maxdepth 5 -type f \( -name "${target}" -o -name "${target}.exe" \) | head -n 1)"
    if [[ -n "${candidate}" ]]; then
        printf '%s\n' "${candidate}"
        return 0
    fi

    printf 'Could not locate %s under build/%s.\n' "${target}" "${preset}" >&2
    return 1
}

for benchmark in "${selected[@]}"; do
    target="${targets[${benchmark}]}"
    executable="$(find_executable "${target}")"
    csv="${output_dir}/${outputs[${benchmark}]}"

    printf 'Running %s -> %s\n' "${target}" "${csv}" >&2
    "${executable}" "${benchmark_args[@]}" > "${csv}"
done

printf 'Benchmark CSV files written to %s\n' "${output_dir}" >&2

if [[ "${summary}" == true ]]; then
    summary_args=(--input-dir "${output_dir}" --output "${output_dir}/benchmark_summary.csv")
    if [[ "${selection}" != "all" ]]; then
        summary_args+=(--benchmarks "${selection}")
    fi
    scripts/summarize_benchmarks.py "${summary_args[@]}"
fi

if [[ "${plot}" == true ]]; then
    benchmark_selection="${selection}"
    if [[ "${benchmark_selection}" != "all" ]]; then
        plot_args+=(--benchmarks "${benchmark_selection}")
    fi
    scripts/plot_benchmarks.sh --input-dir "${output_dir}" "${plot_args[@]}"
fi
