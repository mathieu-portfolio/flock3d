#!/usr/bin/env bash
# Shared helpers for curated flock3d study scripts.

set -euo pipefail

study_repo_root() {
    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    cd "${script_dir}/.."
}

study_require_python_deps() {
    python - <<'PY'
import importlib.util
import sys

missing = [name for name in ("pandas", "matplotlib") if importlib.util.find_spec(name) is None]
if missing:
    print(
        "Missing Python plotting dependencies: " + ", ".join(missing) +
        ". Install them with: python -m pip install pandas matplotlib",
        file=sys.stderr,
    )
    raise SystemExit(1)
PY
}

study_build_runner() {
    local preset="${CMAKE_PRESET:-debug}"
    cmake --preset "${preset}"
    cmake --build --preset "${preset}" --target flock3d_experiment_runner
}

study_runner_path() {
    local preset="${CMAKE_PRESET:-debug}"
    local candidate
    local candidates=(
        "build/${preset}/src/experiment/flock3d_experiment_runner"
        "build/${preset}/src/experiment/Debug/flock3d_experiment_runner"
        "build/${preset}/src/experiment/Release/flock3d_experiment_runner"
        "build/${preset}/src/experiment/RelWithDebInfo/flock3d_experiment_runner"
        "build/${preset}/bin/flock3d_experiment_runner"
        "build/${preset}/Debug/flock3d_experiment_runner"
        "build/${preset}/Release/flock3d_experiment_runner"
    )

    for candidate in "${candidates[@]}"; do
        if [[ -f "${candidate}" ]]; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done

    candidate="$(find "build/${preset}" -maxdepth 5 -type f -name 'flock3d_experiment_runner*' | head -n 1)"
    if [[ -n "${candidate}" ]]; then
        printf '%s\n' "${candidate}"
        return 0
    fi

    printf 'Could not locate flock3d_experiment_runner under build/%s after building.\n' "${preset}" >&2
    return 1
}


study_plot_sweep_metric() {
    local csv="$1"
    local metric="$2"
    local plot="$3"

    python scripts/compare_sweeps.py \
        --input "${csv}" \
        --sweep-column sweep_value \
        --metric "${metric}" \
        --output "${plot}"
}
