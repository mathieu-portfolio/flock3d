#!/usr/bin/env bash
set -euo pipefail

preset="${CMAKE_PRESET:-debug}"

if [[ $# -gt 0 && "${1}" != "--" ]]; then
    preset="$1"
    shift
fi

if [[ $# -gt 0 && "${1}" == "--" ]]; then
    shift
fi

"$(dirname "$0")/build.sh" "${preset}"

build_dir="build/${preset}"
candidates=(
    "${build_dir}/bin/flock3d"
    "${build_dir}/bin/flock3d.exe"
)

for executable in "${candidates[@]}"; do
    if [[ -f "${executable}" ]]; then
        printf 'Running executable: %s\n' "${executable}"
        exec "${executable}" "$@"
    fi
done

printf 'Unable to find flock3d executable for preset %s. Looked in:\n' "${preset}" >&2
printf '  %s\n' "${candidates[@]}" >&2
exit 1