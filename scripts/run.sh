#!/usr/bin/env bash
set -euo pipefail

preset="${CMAKE_PRESET:-debug}"
if [[ $# -gt 0 && "${1}" != "--" ]]; then
    preset="${1}"
    shift
fi
if [[ $# -gt 0 && "${1}" == "--" ]]; then
    shift
fi

cmake --preset "${preset}"
cmake --build --preset "${preset}"

configuration="Debug"
if [[ "${preset}" == *release* ]]; then
    configuration="Release"
fi

build_dir="build/${preset}"
candidates=(
    "${build_dir}/flock3d"
    "${build_dir}/${configuration}/flock3d"
    "${build_dir}/flock3d.exe"
    "${build_dir}/${configuration}/flock3d.exe"
)

for executable in "${candidates[@]}"; do
    if [[ -f "${executable}" ]]; then
        exec "${executable}" "$@"
    fi
done

printf 'Unable to find flock3d executable for preset %s. Looked in:\n' "${preset}" >&2
printf '  %s\n' "${candidates[@]}" >&2
exit 1
