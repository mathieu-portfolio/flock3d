#!/usr/bin/env bash
set -euo pipefail

clean=false
preset="${CMAKE_PRESET:-debug}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean)
            clean=true
            shift
            ;;
        *)
            preset="$1"
            shift
            ;;
    esac
done

if [[ "${clean}" == true ]]; then
    rm -rf "build/${preset}"
fi

cmake --preset "${preset}"
cmake --build --preset "${preset}"