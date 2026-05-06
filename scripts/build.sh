#!/usr/bin/env bash
set -euo pipefail

preset="${1:-${CMAKE_PRESET:-debug}}"

cmake --preset "${preset}"
cmake --build --preset "${preset}"
