#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
cd "${repo_root}"

python_bin="${PYTHON:-python}"

for arg in "$@"; do
    case "${arg}" in
        -h|--help)
            exec "${python_bin}" scripts/plot_benchmarks.py "$@"
            ;;
    esac
done

if ! "${python_bin}" - <<'PY' >/dev/null 2>&1
import matplotlib
import pandas
PY
then
    cat >&2 <<'MSG'
Missing Python plotting dependencies. Install them with:
  python -m pip install pandas matplotlib
MSG
    exit 1
fi

exec "${python_bin}" scripts/plot_benchmarks.py "$@"
