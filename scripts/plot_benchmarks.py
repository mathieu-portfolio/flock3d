#!/usr/bin/env python
"""Generate benchmark summary CSVs and PNG plots from flock3d benchmark exports."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

plt = None
pd = None


def load_plotting_dependencies() -> None:
    global pd, plt
    import matplotlib
    import pandas as pandas_module

    matplotlib.use("Agg")
    import matplotlib.pyplot as pyplot_module

    pd = pandas_module
    plt = pyplot_module


@dataclass(frozen=True)
class BenchmarkSpec:
    name: str
    filename: str
    group_columns: tuple[str, ...]
    primary_metric: str
    time_metrics: tuple[str, ...]
    scaling_metrics: tuple[str, ...]
    diagnostic_metrics: tuple[str, ...] = ()
    sample_column: str = "sample_index"
    elapsed_column: str = "elapsed_seconds"


BENCHMARKS: tuple[BenchmarkSpec, ...] = (
    BenchmarkSpec(
        name="simulation_update",
        filename="simulation_update.csv",
        group_columns=("scenario", "model", "neighbor_mode"),
        primary_metric="mean_update_ms",
        time_metrics=("mean_update_ms", "min_update_ms", "max_update_ms"),
        scaling_metrics=("mean_update_ms", "min_update_ms", "max_update_ms"),
        diagnostic_metrics=(
            "parallel_workspace_ms",
            "parallel_dispatch_ms",
            "parallel_for_calls_mean",
            "parallel_worker_count_mean",
        ),
    ),
    BenchmarkSpec(
        name="spatial_hash",
        filename="spatial_hash.csv",
        group_columns=("scenario",),
        primary_metric="mean_spatial_query_ns_per_tick",
        time_metrics=(
            "mean_rebuild_ns_per_tick",
            "mean_spatial_query_ns_per_tick",
            "mean_naive_query_ns_per_tick",
            "p99_spatial_query_ms",
        ),
        scaling_metrics=("mean_rebuild_ns_per_tick", "mean_spatial_query_ns_per_tick", "mean_naive_query_ns_per_tick"),
        diagnostic_metrics=(
            "candidates_per_query",
            "visited_cells_per_query",
            "effective_neighbors_per_query",
            "occupied_cell_count",
            "max_cell_occupancy",
        ),
    ),
    BenchmarkSpec(
        name="metrics",
        filename="metrics.csv",
        group_columns=("scenario", "metric_mode"),
        primary_metric="mean_ns_per_tick",
        time_metrics=("mean_ns_per_tick", "p95_update_ms", "p99_update_ms", "ticks_per_second", "real_time_factor"),
        scaling_metrics=("mean_ns_per_tick", "p95_update_ms", "p99_update_ms", "ticks_per_second"),
    ),
    BenchmarkSpec(
        name="noise",
        filename="noise.csv",
        group_columns=("scenario", "noise_mode"),
        primary_metric="mean_ns_per_tick",
        time_metrics=("mean_ns_per_tick", "p95_update_ms", "p99_update_ms", "ticks_per_second", "real_time_factor"),
        scaling_metrics=("mean_ns_per_tick", "p95_update_ms", "p99_update_ms", "ticks_per_second"),
    ),
    BenchmarkSpec(
        name="aggregate_social",
        filename="aggregate_social.csv",
        group_columns=("scenario", "aggregate_social_mode"),
        primary_metric="mean_update_ms",
        time_metrics=("mean_update_ms", "min_update_ms", "max_update_ms"),
        scaling_metrics=("mean_update_ms", "min_update_ms", "max_update_ms"),
        diagnostic_metrics=(
            "mean_metrics_update_ms",
            "visible_aggregate_cells_mean",
            "rejected_aggregate_cells_mean",
            "aggregate_cells_used_mean",
            "aggregate_query_radius_mean",
            "exact_separation_neighbors_mean",
            "exact_separation_neighbors_max",
            "social_weight_sum_mean",
            "flock_spread",
            "polarization",
        ),
    ),
    BenchmarkSpec(
        name="simulation_ticks",
        filename="simulation_ticks.csv",
        group_columns=("scenario",),
        primary_metric="mean_ns_per_tick",
        time_metrics=("mean_ns_per_tick", "p95_ms_per_tick", "p99_ms_per_tick", "ticks_per_second", "real_time_factor"),
        scaling_metrics=("mean_ns_per_tick", "p95_ms_per_tick", "p99_ms_per_tick", "ticks_per_second", "real_time_factor"),
        sample_column="repetition",
        elapsed_column="simulated_seconds",
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot focused benchmark CSV exports and write summary tables."
    )
    parser.add_argument(
        "--input-dir",
        default="outputs/benchmarks",
        help="Directory containing benchmark CSV files (default: outputs/benchmarks).",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory for plots and summaries (default: <input-dir>/plots).",
    )
    parser.add_argument(
        "--format",
        default="png",
        choices=("png", "pdf", "svg"),
        help="Plot file format (default: png).",
    )
    parser.add_argument(
        "--benchmarks",
        default="all",
        help="Comma-separated benchmark names to plot, or 'all' (default: all).",
    )
    return parser.parse_args()


def selected_specs(selection: str) -> list[BenchmarkSpec]:
    if selection == "all":
        return list(BENCHMARKS)
    names = {name.strip() for name in selection.split(",") if name.strip()}
    known = {spec.name for spec in BENCHMARKS}
    unknown = sorted(names - known)
    if unknown:
        raise SystemExit(f"Unknown benchmark name(s): {', '.join(unknown)}. Known: {', '.join(sorted(known))}")
    return [spec for spec in BENCHMARKS if spec.name in names]


def require_columns(frame: pd.DataFrame, columns: Iterable[str], csv_path: Path) -> None:
    missing = [column for column in columns if column not in frame.columns]
    if missing:
        available = ", ".join(frame.columns) if len(frame.columns) else "<none>"
        raise ValueError(
            f"Missing column(s) in {csv_path}: {', '.join(missing)}. Available columns: {available}"
        )


def available_spec(spec: BenchmarkSpec, frame: pd.DataFrame) -> BenchmarkSpec:
    def available(metrics: tuple[str, ...]) -> tuple[str, ...]:
        return tuple(metric for metric in metrics if metric in frame.columns)

    return BenchmarkSpec(
        name=spec.name,
        filename=spec.filename,
        group_columns=spec.group_columns,
        primary_metric=spec.primary_metric,
        time_metrics=available(spec.time_metrics),
        scaling_metrics=available(spec.scaling_metrics),
        diagnostic_metrics=available(spec.diagnostic_metrics),
        sample_column=spec.sample_column,
        elapsed_column=spec.elapsed_column,
    )


def coerce_numeric(frame: pd.DataFrame, columns: Iterable[str], csv_path: Path) -> pd.DataFrame:
    converted = frame.copy()
    for column in columns:
        converted[column] = pd.to_numeric(converted[column], errors="coerce")
        if converted[column].notna().sum() == 0:
            raise ValueError(f"Column '{column}' in {csv_path} has no numeric values.")
    return converted


def line_label(row: pd.Series, group_columns: tuple[str, ...]) -> str:
    parts = [str(row[column]) for column in group_columns if column in row.index]
    parts.append(f"{int(row['boid_count'])} boids")
    return " / ".join(parts)


def latest_samples(spec: BenchmarkSpec, frame: pd.DataFrame) -> pd.DataFrame:
    sort_columns = [*spec.group_columns, "boid_count", spec.sample_column, spec.elapsed_column]
    latest = frame.sort_values(sort_columns).groupby([*spec.group_columns, "boid_count"], as_index=False).tail(1)
    return latest.sort_values([*spec.group_columns, "boid_count"])


def write_summary(spec: BenchmarkSpec, frame: pd.DataFrame, output_dir: Path) -> Path:
    latest = latest_samples(spec, frame)
    columns = [
        *spec.group_columns,
        "boid_count",
        spec.sample_column,
        spec.elapsed_column,
        spec.primary_metric,
    ]
    optional_columns = [
        column
        for column in (
            "simulated_seconds",
            "simulated_ticks",
            "ticks_in_sample",
            "iterations_in_sample",
            "sample_wall_seconds",
            "total_wall_seconds",
            "p95_update_ms",
            "p99_update_ms",
            "p95_ms_per_tick",
            "p99_ms_per_tick",
            "max_candidates_per_query",
            "max_effective_neighbors_per_query",
            "visited_cells_per_query",
            "max_cell_occupancy",
        )
        if column in latest.columns
    ]
    summary = latest[[*columns, *optional_columns]].copy()
    output_path = output_dir / f"{spec.name}_summary.csv"
    summary.to_csv(output_path, index=False)
    return output_path


def plot_time_series(spec: BenchmarkSpec, frame: pd.DataFrame, output_dir: Path, file_format: str) -> list[Path]:
    paths: list[Path] = []
    if not spec.time_metrics:
        return paths
    for metric in spec.time_metrics:
        fig, ax = plt.subplots(figsize=(10, 5.5))
        for _, group in frame.groupby([*spec.group_columns, "boid_count"], dropna=False):
            group = group.sort_values(spec.elapsed_column)
            ax.plot(group[spec.elapsed_column], group[metric], marker="o", linewidth=1.2, label=line_label(group.iloc[0], spec.group_columns))
        ax.set_title(f"{spec.name}: {metric} over simulated time")
        ax.set_xlabel(f"{spec.elapsed_column} (simulated)")
        ax.set_ylabel(metric)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize="x-small", ncols=2)
        fig.tight_layout()
        output_path = output_dir / f"{spec.name}_{metric}_time_series.{file_format}"
        fig.savefig(output_path, dpi=150)
        plt.close(fig)
        paths.append(output_path)
    return paths


def plot_scaling(spec: BenchmarkSpec, frame: pd.DataFrame, output_dir: Path, file_format: str) -> Path | None:
    if not spec.scaling_metrics:
        return None
    latest = latest_samples(spec, frame)
    averaged = latest.groupby([*spec.group_columns, "boid_count"], as_index=False)[list(spec.scaling_metrics)].mean()

    fig, axes = plt.subplots(len(spec.scaling_metrics), 1, figsize=(9, 4.0 * len(spec.scaling_metrics)), squeeze=False)
    for axis, metric in zip(axes[:, 0], spec.scaling_metrics):
        for _, group in averaged.groupby(list(spec.group_columns), dropna=False):
            group = group.sort_values("boid_count")
            label = " / ".join(str(group.iloc[0][column]) for column in spec.group_columns)
            axis.plot(group["boid_count"], group[metric], marker="o", linewidth=1.5, label=label)
        axis.set_title(f"Latest sample: {metric} by boid count")
        axis.set_xlabel("boid_count")
        axis.set_ylabel(metric)
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize="small")
    fig.tight_layout()
    output_path = output_dir / f"{spec.name}_latest_scaling.{file_format}"
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def plot_diagnostics(spec: BenchmarkSpec, frame: pd.DataFrame, output_dir: Path, file_format: str) -> list[Path]:
    paths: list[Path] = []
    if not spec.diagnostic_metrics:
        return paths
    latest = latest_samples(spec, frame)
    for metric in spec.diagnostic_metrics:
        fig, ax = plt.subplots(figsize=(9, 5))
        for _, group in latest.groupby(list(spec.group_columns), dropna=False):
            group = group.sort_values("boid_count")
            label = " / ".join(str(group.iloc[0][column]) for column in spec.group_columns)
            ax.plot(group["boid_count"], group[metric], marker="o", linewidth=1.5, label=label)
        ax.set_title(f"{spec.name}: latest {metric}")
        ax.set_xlabel("boid_count")
        ax.set_ylabel(metric)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize="small")
        fig.tight_layout()
        output_path = output_dir / f"{spec.name}_{metric}_latest.{file_format}"
        fig.savefig(output_path, dpi=150)
        plt.close(fig)
        paths.append(output_path)
    return paths


def process_benchmark(spec: BenchmarkSpec, input_dir: Path, output_dir: Path, file_format: str) -> list[Path]:
    csv_path = input_dir / spec.filename
    if not csv_path.exists():
        print(f"Skipping {spec.name}: {csv_path} does not exist")
        return []

    frame = pd.read_csv(csv_path)
    require_columns(frame, [*spec.group_columns, "boid_count", spec.elapsed_column, spec.sample_column, spec.primary_metric], csv_path)
    metric_columns = {
        column
        for column in (*spec.time_metrics, *spec.scaling_metrics, *spec.diagnostic_metrics, spec.primary_metric)
        if column in frame.columns
    }
    numeric_columns = {"boid_count", spec.elapsed_column, spec.sample_column, *metric_columns}
    frame = coerce_numeric(frame, numeric_columns, csv_path).dropna(subset=["boid_count", spec.elapsed_column, spec.sample_column])
    if frame.empty:
        raise ValueError(f"No plottable rows found in {csv_path}.")

    plot_spec = available_spec(spec, frame)
    written = [write_summary(plot_spec, frame, output_dir)]
    written.extend(plot_time_series(plot_spec, frame, output_dir, file_format))
    scaling_plot = plot_scaling(plot_spec, frame, output_dir, file_format)
    if scaling_plot is not None:
        written.append(scaling_plot)
    written.extend(plot_diagnostics(plot_spec, frame, output_dir, file_format))
    return written


def main() -> int:
    args = parse_args()
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir) if args.output_dir else input_dir / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        load_plotting_dependencies()
    except ModuleNotFoundError as error:
        raise SystemExit(
            f"Missing Python plotting dependency: {error.name}. "
            "Install dependencies with: python -m pip install pandas matplotlib"
        ) from error

    all_written: list[Path] = []
    try:
        for spec in selected_specs(args.benchmarks):
            all_written.extend(process_benchmark(spec, input_dir, output_dir, args.format))
    except FileNotFoundError as error:
        raise SystemExit(str(error)) from error
    except pd.errors.EmptyDataError as error:
        raise SystemExit(f"Input CSV is empty: {error}") from error
    except ValueError as error:
        raise SystemExit(str(error)) from error

    if not all_written:
        print(f"No benchmark plots generated from {input_dir}")
        return 0

    for path in all_written:
        print(f"Wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
