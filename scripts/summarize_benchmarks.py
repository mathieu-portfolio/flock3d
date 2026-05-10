#!/usr/bin/env python
"""Write compact latest-sample benchmark summaries from flock3d CSV exports."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class SummarySpec:
    name: str
    filename: str
    group_columns: tuple[str, ...]
    primary_metric: str
    sample_column: str = "sample_index"
    elapsed_column: str = "elapsed_seconds"
    iterations_column: str = "iterations_in_sample"


BENCHMARKS: tuple[SummarySpec, ...] = (
    SummarySpec("simulation_update", "simulation_update.csv", ("scenario", "model", "neighbor_mode"), "mean_ns_per_tick"),
    SummarySpec("spatial_hash", "spatial_hash.csv", ("scenario",), "mean_spatial_query_ns_per_tick"),
    SummarySpec("metrics", "metrics.csv", ("scenario", "metric_mode"), "mean_ns_per_tick"),
    SummarySpec("noise", "noise.csv", ("scenario", "noise_mode"), "mean_ns_per_tick"),
    SummarySpec(
        "simulation_ticks",
        "simulation_ticks.csv",
        ("scenario",),
        "mean_ns_per_tick",
        sample_column="repetition",
        elapsed_column="simulated_seconds",
        iterations_column="measured_ticks",
    ),
)

OUTPUT_COLUMNS = (
    "benchmark",
    "scenario",
    "mode",
    "boid_count",
    "sample_index",
    "elapsed_seconds",
    "simulated_seconds",
    "ticks_in_sample",
    "primary_metric",
    "primary_value",
    "p95_value",
    "p99_value",
    "iterations_in_sample",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize latest rows from focused benchmark CSV exports.")
    parser.add_argument(
        "--input-dir",
        default="outputs/benchmarks",
        help="Directory containing benchmark CSV files (default: outputs/benchmarks).",
    )
    parser.add_argument(
        "--output",
        default="outputs/benchmarks/benchmark_summary.csv",
        help="Summary CSV path (default: outputs/benchmarks/benchmark_summary.csv).",
    )
    parser.add_argument(
        "--benchmarks",
        default="all",
        help="Comma-separated benchmark names to summarize, or 'all' (default: all).",
    )
    parser.add_argument("--markdown", action="store_true", help="Also print a Markdown table to stdout.")
    return parser.parse_args()


def selected_specs(selection: str) -> list[SummarySpec]:
    if selection == "all":
        return list(BENCHMARKS)
    names = {name.strip() for name in selection.split(",") if name.strip()}
    known = {spec.name for spec in BENCHMARKS}
    unknown = sorted(names - known)
    if unknown:
        raise SystemExit(f"Unknown benchmark name(s): {', '.join(unknown)}. Known: {', '.join(sorted(known))}")
    return [spec for spec in BENCHMARKS if spec.name in names]


def require_columns(headers: Iterable[str], columns: Iterable[str], csv_path: Path) -> None:
    header_set = set(headers)
    missing = [column for column in columns if column not in header_set]
    if missing:
        available = ", ".join(headers) if header_set else "<none>"
        raise SystemExit(f"Missing column(s) in {csv_path}: {', '.join(missing)}. Available columns: {available}")


def sort_value(row: dict[str, str], column: str) -> float:
    try:
        return float(row.get(column, ""))
    except ValueError:
        return float("-inf")


def mode_value(row: dict[str, str], spec: SummarySpec) -> str:
    mode_columns = [column for column in spec.group_columns if column != "scenario"]
    return " / ".join(row.get(column, "") for column in mode_columns) or row.get("scenario", "")


def latest_rows(spec: SummarySpec, input_dir: Path) -> list[dict[str, str]]:
    csv_path = input_dir / spec.filename
    if not csv_path.exists():
        print(f"Skipping {spec.name}: {csv_path} does not exist")
        return []

    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        require_columns(
            reader.fieldnames or (),
            (*spec.group_columns, "boid_count", spec.sample_column, spec.elapsed_column, spec.primary_metric),
            csv_path,
        )
        latest: dict[tuple[str, ...], dict[str, str]] = {}
        for row in reader:
            key = (*[row[column] for column in spec.group_columns], row["boid_count"])
            current = latest.get(key)
            if current is None or (
                sort_value(row, spec.sample_column), sort_value(row, spec.elapsed_column)
            ) > (sort_value(current, spec.sample_column), sort_value(current, spec.elapsed_column)):
                latest[key] = row

    summaries: list[dict[str, str]] = []
    for row in sorted(latest.values(), key=lambda item: (*[item[column] for column in spec.group_columns], sort_value(item, "boid_count"))):
        summaries.append(
            {
                "benchmark": spec.name,
                "scenario": row.get("scenario", ""),
                "mode": mode_value(row, spec),
                "boid_count": row.get("boid_count", ""),
                "sample_index": row.get(spec.sample_column, ""),
                "elapsed_seconds": row.get(spec.elapsed_column, ""),
                "simulated_seconds": row.get("simulated_seconds", row.get(spec.elapsed_column, "")),
                "ticks_in_sample": row.get("ticks_in_sample", row.get(spec.iterations_column, "")),
                "primary_metric": spec.primary_metric,
                "primary_value": row.get(spec.primary_metric, ""),
                "p95_value": row.get("p95_update_ms", row.get("p95_ms_per_tick", "")),
                "p99_value": row.get("p99_update_ms", row.get("p99_ms_per_tick", "")),
                "iterations_in_sample": row.get(spec.iterations_column, ""),
            }
        )
    return summaries


def print_markdown(rows: list[dict[str, str]]) -> None:
    print("| " + " | ".join(OUTPUT_COLUMNS) + " |")
    print("| " + " | ".join("---" for _ in OUTPUT_COLUMNS) + " |")
    for row in rows:
        print("| " + " | ".join(row[column] for column in OUTPUT_COLUMNS) + " |")


def main() -> int:
    args = parse_args()
    input_dir = Path(args.input_dir)
    output_path = Path(args.output)

    rows: list[dict[str, str]] = []
    for spec in selected_specs(args.benchmarks):
        rows.extend(latest_rows(spec, input_dir))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)

    if args.markdown:
        print_markdown(rows)
    print(f"Wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
