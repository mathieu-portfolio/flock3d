#!/usr/bin/env python3
"""Compare one-parameter sweep CSV exports by plotting mean metric per sweep value."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Group a flock3d sweep CSV by sweep value and plot the mean metric."
    )
    parser.add_argument("--input", required=True, help="Path to an input sweep CSV file.")
    parser.add_argument(
        "--sweep-column",
        required=True,
        help="CSV column containing the swept parameter values, such as sweep_value.",
    )
    parser.add_argument("--metric", required=True, help="Metric column to average per sweep value.")
    parser.add_argument("--output", required=True, help="Path for the output image file.")
    return parser.parse_args()


def require_columns(frame: pd.DataFrame, columns: list[str], csv_path: Path) -> None:
    missing = [column for column in columns if column not in frame.columns]
    if missing:
        available = ", ".join(frame.columns) if len(frame.columns) else "<none>"
        raise ValueError(
            f"Missing column(s) in {csv_path}: {', '.join(missing)}. "
            f"Available columns: {available}"
        )


def numeric_series(frame: pd.DataFrame, column: str, csv_path: Path) -> pd.Series:
    series = pd.to_numeric(frame[column], errors="coerce")
    if series.notna().sum() == 0:
        raise ValueError(f"Column '{column}' in {csv_path} has no numeric values to plot.")
    return series


def main() -> int:
    args = parse_args()
    csv_path = Path(args.input)
    output_path = Path(args.output)

    try:
        data = pd.read_csv(csv_path)
        require_columns(data, [args.sweep_column, args.metric], csv_path)
        numeric_data = pd.DataFrame(
            {
                args.sweep_column: numeric_series(data, args.sweep_column, csv_path),
                args.metric: numeric_series(data, args.metric, csv_path),
            }
        ).dropna()
        if numeric_data.empty:
            raise ValueError(
                f"Columns '{args.sweep_column}' and '{args.metric}' in {csv_path} "
                "have no overlapping numeric rows."
            )

        grouped = (
            numeric_data.groupby(args.sweep_column, as_index=False)[args.metric]
            .mean()
            .sort_values(args.sweep_column)
        )
        if grouped.empty:
            raise ValueError(f"No sweep groups found in {csv_path}.")

        x_label = args.sweep_column
        if "sweep_parameter" in data.columns:
            sweep_parameters = data["sweep_parameter"].dropna().astype(str).unique()
            if len(sweep_parameters) == 1 and sweep_parameters[0]:
                x_label = sweep_parameters[0]

        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(grouped[args.sweep_column], grouped[args.metric], marker="o", linewidth=1.5)
        ax.set_xlabel(x_label)
        ax.set_ylabel(f"mean {args.metric}")
        ax.set_title(f"Mean {args.metric} by {x_label}")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(output_path, dpi=150)
        plt.close(fig)
    except FileNotFoundError:
        raise SystemExit(f"Input CSV not found: {csv_path}")
    except pd.errors.EmptyDataError:
        raise SystemExit(f"Input CSV is empty: {csv_path}")
    except ValueError as error:
        raise SystemExit(str(error)) from error

    print(f"Wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
