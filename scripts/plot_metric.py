#!/usr/bin/env python3
"""Plot one metric column against another from a flock3d CSV export."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot a y metric against an x column from a flock3d CSV export."
    )
    parser.add_argument("--input", required=True, help="Path to an input CSV file.")
    parser.add_argument("--x", required=True, help="CSV column to use for the x-axis.")
    parser.add_argument("--y", required=True, help="CSV column to use for the y-axis.")
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
        require_columns(data, [args.x, args.y], csv_path)
        plot_data = pd.DataFrame(
            {
                args.x: numeric_series(data, args.x, csv_path),
                args.y: numeric_series(data, args.y, csv_path),
            }
        ).dropna()
        if plot_data.empty:
            raise ValueError(
                f"Columns '{args.x}' and '{args.y}' in {csv_path} have no overlapping numeric rows."
            )

        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(plot_data[args.x], plot_data[args.y], marker="o", linewidth=1.5)
        ax.set_xlabel(args.x)
        ax.set_ylabel(args.y)
        ax.set_title(f"{args.y} vs {args.x}")
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
