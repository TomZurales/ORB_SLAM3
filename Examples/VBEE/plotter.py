#!/usr/bin/env python3
"""
Dynamic CSV plotter with optional mean±std aggregation.

Assumptions:
  - CSV has at least: world_idx, iteration
  - All other columns are treated as numeric series (non-numeric are coerced to NaN).
  - Rows may arrive out of order; we sort by (world_idx, iteration).

Features:
  - Plot per-world lines for selected columns (default: all non-key numeric cols).
  - Optional: for chosen metrics, also plot iteration-wise mean with ±1 std as error bars
    computed across world_idx at each iteration.

Examples:
  python plot_dynamic.py data.csv
  python plot_dynamic.py data.csv --cols seen observability
  python plot_dynamic.py data.csv --world 26
  python plot_dynamic.py data.csv --avg observability
  python plot_dynamic.py data.csv --avg observability seen --avg-only
  python plot_dynamic.py data.csv --avg observability --avg-every 5
  python plot_dynamic.py data.csv --save-prefix out/run1 --avg observability
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


KEY_COLS = ["world_idx", "iteration"]


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path, help="Input CSV file")

    ap.add_argument("--world", type=int, default=None, help="Plot only this world_idx")

    ap.add_argument(
        "--cols",
        nargs="+",
        default=None,
        help="Subset of columns to plot (default: all non-key columns)",
    )

    ap.add_argument(
        "--all-in-one",
        action="store_true",
        help="Plot all selected columns on a single figure (can get busy)",
    )

    ap.add_argument(
        "--save-prefix",
        type=str,
        default=None,
        help="If set, save figures as '<prefix>_<col>.png' (or '<prefix>_all.png')",
    )

    ap.add_argument(
        "--max-worlds",
        type=int,
        default=None,
        help="Only plot the first N world_idx values (sorted). Useful for huge logs.",
    )

    # Aggregation options
    ap.add_argument(
        "--avg",
        nargs="+",
        default=None,
        help=(
            "One or more metric columns for which to plot iteration-wise mean with ±1 std "
            "across world_idx. Example: --avg observability seen"
        ),
    )
    ap.add_argument(
        "--avg-only",
        action="store_true",
        help="If set, do not plot per-world lines; plot only the mean±std for selected metrics.",
    )
    ap.add_argument(
        "--avg-every",
        type=int,
        default=1,
        help="Only draw error bars every N iterations to reduce clutter (default: 1 = every point).",
    )

    return ap.parse_args()


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)

    missing = [c for c in KEY_COLS if c not in df.columns]
    if missing:
        raise SystemExit(f"Missing required columns: {missing}")

    df["world_idx"] = pd.to_numeric(df["world_idx"], errors="raise").astype(int)
    df["iteration"] = pd.to_numeric(df["iteration"], errors="raise").astype(int)

    # Convert all non-key columns to numeric; non-numeric becomes NaN.
    for c in df.columns:
        if c in KEY_COLS:
            continue
        df[c] = pd.to_numeric(df[c], errors="coerce")

    df = df.sort_values(["world_idx", "iteration"], kind="mergesort").reset_index(drop=True)
    return df


def maybe_limit_worlds(df: pd.DataFrame, max_worlds: int | None) -> pd.DataFrame:
    if max_worlds is None:
        return df
    worlds = sorted(df["world_idx"].unique().tolist())[:max_worlds]
    return df[df["world_idx"].isin(worlds)].copy()


def choose_series_columns(df: pd.DataFrame, requested: list[str] | None) -> list[str]:
    candidates = [c for c in df.columns if c not in KEY_COLS]

    if requested is None:
        cols = candidates
    else:
        bad = [c for c in requested if c not in df.columns]
        if bad:
            raise SystemExit(f"Requested columns not found: {bad}\nAvailable: {list(df.columns)}")
        cols = requested

    # Only keep columns with at least one numeric value
    cols = [c for c in cols if not df[c].isna().all()]
    if not cols:
        raise SystemExit("No plottable columns found (all selected columns are non-numeric/empty).")
    return cols


def validate_avg_columns(df: pd.DataFrame, avg_cols: list[str] | None) -> list[str]:
    if not avg_cols:
        return []
    bad = [c for c in avg_cols if c not in df.columns or c in KEY_COLS]
    if bad:
        raise SystemExit(f"--avg columns not found / invalid: {bad}\nAvailable: {list(df.columns)}")
    avg_cols = [c for c in avg_cols if not df[c].isna().all()]
    if not avg_cols:
        raise SystemExit("--avg requested, but none of those columns contain numeric data.")
    return avg_cols


def compute_mean_std_by_iteration(df: pd.DataFrame, col: str) -> pd.DataFrame:
    """
    Returns a dataframe with columns: iteration, mean, std, n
    computed across world_idx per iteration.
    """
    g = df.groupby("iteration")[col]
    out = g.agg(mean="mean", std="std", n="count").reset_index()
    # std will be NaN when n == 1; keep it (matplotlib will just omit the bar)
    return out


def plot_column_per_world(
    df: pd.DataFrame,
    col: str,
    avg: bool,
    avg_every: int,
    avg_only: bool,
) -> plt.Figure:
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    if not avg_only:
        for world_idx, g in df.groupby("world_idx", sort=True):
            ax.plot(g["iteration"].to_numpy(), g[col].to_numpy(), label=f"world {world_idx}")

    if avg:
        stats = compute_mean_std_by_iteration(df, col)
        x = stats["iteration"].to_numpy()
        y = stats["mean"].to_numpy()
        yerr = stats["std"].to_numpy()

        if avg_every > 1:
            mask = (x % avg_every) == 0
            x, y, yerr = x[mask], y[mask], yerr[mask]

        ax.errorbar(x, y, yerr=yerr, fmt="-o", label=f"{col} mean±std")

    ax.set_title(f"{col} vs iteration (per world_idx)")
    ax.set_xlabel("iteration")
    ax.set_ylabel(col)
    ax.grid(True, which="both", linestyle="--", linewidth=0.5)

    # Keep legend readable
    ax.legend(ncol=2, fontsize=8)
    fig.tight_layout()
    return fig


def plot_all_columns(
    df: pd.DataFrame,
    cols: list[str],
    avg_cols: set[str],
    avg_every: int,
    avg_only: bool,
) -> plt.Figure:
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    if not avg_only:
        for world_idx, g in df.groupby("world_idx", sort=True):
            x = g["iteration"].to_numpy()
            for col in cols:
                ax.plot(x, g[col].to_numpy(), label=f"world {world_idx} • {col}")

    for col in cols:
        if col not in avg_cols:
            continue
        stats = compute_mean_std_by_iteration(df, col)
        x = stats["iteration"].to_numpy()
        y = stats["mean"].to_numpy()
        yerr = stats["std"].to_numpy()

        if avg_every > 1:
            mask = (x % avg_every) == 0
            x, y, yerr = x[mask], y[mask], yerr[mask]

        ax.errorbar(x, y, yerr=yerr, fmt="-o", label=f"{col} mean±std")

    ax.set_title("All series vs iteration (per world_idx)")
    ax.set_xlabel("iteration")
    ax.set_ylabel("value")
    ax.grid(True, which="both", linestyle="--", linewidth=0.5)
    ax.legend(ncol=2, fontsize=8)
    fig.tight_layout()
    return fig


def main() -> None:
    args = parse_args()
    df = load_csv(args.csv)

    if args.world is not None:
        df = df[df["world_idx"] == args.world].copy()
        if df.empty:
            raise SystemExit(f"No rows found for world_idx={args.world}")

    df = maybe_limit_worlds(df, args.max_worlds)

    cols = choose_series_columns(df, args.cols)
    avg_cols = set(validate_avg_columns(df, args.avg))

    # If user asked for avg-only but didn't specify avg cols, that's ambiguous.
    if args.avg_only and not avg_cols:
        raise SystemExit("--avg-only requires at least one column via --avg")

    figs: list[tuple[str, plt.Figure]] = []

    if args.all_in_one:
        figs.append(
            (
                "all",
                plot_all_columns(df, cols, avg_cols, args.avg_every, args.avg_only),
            )
        )
    else:
        for col in cols:
            figs.append(
                (
                    col,
                    plot_column_per_world(
                        df,
                        col,
                        avg=(col in avg_cols),
                        avg_every=args.avg_every,
                        avg_only=args.avg_only,
                    ),
                )
            )

    if args.save_prefix:
        for name, fig in figs:
            out = f"{args.save_prefix}_{name}.png"
            fig.savefig(out, dpi=200)
            print(f"Saved {out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
