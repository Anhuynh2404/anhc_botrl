#!/usr/bin/env python3
"""Generate comparison plots from the latest anhc benchmark CSV.

Plots produced (PNG, saved to ~/anhc_benchmark_results/plots/):
  1. Bar chart — mean planning_time_ms ± std per algorithm
  2. Bar chart — mean path_length_m ± std per algorithm
  3. Bar chart — mean nodes_expanded ± std per algorithm
  4. Scatter   — path_length_m vs planning_time_ms, coloured per algorithm

Usage:
    python3 src/anhc_viz/scripts/anhc_plot_results.py
    python3 src/anhc_viz/scripts/anhc_plot_results.py --csv /path/to/file.csv
"""

import argparse
import glob
import os
import sys

import os as _os

# Redirect matplotlib cache to a writable workspace-local directory when the
# default ~/.cache/matplotlib is not writable (e.g. in sandbox / CI environments).
if not _os.access(_os.path.expanduser("~/.cache/matplotlib"), _os.W_OK):
    _mpl_cache = _os.path.join(
        _os.path.dirname(_os.path.abspath(__file__)), ".mpl_cache"
    )
    _os.makedirs(_mpl_cache, exist_ok=True)
    _os.environ.setdefault("MPLCONFIGDIR", _mpl_cache)

import matplotlib
matplotlib.use("Agg")  # non-interactive backend — safe in headless environments
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

_DEFAULT_RESULTS_DIR = os.path.expanduser("~/anhc_benchmark_results")
_ALGO_COLORS = [
    "#2196F3",  # blue   — astar
    "#FF5722",  # orange — dijkstra
    "#4CAF50",  # green  — rrt_star
    "#9C27B0",  # purple — dstar_lite
    "#FF9800",  # amber  — rl
    "#00BCD4",  # cyan   — extra
]


def _latest_csv(results_dir: str) -> str:
    pattern = os.path.join(results_dir, "benchmark_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        return ""
    return files[-1]


def _normalize_schema(df: pd.DataFrame) -> pd.DataFrame:
    """Back-fill ``planning_success`` for v1 CSV files that pre-date the column.

    Inference rule (matches the v2 writer behaviour):
        planning_success = (success == True) OR (path_length_m > 0)
    """
    if "planning_success" not in df.columns:
        df = df.copy()
        inferred = df["success"].astype(bool) | (
            pd.to_numeric(df["path_length_m"], errors="coerce").fillna(0.0) > 0
        )
        insert_pos = (
            df.columns.get_loc("nodes_expanded") + 1
            if "nodes_expanded" in df.columns
            else len(df.columns)
        )
        df.insert(insert_pos, "planning_success", inferred)
    return df


def _algo_colors(algorithms: list[str]) -> dict[str, str]:
    return {a: _ALGO_COLORS[i % len(_ALGO_COLORS)] for i, a in enumerate(sorted(algorithms))}


def plot_bar(
    ax: plt.Axes,
    df: pd.DataFrame,
    metric: str,
    label: str,
    algo_colors: dict,
) -> None:
    """Draw a bar chart with error bars (mean ± std) for *metric* grouped by algorithm."""
    algos = sorted(df["algorithm_name"].unique())
    x = np.arange(len(algos))
    means, stds = [], []
    for algo in algos:
        col = pd.to_numeric(df.loc[df["algorithm_name"] == algo, metric], errors="coerce")
        means.append(col.mean())
        stds.append(col.std(ddof=0) if len(col) > 1 else 0.0)

    bars = ax.bar(
        x,
        means,
        yerr=stds,
        capsize=6,
        color=[algo_colors.get(a, "#607D8B") for a in algos],
        edgecolor="white",
        linewidth=0.8,
        error_kw={"elinewidth": 1.5, "ecolor": "black", "capthick": 1.5},
    )
    ax.set_xticks(x)
    ax.set_xticklabels(algos, fontsize=10)
    ax.set_ylabel(label, fontsize=10)
    ax.set_title(label, fontsize=11, fontweight="bold")
    ax.yaxis.grid(True, linestyle="--", alpha=0.6)
    ax.set_axisbelow(True)

    for bar, mean in zip(bars, means):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            bar.get_height() + max(stds) * 0.05 if stds else bar.get_height() * 0.02,
            f"{mean:.2f}",
            ha="center",
            va="bottom",
            fontsize=8,
        )


def plot_scatter(
    ax: plt.Axes,
    df: pd.DataFrame,
    algo_colors: dict,
) -> None:
    """Scatter: path_length_m vs planning_time_ms, one colour per algorithm."""
    for algo in sorted(df["algorithm_name"].unique()):
        grp = df[df["algorithm_name"] == algo]
        ax.scatter(
            pd.to_numeric(grp["planning_time_ms"], errors="coerce"),
            pd.to_numeric(grp["path_length_m"], errors="coerce"),
            label=algo,
            color=algo_colors.get(algo, "#607D8B"),
            alpha=0.75,
            s=60,
            edgecolors="white",
            linewidths=0.5,
        )
    ax.set_xlabel("Planning Time (ms)", fontsize=10)
    ax.set_ylabel("Path Length (m)", fontsize=10)
    ax.set_title("Path Length vs Planning Time", fontsize=11, fontweight="bold")
    ax.legend(fontsize=9)
    ax.yaxis.grid(True, linestyle="--", alpha=0.6)
    ax.xaxis.grid(True, linestyle="--", alpha=0.6)
    ax.set_axisbelow(True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot anhc benchmark results")
    parser.add_argument(
        "--csv",
        default="",
        help="Path to a specific CSV file. Defaults to the latest in ~/anhc_benchmark_results/.",
    )
    parser.add_argument(
        "--output-dir",
        default=os.path.join(_DEFAULT_RESULTS_DIR, "plots"),
        help="Directory where PNG files are saved.",
    )
    args = parser.parse_args()

    csv_path = args.csv or _latest_csv(_DEFAULT_RESULTS_DIR)
    if not csv_path:
        print(
            f"ERROR: no benchmark CSV found in {_DEFAULT_RESULTS_DIR}. "
            "Run the benchmark first.",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Reading: {csv_path}")
    df = _normalize_schema(pd.read_csv(csv_path))
    if df.empty:
        print(f"ERROR: {csv_path} is empty.", file=sys.stderr)
        sys.exit(1)

    output_dir = os.path.expanduser(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    algos = sorted(df["algorithm_name"].unique())
    algo_colors = _algo_colors(algos)

    plt.style.use("seaborn-v0_8-whitegrid")

    # ── Plot 1: planning_time_ms bar chart ────────────────────────────────────
    fig, ax = plt.subplots(figsize=(6, 4))
    plot_bar(ax, df, "planning_time_ms", "Planning Time (ms)", algo_colors)
    fig.tight_layout()
    p1 = os.path.join(output_dir, "01_planning_time.png")
    fig.savefig(p1, dpi=120)
    plt.close(fig)
    print(f"  Saved: {p1}")

    # ── Plot 2: path_length_m bar chart ───────────────────────────────────────
    fig, ax = plt.subplots(figsize=(6, 4))
    plot_bar(ax, df, "path_length_m", "Path Length (m)", algo_colors)
    fig.tight_layout()
    p2 = os.path.join(output_dir, "02_path_length.png")
    fig.savefig(p2, dpi=120)
    plt.close(fig)
    print(f"  Saved: {p2}")

    # ── Plot 3: nodes_expanded bar chart ──────────────────────────────────────
    fig, ax = plt.subplots(figsize=(6, 4))
    plot_bar(ax, df, "nodes_expanded", "Nodes Expanded", algo_colors)
    fig.tight_layout()
    p3 = os.path.join(output_dir, "03_nodes_expanded.png")
    fig.savefig(p3, dpi=120)
    plt.close(fig)
    print(f"  Saved: {p3}")

    # ── Plot 4: scatter path_length vs planning_time ──────────────────────────
    fig, ax = plt.subplots(figsize=(6, 4))
    plot_scatter(ax, df, algo_colors)
    fig.tight_layout()
    p4 = os.path.join(output_dir, "04_scatter_length_vs_time.png")
    fig.savefig(p4, dpi=120)
    plt.close(fig)
    print(f"  Saved: {p4}")

    print(f"\nPlots saved to {output_dir}/")


if __name__ == "__main__":
    main()
