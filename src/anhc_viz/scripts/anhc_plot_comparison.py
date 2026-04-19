#!/usr/bin/env python3
"""Comparison plots from BENCHMARK_PLAN_v1 CSV (radar, bars, scatter).

Example:
  python3 src/anhc_viz/scripts/anhc_plot_comparison.py \\
    --csv ~/anhc_benchmark_results/raw/benchmark_latest.csv \\
    --out-dir ~/anhc_benchmark_results/plots/
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
# Headless / CI: writable matplotlib config dir
if not os.access(os.path.expanduser("~/.cache/matplotlib"), os.W_OK):
    _mpl_cache = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".mpl_cache")
    os.makedirs(_mpl_cache, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", _mpl_cache)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

_DEFAULT_DIR = os.path.expanduser("~/anhc_benchmark_results")


def _latest_csv(results_dir: str) -> str:
    candidates = sorted(
        glob.glob(os.path.join(results_dir, "raw", "benchmark_*.csv"))
        + glob.glob(os.path.join(results_dir, "benchmark_*.csv"))
    )
    return candidates[-1] if candidates else ""


def normalize_for_radar(
    values: np.ndarray,
    higher_is_better: bool = False,
    log_scale: bool = False,
) -> np.ndarray:
    """Map to [0,1] with larger = better (BENCHMARK_PLAN_v1 §5.1)."""
    arr = np.asarray(values, dtype=float)
    work = np.copy(arr)
    if log_scale:
        work = np.log10(np.maximum(work, 1e-9))
    finite = work[np.isfinite(work)]
    if finite.size == 0:
        return np.full_like(arr, 0.5)
    vmin, vmax = float(finite.min()), float(finite.max())
    if vmax <= vmin:
        out = np.full_like(arr, 0.5)
    else:
        out = (work - vmin) / (vmax - vmin)
        out = np.where(np.isfinite(work), out, 0.5)
    if not higher_is_better:
        out = 1.0 - out
    return np.clip(out, 0.0, 1.0)


def load_and_validate(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    required = [
        "algorithm",
        "scenario_id",
        "trial",
        "success",
        "L_planned_m",
        "Tc_ms",
        "Tg_s",
        "S_rad_per_m",
        "C_min_m",
    ]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(
            f"CSV missing columns {missing}. "
            "Use the v1 benchmark runner (anhc_benchmark_runner_node)."
        )
    return df


def compute_sr(df: pd.DataFrame) -> pd.DataFrame:
    return (
        df.groupby(["algorithm", "scenario_id"])["success"]
        .mean()
        .mul(100.0)
        .reset_index()
        .rename(columns={"success": "SR_pct"})
    )


def _algo_summary(df: pd.DataFrame) -> pd.DataFrame:
    """One row per algorithm: means used for radar."""
    rows = []
    for algo in sorted(df["algorithm"].unique()):
        g = df[df["algorithm"] == algo]
        sr = float(g["success"].mean() * 100.0) if len(g) else 0.0
        cmin = pd.to_numeric(g["C_min_m"], errors="coerce")
        cmin = cmin.where(cmin >= 0.0)
        rows.append(
            {
                "algorithm": algo,
                "L_planned_m": float(pd.to_numeric(g["L_planned_m"], errors="coerce").mean()),
                "Tc_ms": float(pd.to_numeric(g["Tc_ms"], errors="coerce").mean()),
                "Tg_s": float(pd.to_numeric(g["Tg_s"], errors="coerce").mean()),
                "S_rad_per_m": float(
                    pd.to_numeric(g["S_rad_per_m"], errors="coerce").mean()
                ),
                "C_min_m": float(cmin.mean()) if cmin.notna().any() else 0.0,
                "SR_pct": sr,
            }
        )
    return pd.DataFrame(rows)


def plot_radar(summary: pd.DataFrame, out_path: str) -> None:
    metrics = ["L_planned_m", "Tc_ms", "Tg_s", "S_rad_per_m", "SR_pct", "C_min_m"]
    labels = [
        "Path len\n(lower)",
        "Plan time\n(log, lower)",
        "Time to goal\n(lower)",
        "Smoothness\n(lower)",
        "Success\n(higher)",
        "Clearance\n(higher)",
    ]
    higher = [False, False, False, False, True, True]
    log_sc = [False, True, False, False, False, False]

    algos = summary["algorithm"].tolist()
    n = len(metrics)
    angles = np.linspace(0, 2 * np.pi, n, endpoint=False).tolist()
    angles += angles[:1]

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))
    cmap = plt.colormaps["tab10"]

    norm = np.zeros((len(algos), n))
    for j, m in enumerate(metrics):
        col = summary[m].values.astype(float)
        norm[:, j] = normalize_for_radar(
            col, higher_is_better=higher[j], log_scale=log_sc[j]
        )

    for i, algo in enumerate(algos):
        vals = norm[i].tolist() + [norm[i][0]]
        color = cmap(i % 10)
        ax.plot(angles, vals, "o-", linewidth=1.5, label=algo, color=color)
        ax.fill(angles, vals, alpha=0.08, color=color)

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(labels, size=9)
    ax.set_ylim(0, 1)
    ax.legend(loc="upper right", bbox_to_anchor=(1.25, 1.1))
    ax.set_title("Algorithm comparison (normalized)", y=1.08, fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _bar_metric_by_algo(
    df: pd.DataFrame,
    metric: str,
    ylabel: str,
    title: str,
    out_path: str,
    log_y: bool = False,
) -> None:
    algos = sorted(df["algorithm"].unique())
    means, stds = [], []
    for a in algos:
        col = pd.to_numeric(df.loc[df["algorithm"] == a, metric], errors="coerce")
        means.append(float(col.mean()))
        stds.append(float(col.std(ddof=0)) if len(col) > 1 else 0.0)
    x = np.arange(len(algos))
    fig, ax = plt.subplots(figsize=(max(8, len(algos) * 0.9), 4.5))
    ax.bar(
        x,
        means,
        yerr=stds,
        capsize=5,
        color=plt.cm.tab10(np.linspace(0, 0.9, len(algos))),
        edgecolor="white",
    )
    ax.set_xticks(x)
    ax.set_xticklabels(algos, rotation=25, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if log_y:
        ax.set_yscale("log")
    ax.yaxis.grid(True, linestyle="--", alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _bar_path_length_by_scenario(df: pd.DataFrame, out_path: str) -> None:
    scenarios = sorted(df["scenario_id"].unique())
    algos = sorted(df["algorithm"].unique())
    n_s, n_a = len(scenarios), len(algos)
    width = 0.8 / n_a
    x = np.arange(n_s)
    fig, ax = plt.subplots(figsize=(max(10, n_s * 1.2), 5))
    for i, a in enumerate(algos):
        means = []
        stds = []
        for s in scenarios:
            col = pd.to_numeric(
                df.loc[(df["algorithm"] == a) & (df["scenario_id"] == s), "L_planned_m"],
                errors="coerce",
            )
            means.append(float(col.mean()) if len(col) else 0.0)
            stds.append(float(col.std(ddof=0)) if len(col) > 1 else 0.0)
        pos = x + (i - n_a / 2 + 0.5) * width
        ax.bar(pos, means, width, yerr=stds, capsize=3, label=a)
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios, rotation=20, ha="right")
    ax.set_ylabel("L_planned (m)")
    ax.set_title("Path length by scenario")
    ax.legend(fontsize=8, ncol=2)
    ax.yaxis.grid(True, linestyle="--", alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _bar_success_by_scenario(df: pd.DataFrame, out_path: str) -> None:
    sr = compute_sr(df)
    scenarios = sorted(sr["scenario_id"].unique())
    algos = sorted(sr["algorithm"].unique())
    x = np.arange(len(scenarios))
    width = 0.8 / max(len(algos), 1)
    fig, ax = plt.subplots(figsize=(max(10, len(scenarios) * 1.0), 5))
    for i, a in enumerate(algos):
        heights = []
        for s in scenarios:
            sel = sr.loc[(sr["algorithm"] == a) & (sr["scenario_id"] == s), "SR_pct"]
            heights.append(float(sel.iloc[0]) if len(sel) else 0.0)
        pos = x + (i - len(algos) / 2 + 0.5) * width
        ax.bar(pos, heights, width, label=a)
    ax.set_xticks(x)
    ax.set_xticklabels(scenarios, rotation=20, ha="right")
    ax.set_ylabel("SR %")
    ax.set_ylim(0, 105)
    ax.set_title("Success rate by scenario")
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _scatter_tc_tg(df: pd.DataFrame, out_path: str) -> None:
    fig, ax = plt.subplots(figsize=(7, 5))
    for i, a in enumerate(sorted(df["algorithm"].unique())):
        g = df[df["algorithm"] == a]
        ax.scatter(
            pd.to_numeric(g["Tc_ms"], errors="coerce"),
            pd.to_numeric(g["Tg_s"], errors="coerce"),
            label=a,
            alpha=0.7,
            s=36,
        )
    ax.set_xlabel("Tc (ms)")
    ax.set_ylabel("Tg (s)")
    ax.set_title("Planning time vs time to goal")
    ax.legend()
    ax.grid(True, linestyle="--", alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="BENCHMARK_PLAN_v1 comparison plots")
    parser.add_argument("--csv", default="", help="Path to v1 benchmark CSV")
    parser.add_argument(
        "--out-dir",
        default=os.path.join(_DEFAULT_DIR, "plots"),
        help="Output directory for PNG files",
    )
    parser.add_argument(
        "--radar-out",
        default="radar_all_algorithms.png",
        help="Radar chart filename",
    )
    parser.add_argument(
        "--group-by",
        default="scenario",
        choices=["all", "scenario"],
        help="Bar grouping (reserved for future facets)",
    )
    args = parser.parse_args()
    _ = args.group_by

    csv_path = args.csv or _latest_csv(_DEFAULT_DIR)
    if not csv_path:
        print(f"No benchmark CSV under {_DEFAULT_DIR}", file=sys.stderr)
        sys.exit(1)

    df = load_and_validate(csv_path)
    out_dir = os.path.expanduser(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    summary = _algo_summary(df)
    plot_radar(summary, os.path.join(out_dir, args.radar_out))

    _bar_path_length_by_scenario(df, os.path.join(out_dir, "bar_path_length.png"))
    _bar_metric_by_algo(
        df,
        "Tc_ms",
        "Tc (ms)",
        "Mean planning time ± std",
        os.path.join(out_dir, "bar_planning_time.png"),
        log_y=True,
    )
    _bar_metric_by_algo(
        df,
        "Tg_s",
        "Tg (s)",
        "Mean time to goal ± std",
        os.path.join(out_dir, "bar_time_to_goal.png"),
    )
    _bar_metric_by_algo(
        df,
        "S_rad_per_m",
        "S (rad/m)",
        "Mean smoothness ± std",
        os.path.join(out_dir, "bar_smoothness.png"),
    )
    # Clearance: show C_min
    _bar_metric_by_algo(
        df,
        "C_min_m",
        "C_min (m)",
        "Mean minimum clearance ± std",
        os.path.join(out_dir, "bar_clearance.png"),
    )
    _bar_success_by_scenario(df, os.path.join(out_dir, "bar_success_rate.png"))
    _scatter_tc_tg(df, os.path.join(out_dir, "scatter_tc_vs_tg.png"))

    print(f"Plots written to {out_dir}/")


if __name__ == "__main__":
    main()
