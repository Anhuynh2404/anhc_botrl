"""Results analyzer node for the anhc autonomous vehicle benchmark.

Reads the most recent CSV from ``<output_dir>/raw/`` (default: workspace
``bench_results``, same as ``anhc_benchmark.launch.py``), publishes
``/benchmark/summary``, and writes ``<output_dir>/report.md``.
"""

import glob
import json
import os
import time

import pandas as pd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from anhc_benchmark.bench_paths import default_benchmark_output_dir

_NUMERIC_METRICS = [
    "planning_time_ms",
    "path_length_m",
    "path_smoothness",
    "nodes_expanded",
    "execution_time_s",
    "clearance_avg_m",
    "cpu_usage_percent",
    "memory_mb",
]

_DEFAULT_OUTPUT_DIR = default_benchmark_output_dir()


class AnhcResultsAnalyzerNode(Node):
    """Reads benchmark CSV results, publishes aggregated stats, saves report.

    Topics
    ------
    Publications:
        /benchmark/summary — std_msgs/String (JSON, 0.5 Hz)

    The JSON payload contains a top-level "algorithms" key mapping each
    algorithm name to a dict of { metric: {mean, std} } entries, plus
    a top-level "meta" dict with file path and generation timestamp.
    """

    def __init__(self) -> None:
        super().__init__("anhc_results_analyzer")

        self.declare_parameter("output_dir", _DEFAULT_OUTPUT_DIR)

        self._summary_payload: str = json.dumps({"algorithms": {}, "meta": {}})
        self._last_csv: str = ""

        self._pub = self.create_publisher(String, "/benchmark/summary", 10)

        # Attempt analysis at startup, then re-check every 30 s for new files.
        self.create_timer(2.0, self._startup_analyze)
        self.create_timer(30.0, self._periodic_analyze)
        self.create_timer(2.0, self._publish_summary)  # 0.5 Hz

        self.get_logger().info("[anhc_results_analyzer] started")

    # ──────────────────────────────────────────────────────────────────────────
    # Timers
    # ──────────────────────────────────────────────────────────────────────────

    def _startup_analyze(self) -> None:
        """One-shot startup analysis; cancelled after first successful load."""
        if self._try_analyze():
            # Cancel this one-shot timer by making it a no-op from now on.
            self._startup_analyze = lambda: None  # type: ignore[assignment]

    def _periodic_analyze(self) -> None:
        self._try_analyze()

    def _publish_summary(self) -> None:
        self._pub.publish(String(data=self._summary_payload))

    # ──────────────────────────────────────────────────────────────────────────
    # Core logic
    # ──────────────────────────────────────────────────────────────────────────

    def _try_analyze(self) -> bool:
        """Find the newest CSV, analyse it, update payload, write report."""
        csv_path = self._latest_csv()
        if not csv_path:
            self.get_logger().debug(
                "[anhc_results_analyzer] no CSV found in "
                f"{self._output_dir} — waiting"
            )
            return False

        if csv_path == self._last_csv:
            return True  # already analysed this file

        try:
            df = pd.read_csv(csv_path)
            df = self._normalize_schema(df)
        except Exception as exc:
            self.get_logger().error(
                f"[anhc_results_analyzer] cannot read {csv_path}: {exc}"
            )
            return False

        if df.empty:
            self.get_logger().warn(
                f"[anhc_results_analyzer] {csv_path} is empty — skipping"
            )
            return False

        summary = self._compute_summary(df)
        meta = {
            "csv_file": csv_path,
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "total_trials": len(df),
        }
        payload = {"algorithms": summary, "meta": meta}
        self._summary_payload = json.dumps(payload)
        self._last_csv = csv_path

        self._write_report(df, summary, meta)
        self.get_logger().info(
            f"[anhc_results_analyzer] analysed {csv_path} "
            f"({len(df)} trials, {len(summary)} algorithm(s))"
        )
        return True

    @property
    def _output_dir(self) -> str:
        return os.path.expanduser(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )

    def _latest_csv(self) -> str:
        patterns = [
            os.path.join(self._output_dir, "raw", "benchmark_*.csv"),
            os.path.join(self._output_dir, "benchmark_*.csv"),
        ]
        files: list[str] = []
        for pattern in patterns:
            files.extend(glob.glob(pattern))
        files = sorted(files)
        return files[-1] if files else ""

    @staticmethod
    def _normalize_schema(df: pd.DataFrame) -> pd.DataFrame:
        """Legacy runner CSV + BENCHMARK_PLAN_v1 CSV → unified columns for reports."""
        df = df.copy()

        # BENCHMARK_PLAN_v1 raw CSV
        if "L_planned_m" in df.columns and "algorithm_name" not in df.columns:
            df["algorithm_name"] = df["algorithm"]
            df["path_length_m"] = pd.to_numeric(
                df["L_planned_m"], errors="coerce"
            ).fillna(0.0)
            df["planning_time_ms"] = pd.to_numeric(
                df["Tc_ms"], errors="coerce"
            ).fillna(0.0)
            df["execution_time_s"] = pd.to_numeric(
                df["Tg_s"], errors="coerce"
            ).fillna(0.0)
            # Legacy smoothness was Σ|Δθ| (rad); v1 stores S_rad_per_m
            Lp = pd.to_numeric(df["L_planned_m"], errors="coerce").fillna(0.0)
            Srm = pd.to_numeric(df["S_rad_per_m"], errors="coerce").fillna(0.0)
            df["path_smoothness"] = Srm * Lp
            df["clearance_avg_m"] = pd.to_numeric(
                df["C_avg_m"], errors="coerce"
            ).fillna(-1.0)
            fr = df["failure_reason"].fillna("").astype(str)
            df["planning_success"] = (Lp > 0.0) & (~fr.isin(["no_path", "planning_timeout"]))
            df["collision"] = fr == "collision"
            df["success"] = pd.to_numeric(df["success"], errors="coerce").fillna(0).astype(
                bool
            )
            df["_scenario_group"] = df["scenario_id"].astype(str)
        else:
            df["_scenario_group"] = (
                df["map_name"].astype(str) if "map_name" in df.columns else "default"
            )

        if "planning_success" not in df.columns:
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

    @staticmethod
    def _compute_summary(df: pd.DataFrame) -> dict:
        """Return {algorithm_name: {metric: {mean, std}}} aggregated stats."""
        summary: dict = {}
        for algo, grp in df.groupby("algorithm_name"):
            stats: dict = {}
            for metric in _NUMERIC_METRICS:
                if metric not in grp.columns:
                    continue
                col = pd.to_numeric(grp[metric], errors="coerce").dropna()
                stats[metric] = {
                    "mean": round(float(col.mean()), 4) if len(col) else None,
                    "std": round(float(col.std()), 4) if len(col) > 1 else 0.0,
                    "n": len(col),
                }
            # Boolean metrics
            for bm in ("planning_success", "success", "collision"):
                if bm in grp.columns:
                    stats[bm] = {
                        "rate": round(
                            float(grp[bm].astype(bool).mean()), 4
                        ),
                        "n": len(grp),
                    }
            summary[str(algo)] = stats
        return summary

    def _write_report(self, df: pd.DataFrame, summary: dict, meta: dict) -> None:
        """Write a human-readable Markdown comparison report."""
        report_path = os.path.join(self._output_dir, "report.md")
        lines: list[str] = []

        lines.append("# anhc Benchmark Report\n")
        lines.append(f"**Generated:** {meta.get('generated_at', '')}  \n")
        lines.append(f"**Source CSV:** `{meta.get('csv_file', '')}`  \n")
        lines.append(f"**Total trials:** {meta.get('total_trials', 0)}  \n\n")

        lines.append("## Algorithm Comparison\n")

        # Build comparison table for numeric metrics
        table_metrics = [
            ("planning_time_ms", "Planning Time (ms)"),
            ("path_length_m", "Path Length (m)"),
            ("path_smoothness", "Path Smoothness (rad)"),
            ("nodes_expanded", "Nodes Expanded"),
            ("execution_time_s", "Execution Time (s)"),
            ("clearance_avg_m", "Avg Clearance (m)"),
            ("cpu_usage_percent", "CPU Usage (%)"),
            ("memory_mb", "Memory (MB)"),
        ]

        algos = sorted(summary.keys())
        header = "| Metric | " + " | ".join(algos) + " |"
        sep = "|--------|" + "|".join(["--------"] * len(algos)) + "|"
        lines.append(header)
        lines.append(sep)

        for metric_key, metric_label in table_metrics:
            row_parts = [f"**{metric_label}**"]
            for algo in algos:
                s = summary.get(algo, {}).get(metric_key)
                if s is None:
                    row_parts.append("N/A")
                else:
                    mean = s.get("mean")
                    std = s.get("std", 0.0)
                    if mean is None:
                        row_parts.append("N/A")
                    else:
                        row_parts.append(f"{mean:.3f} ± {std:.3f}")
            lines.append("| " + " | ".join(row_parts) + " |")

        # Boolean metrics: planning_success, success, collision
        for bm, label in [
            ("planning_success", "Planning Success Rate"),
            ("success", "Goal-Reached Rate"),
            ("collision", "Collision Rate"),
        ]:
            row_parts = [f"**{label}**"]
            for algo in algos:
                s = summary.get(algo, {}).get(bm)
                if s is None:
                    row_parts.append("N/A")
                else:
                    row_parts.append(f"{s.get('rate', 0.0):.2%}")
            lines.append("| " + " | ".join(row_parts) + " |")

        lines.append("\n")

        # Per-scenario breakdown
        lines.append("## Per-Scenario Summary\n")
        group_col = "_scenario_group" if "_scenario_group" in df.columns else "map_name"
        for scenario_name, sgrp in df.groupby(group_col):
            lines.append(f"### {scenario_name}\n")
            lines.append(
                f"Trials: {len(sgrp)} | "
                f"Algorithms: {', '.join(sorted(sgrp['algorithm_name'].unique()))}\n\n"
            )
            sc_summary = self._compute_summary(sgrp)
            sc_algos = sorted(sc_summary.keys())
            sc_header = "| Metric | " + " | ".join(sc_algos) + " |"
            sc_sep = "|--------|" + "|".join(["--------"] * len(sc_algos)) + "|"
            lines.append(sc_header)
            lines.append(sc_sep)
            for metric_key, metric_label in table_metrics:
                row_parts = [f"**{metric_label}**"]
                for algo in sc_algos:
                    s = sc_summary.get(algo, {}).get(metric_key)
                    if s is None or s.get("mean") is None:
                        row_parts.append("N/A")
                    else:
                        row_parts.append(
                            f"{s['mean']:.3f} ± {s.get('std', 0.0):.3f}"
                        )
                lines.append("| " + " | ".join(row_parts) + " |")
            # Boolean rates per scenario
            for bm, label in [
                ("planning_success", "**Planning Success Rate**"),
                ("success", "**Goal-Reached Rate**"),
                ("collision", "**Collision Rate**"),
            ]:
                row_parts = [label]
                for algo in sc_algos:
                    s = sc_summary.get(algo, {}).get(bm)
                    row_parts.append(
                        f"{s.get('rate', 0.0):.2%}" if s else "N/A"
                    )
                lines.append("| " + " | ".join(row_parts) + " |")
            lines.append("\n")

        # Raw data sample
        lines.append("## Raw Data Sample (first 10 rows)\n")
        lines.append("```\n")
        lines.append(df.head(10).to_string(index=False))
        lines.append("\n```\n")

        os.makedirs(self._output_dir, exist_ok=True)
        with open(report_path, "w") as fh:
            fh.write("\n".join(lines))

        self.get_logger().info(
            f"[anhc_results_analyzer] report saved to {report_path}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnhcResultsAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
