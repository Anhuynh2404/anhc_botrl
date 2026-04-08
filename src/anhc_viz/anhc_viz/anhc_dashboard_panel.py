"""Live terminal dashboard for the anhc autonomous vehicle.

Subscribes to /planning/stats, /benchmark/live, and /benchmark/summary;
renders a colour-coded status panel using the ``rich`` library every 2 s.

Run standalone:
    ros2 run anhc_viz anhc_dashboard_panel
"""

import json
import time

import rclpy
from rclpy.node import Node
from rich.columns import Columns
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from std_msgs.msg import String


class AnhcDashboardPanel(Node):
    """Terminal dashboard node.

    Topics
    ------
    Subscriptions:
        /planning/stats    — std_msgs/String (JSON)
        /benchmark/live    — std_msgs/String (JSON)
        /benchmark/summary — std_msgs/String (JSON)
    """

    def __init__(self) -> None:
        super().__init__("anhc_dashboard_panel")

        self._stats: dict = {}
        self._live: dict = {}
        self._summary: dict = {}

        self.create_subscription(String, "/planning/stats", self._cb_stats, 10)
        self.create_subscription(String, "/benchmark/live", self._cb_live, 10)
        self.create_subscription(String, "/benchmark/summary", self._cb_summary, 10)

        self.create_timer(2.0, self._render)

        self._console = Console()
        self._start_time = time.time()
        self.get_logger().info("[anhc_dashboard] started — updating every 2 s")

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _cb_stats(self, msg: String) -> None:
        try:
            self._stats = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_live(self, msg: String) -> None:
        try:
            self._live = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_summary(self, msg: String) -> None:
        try:
            self._summary = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ──────────────────────────────────────────────────────────────────────────
    # Render
    # ──────────────────────────────────────────────────────────────────────────

    def _render(self) -> None:
        uptime = time.time() - self._start_time
        self._console.clear()

        # ── header ────────────────────────────────────────────────────────────
        header = Text(
            f"  anhc_botrl Dashboard  —  uptime {uptime:.0f}s  —  "
            f"{time.strftime('%H:%M:%S')}  ",
            style="bold white on dark_blue",
            justify="center",
        )
        self._console.print(header)
        self._console.print()

        # ── planner status ────────────────────────────────────────────────────
        algo = self._stats.get("algorithm", "—")
        plan_ms = self._stats.get("planning_time_ms", 0.0)
        path_m = self._stats.get("path_length_m", 0.0)
        nodes = self._stats.get("nodes_expanded", 0)
        status = self._stats.get("status", "—")

        planner_table = Table(title="Planner (last /planning/stats)", expand=True)
        planner_table.add_column("Field", style="cyan", no_wrap=True)
        planner_table.add_column("Value", style="bold green")
        planner_table.add_row("Algorithm", str(algo))
        planner_table.add_row("Status", str(status))
        planner_table.add_row("Planning time", f"{plan_ms:.2f} ms")
        planner_table.add_row("Path length", f"{path_m:.3f} m")
        planner_table.add_row("Nodes expanded", str(nodes))

        # ── live metrics ──────────────────────────────────────────────────────
        deviation = self._live.get("path_deviation", 0.0)
        speed = self._live.get("current_speed", 0.0)
        eta = self._live.get("eta_s", -1.0)
        eta_str = f"{eta:.1f} s" if eta >= 0 else "N/A"

        live_table = Table(title="Live (/benchmark/live)", expand=True)
        live_table.add_column("Field", style="cyan", no_wrap=True)
        live_table.add_column("Value", style="bold yellow")
        live_table.add_row("Speed", f"{speed:.3f} m/s")
        live_table.add_row("Path deviation", f"{deviation:.3f} m")
        live_table.add_row("ETA", eta_str)
        live_table.add_row(
            "Live algo",
            str(self._live.get("algorithm", "—")),
        )

        self._console.print(Columns([planner_table, live_table]))
        self._console.print()

        # ── benchmark summary ─────────────────────────────────────────────────
        algos = self._summary.get("algorithms", {})
        if algos:
            summary_table = Table(
                title="Benchmark Summary (/benchmark/summary)", expand=True
            )
            summary_table.add_column("Algorithm", style="cyan")
            summary_table.add_column("Plan Time (ms)", justify="right")
            summary_table.add_column("Path Length (m)", justify="right")
            summary_table.add_column("Nodes Exp.", justify="right")
            summary_table.add_column("Success %", justify="right")

            for a_name, a_stats in sorted(algos.items()):
                pt = a_stats.get("planning_time_ms", {})
                pl = a_stats.get("path_length_m", {})
                ne = a_stats.get("nodes_expanded", {})
                sc = a_stats.get("success", {})

                def _fmt(d: dict, key: str = "mean") -> str:
                    v = d.get(key)
                    return f"{v:.2f}" if v is not None else "—"

                summary_table.add_row(
                    a_name,
                    f"{_fmt(pt)} ± {_fmt(pt, 'std')}",
                    f"{_fmt(pl)} ± {_fmt(pl, 'std')}",
                    f"{_fmt(ne)} ± {_fmt(ne, 'std')}",
                    f"{sc.get('rate', 0.0) * 100:.0f}%" if sc else "—",
                )

            self._console.print(
                Panel(summary_table, border_style="dim white")
            )
        else:
            self._console.print(
                Panel(
                    "[dim]No benchmark summary yet — run the benchmark to populate.[/dim]",
                    title="Benchmark Summary",
                    border_style="dim white",
                )
            )

        self._console.print()
        self._console.print(
            "[dim]Press Ctrl+C to exit.[/dim]", justify="center"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnhcDashboardPanel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
