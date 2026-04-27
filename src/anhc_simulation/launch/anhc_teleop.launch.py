"""Teleoperation launch for manual mapping drive.

``teleop_twist_keyboard`` calls ``termios.tcgetattr(sys.stdin)`` and therefore
requires a real interactive TTY. A node started by ``ros2 launch`` typically
does not have stdin attached as a TTY, which causes:

    termios.error: (25, 'Inappropriate ioctl for device')

We fix this by wrapping the node in ``xterm -e`` when available, or spawning
``gnome-terminal`` with a ``bash -c`` one-liner that sources the workspace and
runs ``ros2 run teleop_twist_keyboard``.
"""

from __future__ import annotations

import os
import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("anhc_simulation"))
    install_setup = (share.parents[2] / "setup.bash").resolve()
    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    ros_setup = f"/opt/ros/{ros_distro}/setup.bash"

    # Defaults (ros2 run -p); user can still w/x/e/c in teleop.
    # Conservative defaults for SLAM mapping stability on large indoor meshes.
    _spd, _turn, _rep, _key = 0.16, 0.32, 15.0, 0.4
    _ros_args = (
        "--ros-args -r /cmd_vel:=/cmd_vel "
        f"-p use_sim_time:=true -p speed:={_spd} -p turn:={_turn} "
        f"-p repeat_rate:={_rep} -p key_timeout:={_key}"
    )

    intro = LogInfo(
        msg=(
            "Teleop opens in a separate terminal window — click that window and use i/j/l/k "
            "(see on-screen help in that window). teleop_twist_keyboard needs a real TTY. "
            "Start Gazebo first (e.g. anhc_sim.launch.py or anhc_sim_mapping.launch.py); "
            "commands go /cmd_vel → anhc_cmd_vel_idle_gate → /cmd_vel_gz → bridge."
        )
    )

    xterm = shutil.which("xterm")
    if xterm:
        # Do not use Node(prefix=xterm -e <binary>): mirror gnome path with bash -lc + ros2 run.
        if install_setup.is_file():
            inner_x = (
                f"source {ros_setup} && source {install_setup} && "
                f"ros2 run teleop_twist_keyboard teleop_twist_keyboard {_ros_args}; exec bash"
            )
        else:
            inner_x = (
                f"source {ros_setup} && "
                f"ros2 run teleop_twist_keyboard teleop_twist_keyboard {_ros_args}; exec bash"
            )
        return LaunchDescription(
            [
                intro,
                ExecuteProcess(
                    cmd=[
                        xterm,
                        "-fa",
                        "Monospace",
                        "-fs",
                        "13",
                        "-title",
                        "anhc_teleop",
                        "-e",
                        "bash",
                        "-lc",
                        inner_x,
                    ],
                    output="screen",
                ),
            ]
        )

    gnome = shutil.which("gnome-terminal")
    if gnome and install_setup.is_file():
        inner = (
            f"source {ros_setup} && source {install_setup} && "
            f"ros2 run teleop_twist_keyboard teleop_twist_keyboard {_ros_args}; exec bash"
        )
        return LaunchDescription(
            [
                intro,
                ExecuteProcess(
                    cmd=[gnome, "--title=anhc_teleop", "--", "bash", "-c", inner],
                    output="screen",
                ),
            ]
        )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Cannot start teleop from launch: no xterm or gnome-terminal found, "
                    "or workspace install/setup.bash missing.\n"
                    "Option A: sudo apt install xterm\n"
                    "Option B: run in a normal terminal (after sourcing install/setup.bash):\n"
                    "  ros2 run teleop_twist_keyboard teleop_twist_keyboard "
                    f"{_ros_args}"
                )
            ),
        ]
    )
