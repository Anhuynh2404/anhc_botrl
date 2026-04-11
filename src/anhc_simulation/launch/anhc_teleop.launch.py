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
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("anhc_simulation"))
    install_setup = (share.parents[2] / "setup.bash").resolve()
    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    ros_setup = f"/opt/ros/{ros_distro}/setup.bash"

    teleop_params = [
        {
            "use_sim_time": True,
            "speed": 0.18,
            "turn": 0.45,
            "repeat_rate": 15.0,
            "key_timeout": 0.4,
        }
    ]

    intro = LogInfo(
        msg=(
            "Teleop opens in a separate terminal window — click that window and use i/j/l/k "
            "(see on-screen help in that window). teleop_twist_keyboard needs a real TTY."
        )
    )

    xterm = shutil.which("xterm")
    if xterm:
        return LaunchDescription(
            [
                intro,
                Node(
                    package="teleop_twist_keyboard",
                    executable="teleop_twist_keyboard",
                    name="teleop_twist_keyboard",
                    output="screen",
                    prefix=f"{xterm} -fa Monospace -fs 13 -e ",
                    remappings=[("/cmd_vel", "/cmd_vel")],
                    parameters=teleop_params,
                ),
            ]
        )

    gnome = shutil.which("gnome-terminal")
    if gnome and install_setup.is_file():
        inner = (
            f"source {ros_setup} && source {install_setup} && "
            "ros2 run teleop_twist_keyboard teleop_twist_keyboard "
            "--ros-args -r /cmd_vel:=/cmd_vel "
            "-p use_sim_time:=true -p speed:=0.18 -p turn:=0.45 "
            "-p repeat_rate:=15.0 -p key_timeout:=0.4; exec bash"
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
                    "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args "
                    "-r /cmd_vel:=/cmd_vel -p use_sim_time:=true "
                    "-p speed:=0.18 -p turn:=0.45 -p repeat_rate:=15.0 -p key_timeout:=0.4"
                )
            ),
        ]
    )
