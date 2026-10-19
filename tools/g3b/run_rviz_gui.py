#!/usr/bin/env python3
"""Server-only automated RViz GoalTool interaction and physical verification."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import time
import traceback
from pathlib import Path
from typing import Any
from uuid import uuid4

import httpx
import rclpy

from run_single_goal import (
    PYTHON,
    ROOT,
    GoalProbe,
    Process,
    angle_error,
    atomic_json,
    free_port,
    start_episode,
    utc_now,
    wait_graph,
    wait_navigation_healthy,
    yaw_from_quaternion,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def find_window(env: dict[str, str], timeout_s: float = 30.0) -> str:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["xdotool", "search", "--onlyvisible", "--name", "RViz"],
            env=env,
            text=True,
            capture_output=True,
            timeout=5.0,
        )
        values = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        if values:
            return values[-1]
        time.sleep(0.5)
    raise TimeoutError("RViz window did not appear")


def click_goal(
    env: dict[str, str],
    window_id: str,
    *,
    click_x: int,
    click_y: int,
    drag_x: int,
    drag_y: int,
    toolbar_x: int,
    toolbar_y: int,
) -> list[list[str]]:
    commands = [
        ["xdotool", "windowfocus", window_id],
        [
            "xdotool",
            "mousemove",
            "--window",
            window_id,
            str(toolbar_x),
            str(toolbar_y),
        ],
        ["xdotool", "click", "1"],
        [
            "xdotool",
            "mousemove",
            "--window",
            window_id,
            str(click_x),
            str(click_y),
        ],
        ["xdotool", "mousedown", "1"],
        [
            "xdotool",
            "mousemove",
            "--window",
            window_id,
            "--sync",
            str(drag_x),
            str(drag_y),
        ],
        ["xdotool", "mouseup", "1"],
    ]
    for command in commands:
        subprocess.run(
            command,
            env=env,
            check=True,
            text=True,
            capture_output=True,
            timeout=10.0,
        )
        time.sleep(0.15)
    return commands


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--map-root", type=Path, required=True)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--report-root", type=Path, required=True)
    parser.add_argument("--domain-id", type=int, default=111)
    parser.add_argument("--display-number", type=int, default=97)
    parser.add_argument("--click-fraction-x", type=float, default=0.66)
    parser.add_argument("--click-fraction-y", type=float, default=0.56)
    parser.add_argument("--drag-fraction-x", type=float, default=0.74)
    parser.add_argument("--toolbar-x", type=int, default=250)
    parser.add_argument("--toolbar-y", type=int, default=42)
    parser.add_argument("--timeout-wall", type=float, default=180.0)
    args = parser.parse_args()
    report_root = args.report_root.resolve()
    report_root.mkdir(parents=True, exist_ok=True)
    map_root = args.map_root.resolve()
    port = free_port()
    env = os.environ.copy()
    env.update(
        {
            "MUJOCO_GL": "egl",
            "PYOPENGL_PLATFORM": "egl",
            "ROS_DOMAIN_ID": str(args.domain_id),
            "RMW_IMPLEMENTATION": env.get("G3A_ROS_RMW", "rmw_fastrtps_cpp"),
            "PYTHONPATH": str(ROOT / "src")
            + (":" + env["PYTHONPATH"] if env.get("PYTHONPATH") else ""),
        }
    )
    os.environ.update(
        {
            "MUJOCO_GL": env["MUJOCO_GL"],
            "PYOPENGL_PLATFORM": env["PYOPENGL_PLATFORM"],
            "ROS_DOMAIN_ID": env["ROS_DOMAIN_ID"],
            "RMW_IMPLEMENTATION": env["RMW_IMPLEMENTATION"],
            "PYTHONPATH": env["PYTHONPATH"],
        }
    )
    display = f":{args.display_number}"
    gui_env = env.copy()
    gui_env["DISPLAY"] = display
    report: dict[str, Any] = {
        "schema_version": 1,
        "report_version": "g3b-rviz-gui-1",
        "execution_location": "server_only",
        "generated_wall_utc": utc_now(),
        "success": False,
        "display": display,
    }
    app = Process(
        [
            str(PYTHON),
            "-m",
            "mobilecar",
            "--config",
            str(args.config.resolve()),
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
        ],
        report_root / "mobilecar_server.log",
        env,
    )
    supervisor = slam = nav2 = xvfb = rviz = None
    probe = None
    rclpy.init()
    try:
        base_url = f"http://127.0.0.1:{port}"
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if not app.alive():
                raise RuntimeError("MuJoCo application exited")
            try:
                if httpx.get(f"{base_url}/api/v1/health", timeout=1.0).status_code == 200:
                    break
            except Exception:
                pass
            time.sleep(0.1)
        else:
            raise RuntimeError("MuJoCo application startup timeout")
        with httpx.Client(base_url=base_url, timeout=30.0) as client:
            episode = start_episode(client, "gui_rviz_goal")
            if not episode.get("accepted"):
                raise RuntimeError(f"Episode start rejected: {episode}")
            snapshot = client.get("/api/v1/snapshot").json()
            report.update(
                {
                    "run_id": snapshot["run_id"],
                    "episode_id": snapshot["episode_id"],
                    "map_id": args.map_id,
                }
            )
            supervisor = Process(
                [
                    "ros2",
                    "launch",
                    "mobilecar_goal_supervisor",
                    "g3b_supervisor.launch.py",
                    f"map_id:={args.map_id}",
                    f"map_manifest:={map_root / 'manifest.json'}",
                    f"runtime_config:={args.config.resolve()}",
                    "socket_path:=../../var/ipc/g3b_navigation.sock",
                ],
                report_root / "g3b_supervisor.log",
                env,
            )
            slam = Process(
                [
                    "ros2",
                    "launch",
                    "mobilecar_mapping_localization",
                    "localization.launch.py",
                    f"map_file_name:={map_root / 'posegraph'}",
                    "use_sim_time:=true",
                ],
                report_root / "slam_toolbox_localization.log",
                env,
            )
            nav2 = Process(
                [
                    "ros2",
                    "launch",
                    "mobilecar_nav_bringup",
                    "navigation_g3b.launch.py",
                    "use_sim_time:=true",
                ],
                report_root / "nav2.log",
                env,
            )
            topics, actions = wait_graph(
                env,
                topics={
                    "/clock",
                    "/scan",
                    "/odom",
                    "/map",
                    "/pose",
                    "/goal_pose",
                    "/mobilecar/navigation_system_health",
                },
                actions={"/navigate_to_pose", "/nav2/navigate_to_pose"},
                timeout_s=60.0,
            )
            report["topics"] = sorted(topics)
            report["actions"] = sorted(actions)
            probe = GoalProbe()
            if not wait_navigation_healthy(probe):
                raise RuntimeError("navigation did not become healthy")
            xvfb = Process(
                [
                    "Xvfb",
                    display,
                    "-screen",
                    "0",
                    "1400x900x24",
                    "-nolisten",
                    "tcp",
                ],
                report_root / "xvfb.log",
                env,
            )
            time.sleep(1.0)
            rviz_config = (
                ROOT
                / "ros2_ws/install/mobilecar_nav_bringup/share/mobilecar_nav_bringup/rviz/g3b_navigation.rviz"
            )
            rviz = Process(
                [
                    "rviz2",
                    "-d",
                    str(rviz_config),
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                ],
                report_root / "rviz.log",
                gui_env,
            )
            window_id = find_window(gui_env)
            report["window_id"] = window_id
            geometry_result = subprocess.run(
                ["xdotool", "getwindowgeometry", "--shell", window_id],
                env=gui_env,
                check=True,
                text=True,
                capture_output=True,
                timeout=10.0,
            )
            geometry = {
                key.lower(): int(value)
                for key, value in (
                    line.split("=", 1)
                    for line in geometry_result.stdout.splitlines()
                    if "=" in line
                )
            }
            report["window_geometry"] = geometry
            initial_screenshot = report_root / "rviz_goal_tool_ready.png"
            subprocess.run(
                ["import", "-window", window_id, str(initial_screenshot)],
                env=gui_env,
                check=True,
                text=True,
                capture_output=True,
                timeout=30.0,
            )
            report["initial_screenshot_path"] = str(initial_screenshot)
            report["initial_screenshot_sha256"] = sha256(initial_screenshot)
            click_x = int(geometry["width"] * args.click_fraction_x)
            click_y = int(geometry["height"] * args.click_fraction_y)
            drag_x = int(geometry["width"] * args.drag_fraction_x)
            report["gui_commands"] = click_goal(
                gui_env,
                window_id,
                click_x=click_x,
                click_y=click_y,
                drag_x=drag_x,
                drag_y=click_y,
                toolbar_x=args.toolbar_x,
                toolbar_y=args.toolbar_y,
            )
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                status = probe.goal_status
                terminal = status.get("last_terminal_goal")
                if status.get("requested_pose") is not None or terminal is not None:
                    break
                time.sleep(0.1)
            else:
                raise RuntimeError("RViz GoalTool did not submit the outer action")
            status = dict(probe.goal_status)
            goal = status.get("last_terminal_goal") or status
            requested_pose = goal.get("requested_pose")
            if requested_pose is None:
                raise RuntimeError("outer goal status omitted requested pose")
            report["goal_supervisor_initial_status"] = status
            report["requested_pose"] = requested_pose
            screenshot = report_root / "rviz_goal_interaction.png"
            subprocess.run(
                ["import", "-window", window_id, str(screenshot)],
                env=gui_env,
                check=True,
                text=True,
                capture_output=True,
                timeout=30.0,
            )
            deadline = time.monotonic() + args.timeout_wall
            terminal = None
            while time.monotonic() < deadline:
                terminal = probe.goal_status.get("last_terminal_goal")
                if terminal is not None and terminal.get("state") in {
                    "SUCCEEDED",
                    "ABORTED",
                    "REJECTED",
                    "CANCEL_REQUESTED",
                }:
                    break
                time.sleep(0.1)
            accepted = terminal is not None
            settle_started = time.monotonic()
            settled_samples = []
            while time.monotonic() - settle_started < 4.0:
                current = client.get("/api/v1/snapshot").json()
                velocity = current["base"]
                settled = (
                    math.hypot(velocity["vx_m_s"], velocity["vy_m_s"]) <= 0.01
                    and abs(velocity["wz_rad_s"]) <= 0.03
                )
                settled_samples.append(settled)
                if len(settled_samples) >= 6 and all(settled_samples[-6:]):
                    break
                time.sleep(0.1)
            final_snapshot = client.get("/api/v1/snapshot").json()
            final_yaw = yaw_from_quaternion(
                final_snapshot["base"]["orientation_world_sim_xyzw"]
            )
            goal_yaw = float(requested_pose["yaw_rad"])
            position_error = math.hypot(
                final_snapshot["base"]["position_world_sim_m"][0]
                - requested_pose["x_m"],
                final_snapshot["base"]["position_world_sim_m"][1]
                - requested_pose["y_m"],
            )
            settled_count = 0
            for value in reversed(settled_samples):
                if not value:
                    break
                settled_count += 1
            collisions = final_snapshot.get("contact_summary", {}).get(
                "unexpected", []
            )
            report.update(
                {
                    "relay_status": probe.goal_pose_relay_status,
                    "goal_supervisor_status": probe.goal_status,
                    "requested_pose": requested_pose,
                    "goal_accepted": accepted,
                    "terminal_goal": terminal,
                    "position_error_m": position_error,
                    "yaw_error_deg": math.degrees(
                        angle_error(final_yaw, goal_yaw)
                    ),
                    "settled_duration_s": settled_count * 0.1,
                    "unexpected_contacts": collisions,
                    "final_snapshot": final_snapshot,
                    "screenshot_path": str(screenshot),
                    "screenshot_bytes": screenshot.stat().st_size,
                    "screenshot_sha256": sha256(screenshot),
                }
            )
            report["gates"] = {
                "outer_goal_received_from_rviz": requested_pose is not None,
                "outer_action_accepted": accepted,
                "action_succeeded": (
                    terminal is not None
                    and terminal.get("state") == "SUCCEEDED"
                ),
                "physical_position": position_error <= 0.10,
                "physical_yaw": math.degrees(angle_error(final_yaw, goal_yaw))
                <= 10.0,
                "settled": settled_count * 0.1 >= 0.60,
                "screenshot_captured": screenshot.stat().st_size > 20_000,
                "no_unexpected_contacts": not collisions,
            }
            report["success"] = all(report["gates"].values())
            stop = client.post(
                "/api/v1/episodes/stop",
                json={"reason": "G3B_RVIZ_GUI_COMPLETE"},
            )
            stop.raise_for_status()
            report["stop_response"] = stop.json()
            subprocess.run(
                ["xdotool", "key", "--window", window_id, "ctrl+q"],
                env=gui_env,
                text=True,
                capture_output=True,
                timeout=10.0,
            )
            time.sleep(1.0)
    except Exception:
        report["error"] = traceback.format_exc()
    finally:
        if probe is not None:
            probe.close()
        report["rviz_exit_code"] = rviz.close() if rviz is not None else None
        report["xvfb_exit_code"] = xvfb.close() if xvfb is not None else None
        report["nav2_exit_code"] = nav2.close() if nav2 is not None else None
        report["slam_exit_code"] = slam.close() if slam is not None else None
        report["supervisor_exit_code"] = (
            supervisor.close() if supervisor is not None else None
        )
        report["application_exit_code"] = app.close()
        if rclpy.ok():
            rclpy.shutdown()
        atomic_json(report_root / "g3b_rviz_gui_report.json", report)
    print(
        json.dumps(
            {
                "success": report.get("success", False),
                "report": str(report_root / "g3b_rviz_gui_report.json"),
            },
            indent=2,
        )
    )
    return 0 if report.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
