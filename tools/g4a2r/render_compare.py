#!/usr/bin/env python3
"""G4A-2R Phase 1: same-state comparison of MuJoCo native and Three.js views.

Server-only. For a set of frozen arm states this tool:

  1. checks the raw-vs-compiled mesh coordinate convention (raw STL bounds *
     mesh_scale must equal the compiled MuJoCo mesh vertex bounds);
  2. records MuJoCo forward kinematics (body world pos/quat) and an offscreen
     render from a fixed free camera;
  3. drives the real index page in headless Chromium with the real scene
     manifest (built from the same compiled model) and a frozen snapshot, then
     reads back the Three.js link world poses (converted into MuJoCo Z-up
     space) and screenshots the WebGL canvas from the matched camera;
  4. reports per-link position/orientation errors and saves the image pair.

No physics stepping, no commands, no Episodes: pure geometry/display evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np
import yaml
from playwright.sync_api import sync_playwright

from mobilecar.sensors.scene import build_scene_manifest


REPO = Path(__file__).resolve().parents[2]


def _xyzw(quat_wxyz) -> list[float]:
    w, x, y, z = (float(v) for v in quat_wxyz)
    return [x, y, z, w]


def _quat_angle_deg(a: list[float], b: list[float]) -> float:
    """Angle between two xyzw quaternions in degrees."""
    va = np.asarray(a, dtype=float)
    vb = np.asarray(b, dtype=float)
    na, nb = np.linalg.norm(va), np.linalg.norm(vb)
    if na == 0 or nb == 0:
        return 180.0
    va, vb = va / na, vb / nb
    dot = abs(float(np.dot(va, vb)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def _quat_to_matrix(quat_wxyz) -> np.ndarray:
    """Rotation matrix for a wxyz quaternion (MuJoCo convention)."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _stl_bounds(path: Path) -> tuple[list[float], list[float]]:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    offset = 84
    for index in range(count):
        values = struct.unpack_from("<12f", data, offset + index * 50)
        for vertex in range(3):
            for axis in range(3):
                value = values[3 + vertex * 3 + axis]
                lo[axis] = min(lo[axis], value)
                hi[axis] = max(hi[axis], value)
    return lo, hi


def _load_backend_config() -> dict:
    g4a1 = yaml.safe_load((REPO / "configs/g4a1_server.yaml").read_text(encoding="utf-8"))
    return g4a1["backend"]


class DiagnosticState:
    """Mutable holder shared with the intercepted snapshot endpoint."""

    def __init__(self) -> None:
        self.snapshot: dict = {}


def build_frozen_snapshot(state: dict, model: mujoco.MjModel, data: mujoco.MjData) -> dict:
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
    model_positions = [
        float(data.qpos[model.jnt_qposadr[model.joint(name).id]]) for name in joint_names
    ]
    joint_map: dict[str, float] = {}
    for joint_id in range(model.njnt):
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
            continue
        name = model.joint(joint_id).name
        if name:
            joint_map[name] = float(data.qpos[model.jnt_qposadr[joint_id]])
    base_pos = [float(v) for v in data.qpos[model.jnt_qposadr[model.joint("base_free").id] :][:3]]
    base_quat_xyzw = _xyzw(
        data.qpos[model.jnt_qposadr[model.joint("base_free").id] + 3 :][:4]
    )
    tcp = np.zeros(3)
    mujoco.mj_kinematics(model, data)
    tcp[:] = data.site_xpos[model.site("grasp_tcp").id]
    return {
        "contract_version": "mobilecar-1.0",
        "run_id": "diag_run",
        "backend_generation": 1,
        "episode_id": "diag_episode",
        "episode_started_backend_monotonic_s": 0.0,
        "backend_monotonic_s": 0.0,
        "sim_time_s": float(data.time),
        "feedback_time_s": 0.0,
        "wall_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "clock_id": "backend_monotonic",
        "control_tick_id": "diag_tick",
        "task_phase": "OBSERVE",
        "base": {
            "frame_id": "base_link",
            "position_world_sim_m": base_pos,
            "orientation_world_sim_xyzw": base_quat_xyzw,
            "vx_m_s": 0.0,
            "vy_m_s": 0.0,
            "wz_rad_s": 0.0,
            "stopped": True,
            "stop_latched": False,
        },
        "arm": {
            "user_joint_positions_rad": model_positions,
            "user_joint_targets_rad": model_positions,
            "model_joint_positions_rad": model_positions,
            "model_joint_targets_rad": model_positions,
            "joint_velocities_rad_s": [0.0] * 5,
            "joint_torques_nm": [0.0] * 5,
            "joint_margin_rad": [2.09] * 5,
            "model_limits_rad": [[-2.09, 2.09]] * 5,
            "coordinates_status": "ok",
        },
        "gripper": {
            "width_m": 0.025,
            "target_width_m": 0.025,
            "model_drive_position_rad": float(
                data.qpos[model.jnt_qposadr[model.joint("gripper_base_joint").id]]
            ),
            "model_drive_target_rad": float(
                data.qpos[model.jnt_qposadr[model.joint("gripper_base_joint").id]]
            ),
        },
        "tcp": {
            "frame_id": "world_sim",
            "position_m": [float(v) for v in tcp],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "sensors": [],
        "active_session_id": None,
        "safety_latched": False,
        "recording": {"status": "IDLE"},
        "contact_summary": {"unexpected_contact": False},
        "diagnostics": {"mode": "GT_DEBUG"},
        "model_joint_positions_rad": joint_map,
    }


def mesh_convention_report(model: mujoco.MjModel, model_dir: Path) -> list[dict]:
    assets_root = ET.parse(model_dir / "assets.xml").getroot()
    referenced: set[int] = set()
    for geom_id in range(model.ngeom):
        if int(model.geom_type[geom_id]) == int(mujoco.mjtGeom.mjGEOM_MESH):
            referenced.add(int(model.geom_dataid[geom_id]))
    entries = []
    for node in assets_root.findall(".//mesh"):
        name = node.get("name")
        file_name = node.get("file")
        if not (name and file_name):
            continue
        mesh_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, name)
        if mesh_id < 0:
            entries.append({"mesh": name, "error": "not found in compiled model"})
            continue
        if mesh_id not in referenced:
            # Kept by the compiler but rendered nowhere; not a display concern.
            entries.append(
                {
                    "mesh": name,
                    "unreferenced": True,
                    "convention_ok": True,
                    "note": "no geom references this mesh; neither MuJoCo nor the browser renders it",
                }
            )
            continue
        scale = [float(v) for v in model.mesh_scale[mesh_id]]
        vert_adr = int(model.mesh_vertadr[mesh_id])
        vert_num = int(model.mesh_vertnum[mesh_id])
        compiled = model.mesh_vert[vert_adr : vert_adr + vert_num]
        # MuJoCo compiles mesh vertices into the mesh inertial frame and stores
        # the transform back to the authored frame in mesh_pos/mesh_quat. The
        # browser loads the raw STL (authored frame), so compare in the
        # authored frame: raw = mesh_quat * compiled * scale + mesh_pos.
        mesh_pos = np.asarray(model.mesh_pos[mesh_id], dtype=float)
        mesh_quat = np.asarray(model.mesh_quat[mesh_id], dtype=float)  # wxyz
        rot = _quat_to_matrix(mesh_quat)
        authored = compiled @ rot.T * np.asarray(scale, dtype=float) + mesh_pos
        raw_lo, raw_hi = _stl_bounds((model_dir / file_name).resolve())
        expected_lo = [raw_lo[i] * scale[i] for i in range(3)]
        expected_hi = [raw_hi[i] * scale[i] for i in range(3)]
        compiled_lo = [float(authored[:, i].min()) for i in range(3)]
        compiled_hi = [float(authored[:, i].max()) for i in range(3)]
        max_diff = max(
            abs(compiled_lo[i] - expected_lo[i]) for i in range(3)
        ) + max(abs(compiled_hi[i] - expected_hi[i]) for i in range(3))
        entries.append(
            {
                "mesh": name,
                "file": file_name,
                "mesh_scale": scale,
                "raw_bounds_m": [raw_lo, raw_hi],
                "expected_compiled_bounds_m": [expected_lo, expected_hi],
                "compiled_bounds_m": [compiled_lo, compiled_hi],
                "bounds_max_abs_diff_m": float(max_diff),
                "convention_ok": max_diff < 1e-6,
            }
        )
    return entries


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--static-port", type=int, default=0)
    parser.add_argument("--position-tol-mm", type=float, default=2.0)
    parser.add_argument("--orientation-tol-deg", type=float, default=1.0)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    backend_config = _load_backend_config()
    model_path = (REPO / backend_config["model_path"]).resolve()
    model_dir = model_path.parent
    arm_spec = json.loads((model_dir / "arm_spec.json").read_text(encoding="utf-8"))
    prepared_base = backend_config["g4a1"]["prepared_base_qpos"]

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    report: dict = {
        "test": "g4a2r_render_compare_v1",
        "execution_location": "server_only",
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "states": {},
    }

    # --- 1. raw vs compiled mesh convention -----------------------------
    mesh_entries = mesh_convention_report(model, model_dir)
    report["mesh_convention"] = {
        "entries": mesh_entries,
        "all_ok": all(entry.get("convention_ok") for entry in mesh_entries),
    }

    # --- test states ------------------------------------------------------
    initial = dict(arm_spec["initial_positions"])
    states = [
        {"name": "initial", "joints": dict(initial)},
        {
            "name": "reach_forward",
            "joints": {**initial, "joint1": 0.35, "joint2": 0.55, "joint3": -0.45, "joint4": -1.15},
        },
        {
            "name": "tuck_low",
            "joints": {**initial, "joint1": -0.30, "joint2": -0.65, "joint3": 0.50, "joint4": -1.45, "joint5": 0.5},
        },
        {
            # Every jointed link family at non-zero values, mimic-consistent:
            # arm, gripper drive, swing fingers, slide jaws, wheels and rollers
            # (rollers also exercise hinge animation on rotated references).
            "name": "full_joints",
            "joints": {
                **initial,
                "joint1": 0.25,
                "joint2": 0.45,
                "joint3": -0.35,
                "joint4": -1.05,
                "joint5": 0.4,
                "gripper_base_joint": -0.6,
                "gripper_left_join": 0.9,
                "gripper_right_joint": 0.9,
                "left_jaw_joint": 0.0069597,
                "right_jaw_joint": -0.0069597,
                "front_left_joint": 1.1,
                "rear_left_joint": -0.7,
                "front_right_joint": 0.9,
                "rear_right_joint": -1.3,
                "front_left_roller_0_joint": 0.3,
                "front_right_roller_5_joint": -0.45,
                "rear_left_roller_2_joint": 0.6,
                "rear_right_roller_9_joint": 0.2,
            },
        },
    ]

    # --- 2. MuJoCo FK + offscreen render ---------------------------------
    az, el, dist = math.radians(128.0), math.radians(18.0), 1.25
    lookat = np.array([0.06, -0.02, 0.28])
    cam_dir = np.array(
        [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]
    )
    cam_pos = lookat + dist * cam_dir
    width, height, fovy = 640, 480, 45.0

    def project_mujoco(point: np.ndarray, fov_deg: float) -> tuple[float, float]:
        """Pixel coordinates of a Z-up world point in the free-camera image."""
        view = lookat - cam_pos
        depth_axis = view / np.linalg.norm(view)
        up = np.array([0.0, 0.0, 1.0])
        right = np.cross(depth_axis, up)
        right = right / np.linalg.norm(right)
        cam_up = np.cross(right, depth_axis)
        v = np.asarray(point, dtype=float) - cam_pos
        z = float(np.dot(v, depth_axis))
        tanf = math.tan(math.radians(fov_deg) / 2.0)
        x_ndc = float(np.dot(v, right)) / (z * tanf * (width / height))
        y_ndc = float(np.dot(v, cam_up)) / (z * tanf)
        return (x_ndc + 1.0) / 2.0 * width, (1.0 - y_ndc) / 2.0 * height

    def apply_state(state: dict) -> None:
        mujoco.mj_resetData(model, data)
        data.qpos[
            model.jnt_qposadr[model.joint("base_free").id] : model.jnt_qposadr[
                model.joint("base_free").id
            ]
            + 7
        ] = np.asarray(prepared_base, dtype=float)
        for name, value in state["joints"].items():
            data.qpos[model.jnt_qposadr[model.joint(name).id]] = value
        if model.body("target_cube").id >= 0:
            cube_qpos = int(model.jnt_qposadr[model.body_jntadr[model.body("target_cube").id]])
            data.qpos[cube_qpos : cube_qpos + 3] = np.asarray(
                backend_config["g4a1"]["initial_cube_position_m"], dtype=float
            )
        mujoco.mj_forward(model, data)

    mujoco_fk: dict[str, dict] = {}
    renderer = mujoco.Renderer(model, height, width)
    fovy_used = 45.0
    for state in states:
        apply_state(state)
        mujoco_fk[state["name"]] = {
            str(model.body(i).name): {
                "position_m": [float(v) for v in data.xpos[i]],
                "orientation_xyzw": _xyzw(data.xquat[i]),
            }
            for i in range(model.nbody)
        }
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.lookat = lookat
        camera.azimuth = math.degrees(az)
        camera.elevation = math.degrees(el)
        camera.distance = dist
        renderer.update_scene(data, camera)
        try:
            fovy_used = float(renderer._scene.camera[0].fovy)
        except Exception:
            fovy_used = 45.0
        image = renderer.render()
        png_path = args.output_dir / f"mujoco_{state['name']}.png"
        _write_png(png_path, image)
        report["states"][state["name"]] = {
            "mujoco_render": png_path.name,
            "mujoco_camera_fovy_deg": fovy_used,
        }
    try:
        renderer.close()
    except Exception:
        pass

    # --- 3. manifest + frozen snapshot ------------------------------------
    manifest = build_scene_manifest(
        model, model_path, model_id="diag", scene_id="mobile_pick"
    )
    manifest_json = json.loads(manifest.model_dump_json())
    (args.output_dir / "scene_manifest.json").write_text(
        json.dumps(manifest_json, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    mesh_asset_files = {}
    for node in ET.parse(model_dir / "assets.xml").getroot().findall(".//mesh"):
        name, file_name = node.get("name"), node.get("file")
        if name and file_name:
            mesh_asset_files[name.removesuffix("_mesh") or name] = (model_dir / file_name).resolve()

    diag = DiagnosticState()
    diag.snapshot = build_frozen_snapshot(states[0], model, data)

    static_root = REPO / "src/mobilecar/ui/static"

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            return

        def _json(self, value, status=200):
            body = json.dumps(value).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/api/v1/health":
                return self._json(
                    {
                        "status": "ok",
                        "backend_generation": 1,
                        "ui": {"adaptive_rendering": False, "snapshot_hz": 15.0},
                    }
                )
            if path == "/api/v1/capabilities":
                return self._json(
                    {
                        "model_sha256": manifest_json["model_sha256"],
                        "scene_id": "mobile_pick",
                        "camera_parent_body": "link4",
                    }
                )
            if path == "/api/v1/scene":
                return self._json(manifest_json)
            if path == "/api/v1/snapshot":
                return self._json(diag.snapshot)
            if path.startswith("/api/v1/scene/assets/"):
                asset_id = path.rsplit("/", 1)[-1]
                file_path = mesh_asset_files.get(asset_id)
                if file_path is None or not file_path.is_file():
                    return self._json({"detail": "asset not found"}, 404)
                body = file_path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "model/stl")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            if path.startswith("/api/"):
                return self._json({"detail": "diagnostic: not intercepted"}, 404)
            if path.startswith("/static/"):
                candidate = (static_root / path[len("/static/") :]).resolve()
            elif path == "/":
                candidate = static_root / "index.html"
            else:
                candidate = (static_root / path.lstrip("/")).resolve()
            if (
                not candidate.is_file()
                or not str(candidate).startswith(str(static_root.resolve()))
            ):
                candidate = static_root / "index.html"
            body = candidate.read_bytes()
            content_type = (
                "text/html"
                if candidate.suffix == ".html"
                else "text/css"
                if candidate.suffix == ".css"
                else "application/javascript"
                if candidate.suffix == ".js"
                else "application/octet-stream"
            )
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            path = self.path.split("?", 1)[0]
            length = int(self.headers.get("Content-Length") or 0)
            self.rfile.read(length)
            if path == "/api/v1/lease/renew":
                return self._json({"renewed": False, "reason": "diagnostic"})
            return self._json({"detail": "diagnostic: POST blocked"}, 409)

    server = ThreadingHTTPServer(("127.0.0.1", args.static_port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"

    read_back_js = """() => {
      const sv = window.mobileCarApp.sceneView;
      const rootInv = sv.mujocoRoot.matrixWorld.clone().invert();
      const result = {};
      for (const [name, entry] of sv.linkGroups) {
        entry.group.updateWorldMatrix(true, false);
        const m = rootInv.clone().multiply(entry.group.matrixWorld);
        const p = new entry.group.position.constructor().setFromMatrixPosition(m);
        const q = new entry.group.quaternion.constructor();
        if (q.setFromRotationMatrix) q.setFromRotationMatrix(m);
        else q.setFromMatrixRotationMatrix(m);
        result[name] = {
          position: [p.x, p.y, p.z],
          quaternion_xyzw: [q.x, q.y, q.z, q.w],
        };
      }
      return result;
    }"""

    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            headless=True,
            args=[
                "--no-sandbox",
                "--disable-dev-shm-usage",
                "--use-angle=swiftshader",
                "--enable-unsafe-swiftshader",
            ],
        )
        page = browser.new_page(viewport={"width": 1280, "height": 900})
        page_errors: list[str] = []
        console_messages: list[str] = []
        page.on("pageerror", lambda error: page_errors.append(str(error)))
        page.on("console", lambda message: console_messages.append(f"{message.type}:{message.text}"))
        page.on("requestfailed", lambda request: console_messages.append(f"requestfailed:{request.url}"))
        page.goto(base_url + "/", wait_until="domcontentloaded", timeout=20000)
        page.wait_for_function(
            "window.mobileCarApp && window.mobileCarApp.sceneView", timeout=15000
        )
        # The compiled model keeps meshes even when no geom references them
        # (e.g. imu_link_mesh); only geom-referenced assets are ever requested
        # by the browser, so wait on exactly those.
        used_assets: set[str] = set()
        for link in manifest_json["links"] + manifest_json["world_links"]:
            for geom in link.get("geoms", []):
                if geom.get("geom_type") == "mesh" and geom.get("mesh_asset_id"):
                    used_assets.add(geom["mesh_asset_id"])
        expected_assets = [f"/api/v1/scene/assets/{asset}" for asset in sorted(used_assets)]
        page_state = page.evaluate(
            """(expected) => {
              const entries = performance.getEntriesByType('resource')
                .map((e) => e.name)
                .filter((name) => name.includes('/api/v1/scene/assets/'));
              const missing = expected.filter((uri) =>
                !entries.some((name) => name.includes(uri)));
              let pending = 0;
              window.mobileCarApp.sceneView.actualRoot.traverse((node) => {
                if (node.userData && node.userData.meshPending && node.visible) pending += 1;
              });
              return {
                manifest_loaded: !!window.mobileCarApp.manifest,
                link_groups: window.mobileCarApp.sceneView.linkGroups.size,
                scene_asset_entries: entries.length,
                missing_assets: missing,
                mesh_pending_visible: pending,
              };
            }""",
            expected_assets,
        )
        print("PAGE_STATE " + json.dumps(page_state, ensure_ascii=False)[:2400])
        print("CONSOLE " + json.dumps(console_messages[-25:], ensure_ascii=False)[:2400])
        try:
            page.wait_for_function(
                """(expected) => {
                  const entries = performance.getEntriesByType('resource')
                    .map((e) => e.name)
                    .filter((name) => name.includes('/api/v1/scene/assets/'));
                  return expected.every((uri) => entries.some((name) => name.includes(uri)));
                }""",
                arg=expected_assets,
                timeout=30000,
            )
        except Exception:
            page_state = page.evaluate(
                """(expected) => {
                  const entries = performance.getEntriesByType('resource')
                    .map((e) => e.name)
                    .filter((name) => name.includes('/api/v1/scene/assets/'));
                  return {
                    scene_asset_entries: entries.length,
                    missing_assets: expected.filter((uri) =>
                      !entries.some((name) => name.includes(uri))),
                  };
                }""",
                expected_assets,
            )
            print("ASSET_WAIT_TIMEOUT " + json.dumps(page_state, ensure_ascii=False)[:2400])
            print("CONSOLE_AFTER " + json.dumps(console_messages[-25:], ensure_ascii=False)[:2400])
            raise
        page.wait_for_timeout(1200)
        render_info = page.evaluate(
            "() => ({ render_count: window.mobileCarApp.sceneView.renderCount })"
        )
        report["webgl"] = {
            "render_count": render_info["render_count"],
            "canvas_screenshots_meaningful": render_info["render_count"] > 0,
        }

        for state in states:
            apply_state(state)
            diag.snapshot = build_frozen_snapshot(state, model, data)
            marker = state["joints"]["joint1"]
            page.wait_for_function(
                "(marker) => { const s = window.mobileCarApp.snapshot;"
                " return s && Math.abs(s.arm.model_joint_positions_rad[0] - marker) < 1e-9; }",
                arg=marker,
                timeout=10000,
            )
            page.wait_for_timeout(400)
            three = page.evaluate(read_back_js)
            report["states"][state["name"]]["threejs_links"] = three

            # matched camera screenshot
            page.evaluate(
                """(cam) => {
                  const sv = window.mobileCarApp.sceneView;
                  sv.camera.fov = cam.fovy;
                  sv.camera.aspect = cam.width / cam.height;
                  sv.camera.near = 0.01;
                  sv.camera.far = 50;
                  sv.camera.position.set(cam.position[0], cam.position[1], cam.position[2]);
                  sv.controls.target.set(cam.target[0], cam.target[1], cam.target[2]);
                  sv.controls.update();
                  sv.camera.updateProjectionMatrix();
                  sv._requestRender(true);
                }""",
                {
                    "fovy": fovy_used,
                    "width": width,
                    "height": height,
                    # MuJoCo Z-up -> Three.js Y-up: (x, y, z) -> (x, z, -y)
                    "position": [cam_pos[0], cam_pos[2], -cam_pos[1]],
                    "target": [lookat[0], lookat[2], -lookat[1]],
                },
            )
            page.wait_for_timeout(600)
            canvas = page.locator("#three-container canvas")
            png_path = args.output_dir / f"threejs_{state['name']}.png"
            canvas.screenshot(path=str(png_path))
            report["states"][state["name"]]["threejs_render"] = png_path.name

            # Pixel projection cross-check: the same reference points must land
            # on the same pixels in the MuJoCo free-camera image and the
            # browser's matched camera (settles view handedness numerically).
            reference_points = {
                "base_link": [float(v) for v in data.xpos[model.body("base_link").id]],
                "tcp": [float(v) for v in data.site_xpos[model.site("grasp_tcp").id]],
                "target_cube": [float(v) for v in data.xpos[model.body("target_cube").id]],
            }
            three_pixels = page.evaluate(
                """(args) => {
                  const sv = window.mobileCarApp.sceneView;
                  const out = {};
                  for (const [name, point] of Object.entries(args.points)) {
                    const world = sv.mujocoRoot.localToWorld(
                      new sv.camera.position.constructor(point[0], point[1], point[2]));
                    const ndc = world.project(sv.camera);
                    out[name] = {
                      px: (ndc.x + 1) / 2 * args.width,
                      py: (1 - ndc.y) / 2 * args.height,
                    };
                  }
                  return out;
                }""",
                {"points": reference_points, "width": width, "height": height},
            )
            projection = {}
            for name, point in reference_points.items():
                mj_px, mj_py = project_mujoco(np.asarray(point), fovy_used)
                three = three_pixels[name]
                projection[name] = {
                    "mujoco_px": [round(mj_px, 2), round(mj_py, 2)],
                    "threejs_px": [round(three["px"], 2), round(three["py"], 2)],
                    "pixel_error": round(
                        math.hypot(mj_px - three["px"], mj_py - three["py"]), 2
                    ),
                }
            report["states"][state["name"]]["projection"] = projection

        report["page_errors"] = page_errors
        browser.close()
    server.shutdown()

    # --- 4. comparison ------------------------------------------------------
    worst_position_mm = 0.0
    worst_orientation_deg = 0.0
    details = {}
    for state in states:
        name = state["name"]
        three = report["states"][name]["threejs_links"]
        per_link = {}
        for body_name, mujoco_pose in mujoco_fk[name].items():
            three_pose = three.get(body_name)
            if three_pose is None:
                per_link[body_name] = {"error": "missing in Three.js manifest"}
                continue
            dpos = np.asarray(three_pose["position"]) - np.asarray(mujoco_pose["position_m"])
            pos_mm = float(np.linalg.norm(dpos) * 1000.0)
            ori_deg = _quat_angle_deg(three_pose["quaternion_xyzw"], mujoco_pose["orientation_xyzw"])
            per_link[body_name] = {
                "position_error_mm": pos_mm,
                "orientation_error_deg": ori_deg,
            }
            worst_position_mm = max(worst_position_mm, pos_mm)
            worst_orientation_deg = max(worst_orientation_deg, ori_deg)
        details[name] = per_link
    report["comparison"] = {
        "per_link": details,
        "worst_position_error_mm": worst_position_mm,
        "worst_orientation_error_deg": worst_orientation_deg,
        "position_tol_mm": args.position_tol_mm,
        "orientation_tol_deg": args.orientation_tol_deg,
        "consistent": (
            worst_position_mm <= args.position_tol_mm
            and worst_orientation_deg <= args.orientation_tol_deg
        ),
    }
    worst_pixel = 0.0
    for state in states:
        for proj in report["states"][state["name"]]["projection"].values():
            worst_pixel = max(worst_pixel, float(proj["pixel_error"]))
    report["projection"] = {
        "worst_pixel_error_px": worst_pixel,
        "tolerance_px": 3.0,
        "consistent": worst_pixel <= 3.0,
    }
    report["passed"] = (
        report["mesh_convention"]["all_ok"]
        and report["comparison"]["consistent"]
        and report["projection"]["consistent"]
        and not page_errors
    )
    report["finished_wall_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    (args.output_dir / "render_compare.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "passed": report["passed"],
                "mesh_convention_all_ok": report["mesh_convention"]["all_ok"],
                "worst_position_error_mm": round(worst_position_mm, 4),
                "worst_orientation_error_deg": round(worst_orientation_deg, 4),
                "worst_pixel_error_px": round(worst_pixel, 2),
                "webgl_render_count": render_info["render_count"],
                "page_errors": page_errors[:3],
            },
            ensure_ascii=False,
        )
    )
    return 0 if report["passed"] else 1


def _write_png(path: Path, image) -> None:
    array = np.asarray(image, dtype=np.uint8)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError(f"unexpected render buffer shape: {array.shape}")
    height, width = array.shape[0], array.shape[1]
    raw = b"".join(b"\x00" + array[row].tobytes() for row in range(height))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
        )

    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )
    path.write_bytes(png)


if __name__ == "__main__":
    raise SystemExit(main())
