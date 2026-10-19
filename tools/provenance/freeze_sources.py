#!/usr/bin/env python3
"""Freeze selected read-only mujoco_learning sources into a provenance manifest."""

from __future__ import annotations

import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
SOURCE_REPO = (REPO.parent / "mujoco_learning").resolve()
OUT = REPO / "provenance/source_manifest.json"

PAIRS = [
    ("robots/landerpi/build_model.py", "src/mobilecar/backends/modelgen/build_model.py", "adapted package/path constants"),
    ("robots/landerpi/parameters.py", "src/mobilecar/backends/modelgen/parameters.py", "adapted default config path"),
    ("robots/landerpi/joint_coordinates.py", "src/mobilecar/backends/modelgen/joint_coordinates.py", "exact source"),
    ("robots/landerpi/model/__init__.py", "src/mobilecar/backends/modelgen/model/__init__.py", "exact source"),
    ("robots/landerpi/model/arm.py", "src/mobilecar/backends/modelgen/model/arm.py", "removed runtime camera-on-TCP branch; camera remains link4 branch"),
    ("robots/landerpi/model/source.py", "src/mobilecar/backends/modelgen/model/source.py", "adapted asset root to mobileCar"),
    ("robots/landerpi/controller.py", "src/mobilecar/control/wheel.py", "adapted import; core four-mecanum torque controller retained"),
    ("robots/landerpi/manipulation/controller.py", "src/mobilecar/control/arm.py", "adapted imports; added bounded gravity/Coriolis feed-forward"),
    ("robots/landerpi/manipulation/kinematics.py", "src/mobilecar/planning/ik.py", "adapted scene assumptions; DLS/path checks retained"),
    ("robots/landerpi/manipulation/trajectory.py", "src/mobilecar/planning/trajectory.py", "adapted import; synchronized quintic/time-parameterized path retained"),
    ("robots/landerpi/manipulation/contacts.py", "src/mobilecar/sensors/contacts.py", "exact contact-force extraction"),
    ("robots/landerpi/config/nominal.yaml", "configs/robots/landerpi_nominal.yaml", "fixed_arm switched false; G1 development profile"),
    ("resource/objects/cube_20mm.xml", "assets/landerpi/objects/cube_20mm.xml", "exact asset"),
    ("resource/scenes/mobile_pick.xml", "assets/landerpi/scenes/mobile_pick.xml", "exact scene include"),
]

for name in ("mecanum.xacro", "arm.urdf.xacro", "gripper.urdf.xacro"):
    PAIRS.append(
        (
            f"resource/lander_pi/src/simulations/landerpi_description/urdf/{name}",
            f"assets/landerpi/source_description/urdf/{name}",
            "exact source asset",
        )
    )
for group in ("mecanum", "arm", "gripper"):
    for source in sorted(
        (SOURCE_REPO / f"resource/lander_pi/src/simulations/landerpi_description/meshes/{group}").glob("*.STL")
    ):
        PAIRS.append(
            (
                source.relative_to(SOURCE_REPO).as_posix(),
                f"assets/landerpi/source_description/meshes/{group}/{source.name}",
                "exact binary STL",
            )
        )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def command(*args: str) -> str:
    return subprocess.check_output(args, cwd=SOURCE_REPO, text=True).strip()


def main() -> None:
    head = command("git", "rev-parse", "HEAD")
    status = command("git", "status", "--porcelain=v1")
    files = []
    for source_rel, target_rel, adaptation in PAIRS:
        source = (SOURCE_REPO / source_rel).resolve()
        target = (REPO / target_rel).resolve()
        if not source.is_file() or not target.is_file():
            raise FileNotFoundError(f"missing provenance pair: {source} -> {target}")
        source_hash = sha256(source)
        target_hash = sha256(target)
        files.append(
            {
                "source_absolute_path": str(source),
                "source_git_head": head,
                "source_sha256": source_hash,
                "copy_verification_sha256": source_hash,
                "target_path": target.relative_to(REPO).as_posix(),
                "target_sha256": target_hash,
                "adapted": source_hash != target_hash,
                "adaptation": adaptation,
                "license": "No repository-level LICENSE/NOTICE was present at the frozen source HEAD; upstream asset origin requires owner confirmation before redistribution.",
            }
        )
    generated = []
    for path in sorted((REPO / "assets/landerpi/model").glob("*")):
        if path.is_file():
            generated.append(
                {
                    "target_path": path.relative_to(REPO).as_posix(),
                    "sha256": sha256(path),
                    "generator": "src/mobilecar/backends/modelgen/build_model.py",
                    "config": "configs/robots/landerpi_nominal.yaml",
                }
            )
    payload = {
        "schema_version": 1,
        "frozen_wall_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "source_repository": str(SOURCE_REPO),
        "source_head": head,
        "source_dirty_status_sha256": hashlib.sha256(status.encode()).hexdigest(),
        "source_dirty": bool(status),
        "note": "HEAD is not a complete identity because the source worktree has uncommitted and untracked files.",
        "files": files,
        "generated_model_files": generated,
        "forbidden_runtime_dependencies": [
            "mujoco_learning sys.path",
            "mujoco_learning experiments",
            "mutable source asset directories",
        ],
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(OUT)


if __name__ == "__main__":
    main()

