#!/usr/bin/env python3
"""Safe SSH release/deploy/job helper for mobileCar server-only execution."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shlex
import subprocess
import sys
import tarfile
import tempfile
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
REMOTE_CONFIG = ROOT / "remote.md"


def _read_remote() -> dict[str, str]:
    result: dict[str, str] = {}
    for line in REMOTE_CONFIG.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        key, sep, value = line.partition("=")
        if sep:
            result[key.strip()] = value.strip().strip('"\'')
    return result


def _normalize_path(value: str) -> Path:
    if os.name != "nt" and len(value) > 2 and value[1] == ":" and value[2] in "\\/":
        value = "/mnt/" + value[0].lower() + "/" + value[3:].replace("\\", "/")
    return Path(value).expanduser()


def _connection() -> tuple[str, str | None, str | None, str]:
    cfg = _read_remote()
    tokens = shlex.split(cfg.get("ssh_command", ""))
    if not tokens or Path(tokens[0]).name != "ssh":
        raise RuntimeError("remote.md must contain an ssh command")
    key = port = target = None
    index = 1
    while index < len(tokens):
        token = tokens[index]
        if token == "-i" and index + 1 < len(tokens):
            key = tokens[index + 1]
            index += 2
        elif token == "-p" and index + 1 < len(tokens):
            port = tokens[index + 1]
            index += 2
        elif token.startswith("-"):
            raise RuntimeError(f"unsupported ssh option in remote.md: {token}")
        else:
            if target is not None:
                raise RuntimeError("multiple ssh targets in remote.md")
            target = token
            index += 1
    if target is None:
        raise RuntimeError("ssh target missing from remote.md")
    project = cfg.get("proj_path", "")
    if not project:
        raise RuntimeError("proj_path missing from remote.md")
    return target, key, port, project


def _resolved_key(raw_key: str | None) -> Path | None:
    if not raw_key:
        return None
    candidates = []
    key = _normalize_path(raw_key)
    candidates.append(key)
    if not key.is_absolute():
        candidates.append(Path.home() / ".ssh" / raw_key)
    cfg = _read_remote()
    for field in ("key_path", "key_path_windows"):
        value = cfg.get(field)
        if value:
            candidates.append(_normalize_path(value))
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise RuntimeError("configured SSH key not found")


def _project_expr(project: str) -> str:
    if project == "~":
        return '"$HOME"'
    if project.startswith("~/"):
        remainder = project[2:]
        return '"$HOME"/' + shlex.quote(remainder)
    if project.startswith("/"):
        return shlex.quote(project)
    raise RuntimeError("proj_path must be absolute or start with ~/")


def _ssh_base() -> tuple[list[str], str]:
    target, raw_key, port, project = _connection()
    command = [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "StrictHostKeyChecking=accept-new",
        "-o",
        "ServerAliveInterval=15",
        "-o",
        "ServerAliveCountMax=4",
    ]
    if raw_key:
        key = _resolved_key(raw_key)
        command.extend(["-i", str(key)])
    if port:
        command.extend(["-p", str(port)])
    command.append(target)
    return command, project


def _remote(project: str, script: str, *, check: bool = True, timeout: int = 120) -> subprocess.CompletedProcess:
    command, _ = _ssh_base()
    wrapped = f"cd -- {_project_expr(project)} && {script}"
    result = subprocess.run(
        command + [wrapped],
        check=False,
        text=True,
        stdin=subprocess.DEVNULL,
        timeout=timeout,
    )
    if check and result.returncode != 0:
        raise RuntimeError(f"remote command failed with exit status {result.returncode}")
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _release_files() -> list[Path]:
    roots = (
        "src",
        "assets",
        "configs",
        "tests",
        "scripts",
        "tools",
        "provenance",
        "docs",
        "ros2_ws",
    )
    files: list[Path] = []
    for root_name in roots:
        root = ROOT / root_name
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if not path.is_file() or path.is_symlink():
                continue
            relative = path.relative_to(ROOT)
            parts = set(relative.parts)
            if parts & {"__pycache__", ".pytest_cache", "node_modules", ".venv", "runs"}:
                continue
            relative_text = relative.as_posix()
            if relative.parts[:2] in {
                ("ros2_ws", "build"),
                ("ros2_ws", "install"),
                ("ros2_ws", "log"),
            }:
                continue
            if relative.parts[:2] == ("provenance", "releases"):
                continue
            if relative_text.startswith("docs/evidence/g2a_"):
                continue
            if relative_text.startswith("docs/evidence/g2b_"):
                continue
            if relative_text == "docs/evidence/pytest-g2a.xml":
                continue
            if relative_text == "docs/evidence/pytest-g2b.xml":
                continue
            if relative_text in {
                "docs/g2a_server_report.md",
                "docs/g2a_demo_record.md",
                "docs/g2a_reproduction.md",
                "docs/g2a_failures_and_fixes.md",
                "docs/workpackages/G2A.md",
                "docs/workpackages/G2A_agent_receipts.md",
                "docs/g2b_report.md",
                "docs/g2b_reproduction.md",
                "docs/g2b_demo_record.md",
                "docs/g2b_failures_and_fixes.md",
                "docs/workpackages/G2B.md",
                "docs/workpackages/G2B_agent_receipts.md",
                "docs/g3a_report.md",
                "docs/g3a_reproduction.md",
                "docs/g3a_demo_record.md",
                "docs/g3a_failures_and_fixes.md",
                "docs/workpackages/G3A_agent_receipts.md",
                "docs/evidence/g3a_server_artifact_index.json",
                "docs/evidence/pytest-g3a.xml",
            }:
                continue
            if any(part.endswith(".egg-info") for part in parts):
                continue
            if path.suffix in {".pyc", ".pyo"}:
                continue
            if relative.as_posix() in {"src/mobilecar/ui/static/app.js", "src/mobilecar/ui/static/app.js.map"}:
                continue
            files.append(path)
    for name in (
        "AGENTS.md",
        "README.md",
        "LICENSE",
        "pyproject.toml",
        "requirements.lock",
        "requirements-test.lock",
        "requirements-g2b-lerobot.lock",
        "package.json",
        "package-lock.json",
    ):
        path = ROOT / name
        if path.is_file():
            files.append(path)
    return sorted(set(files), key=lambda item: item.relative_to(ROOT).as_posix())


def preflight() -> int:
    command, project = _ssh_base()
    script = r'''
set -u
printf 'cwd=%s\n' "$PWD"
printf 'os='; . /etc/os-release 2>/dev/null && printf '%s\n' "$PRETTY_NAME" || uname -a
printf 'kernel='; uname -r
printf 'cpu_count='; nproc
printf 'memory='; awk '/MemTotal/{printf "%.2fGiB\n",$2/1024/1024}' /proc/meminfo
printf 'disk='; df -Pk . | awk 'NR==2{printf "%.2fGiB_available\n",$4/1024/1024}'
printf 'python='; command -v python3 || true
python3 --version 2>&1 || true
printf 'python310='; command -v python3.10 || true
printf 'node='; command -v node || true
node --version 2>&1 || true
printf 'npm='; command -v npm || true
printf 'gpu='; (nvidia-smi --query-gpu=index,name,memory.total,driver_version --format=csv,noheader 2>/dev/null || echo unavailable)
printf 'browser='; (command -v chromium || command -v chromium-browser || command -v google-chrome || echo unavailable)
printf 'git_head='; git rev-parse HEAD 2>/dev/null || echo no_git
printf 'git_dirty_count='; git status --porcelain 2>/dev/null | wc -l || true
printf 'project_exists=yes\n'
'''
    wrapped = f"cd -- {_project_expr(project)} 2>/dev/null && bash -c {shlex.quote(script)}"
    result = subprocess.run(command + [wrapped], text=True, stdin=subprocess.DEVNULL, timeout=60)
    return result.returncode


def init_layout() -> int:
    command, project = _ssh_base()
    expr = _project_expr(project)
    script = (
        f"mkdir -p {expr}/releases {expr}/envs {expr}/var/jobs "
        f"{expr}/var/reports {expr}/var/runs {expr}/var/datasets {expr}/var/maps"
    )
    result = subprocess.run(
        command + [script],
        text=True,
        stdin=subprocess.DEVNULL,
        timeout=30,
    )
    print(json.dumps({"layout_initialized": result.returncode == 0}, indent=2))
    return result.returncode


def sync(prefix: str = "g4a1") -> int:
    files = _release_files()
    manifest_lines = []
    hashes = []
    for path in files:
        relative = path.relative_to(ROOT).as_posix()
        digest = _sha256(path)
        hashes.append({"path": relative, "sha256": digest, "size": path.stat().st_size})
        manifest_lines.append(f"{digest}  {relative}")
    aggregate = hashlib.sha256(
        "".join(f"{item['path']} {item['sha256']}\n" for item in hashes).encode()
    ).hexdigest()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    snapshot_id = f"{prefix}-{stamp}-{aggregate[:12]}"
    manifest = {
        "snapshot_id": snapshot_id,
        "aggregate_sha256": aggregate,
        "file_count": len(hashes),
        "files": hashes,
        "generated_wall_utc": stamp,
        "excluded": [
            "runs",
            ".venv",
            "node_modules",
            "__pycache__",
            ".git",
            "remote.md",
            "generated UI bundle (server rebuild)",
            "G2A final local reports, evidence and artifact index (bound to runtime release)",
            "G2B final local reports, evidence and artifact index (bound to runtime release)",
            "G3A final local reports and artifact index (bound to runtime release)",
            "G3B final local reports and artifact index (bound to runtime release)",
            "G4A-1 final local reports and artifact index (bound to runtime release)",
            "ROS 2 generated build/install/log trees (built on server)",
            "locally generated provenance/releases manifests (retained as local index)",
        ],
    }
    command, project = _ssh_base()
    release_rel = f"releases/{snapshot_id}"
    _remote(
        project,
        f"mkdir -p releases/{shlex.quote(snapshot_id)} var/jobs var/reports var/runs",
        timeout=30,
    )
    with tempfile.TemporaryDirectory(prefix="mobilecar-g3b-release-") as temporary:
        temp = Path(temporary)
        (temp / "SOURCE_MANIFEST.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        (temp / "SHA256SUMS").write_text("\n".join(manifest_lines) + "\n", encoding="utf-8", newline="\n")
        archive = temp / f"{snapshot_id}.tar.gz"
        with tarfile.open(archive, "w:gz") as tar:
            for path in files:
                tar.add(path, arcname=path.relative_to(ROOT).as_posix())
            tar.add(temp / "SOURCE_MANIFEST.json", arcname="SOURCE_MANIFEST.json")
            tar.add(temp / "SHA256SUMS", arcname="SHA256SUMS")
        target, _, port, _ = _connection()
        scp = ["scp", "-q"]
        raw_key = _connection()[1]
        if raw_key:
            scp.extend(["-i", str(_resolved_key(raw_key))])
        if port:
            scp.extend(["-P", str(port)])
        remote_tmp = project.rstrip("/") + f"/releases/{snapshot_id}.tar.gz"
        subprocess.run(
            scp + [str(archive), f"{target}:{remote_tmp}"],
            check=True,
            stdin=subprocess.DEVNULL,
            timeout=600,
        )
    _remote(
        project,
        " && ".join(
            [
                f"tar -xzf {shlex.quote(release_rel + '.tar.gz')} -C {shlex.quote(release_rel)}",
                f"cd -- {shlex.quote(release_rel)}",
                "sha256sum -c SHA256SUMS",
                f"rm -f {shlex.quote(release_rel + '.tar.gz')}",
            ]
        ),
        timeout=600,
    )
    releases = ROOT / "provenance/releases"
    releases.mkdir(parents=True, exist_ok=True)
    (releases / f"{snapshot_id}.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "snapshot_id": snapshot_id,
                "aggregate_sha256": aggregate,
                "file_count": len(files),
                "remote_release": f"releases/{snapshot_id}",
            },
            indent=2,
        )
    )
    return 0


def execute(args: argparse.Namespace) -> int:
    _, _, _, project = _connection()
    job_id = args.job_id
    job_rel = f"var/jobs/{job_id}"
    command = [
        "set -o pipefail",
        f"cd -- {_project_expr(project)}",
        f"mkdir -p {shlex.quote(job_rel)}",
        f"printf '%s\\n' {shlex.quote(args.command)} > {shlex.quote(job_rel + '/command.txt')}",
        f"cd -- releases/{shlex.quote(args.release)}",
        f"timeout {int(args.timeout)} bash -c {shlex.quote(args.command)} "
        f"2>&1 | tee {shlex.quote('../../' + job_rel + '/console.log')}",
    ]
    command_string = "\n".join(command)
    ssh, _ = _ssh_base()
    result = subprocess.run(
        ssh + [command_string],
        text=True,
        stdin=subprocess.DEVNULL,
        timeout=args.timeout + 30,
    )
    _remote(
        project,
        f"printf '%s\\n' {result.returncode} > {shlex.quote(job_rel + '/exit_code')}",
        check=False,
        timeout=30,
    )
    print(json.dumps({"job_id": job_id, "exit_code": result.returncode, "remote_job": job_rel}, indent=2))
    return result.returncode


def fetch(args: argparse.Namespace) -> int:
    target, raw_key, port, project = _connection()
    scp = ["scp", "-q"]
    if raw_key:
        scp.extend(["-i", str(_resolved_key(raw_key))])
    if port:
        scp.extend(["-P", str(port)])
    local = Path(args.local)
    local.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        scp + [f"{target}:{project.rstrip('/')}/{args.remote.lstrip('/')}", str(local)],
        check=True,
        stdin=subprocess.DEVNULL,
        timeout=600,
    )
    print(json.dumps({"remote": args.remote, "local": str(local), "sha256": _sha256(local)}, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("init")
    sub.add_parser("preflight")
    sync_parser = sub.add_parser("sync")
    sync_parser.add_argument("--prefix", default="g4a1")
    run = sub.add_parser("exec")
    run.add_argument("--release", required=True)
    run.add_argument("--job-id", required=True)
    run.add_argument("--timeout", type=int, default=3600)
    run.add_argument("--command", required=True)
    fetch_parser = sub.add_parser("fetch")
    fetch_parser.add_argument("--remote", required=True)
    fetch_parser.add_argument("--local", required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.action == "init":
        return init_layout()
    if args.action == "preflight":
        return preflight()
    if args.action == "sync":
        return sync(args.prefix)
    if args.action == "exec":
        return execute(args)
    return fetch(args)


if __name__ == "__main__":
    raise SystemExit(main())
