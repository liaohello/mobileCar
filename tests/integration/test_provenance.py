from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def test_provenance_manifest_targets_and_hashes_are_complete():
    manifest = json.loads((ROOT / "provenance/source_manifest.json").read_text(encoding="utf-8"))
    assert manifest["schema_version"] == 1
    assert manifest["source_dirty"] is True
    assert manifest["source_head"] == "dcae2c81eb30c1bdb3b6d92a1116f1345d2a2fca"
    assert manifest["files"]
    for item in manifest["files"]:
        target = ROOT / item["target_path"]
        source = Path(item["source_absolute_path"])
        assert target.is_file(), item["target_path"]
        assert _sha256(target) == item["target_sha256"], item["target_path"]
        # The server release intentionally excludes the read-only source
        # project. When the source is available (local audit), verify both
        # copies; on the server, the release target and recorded source
        # metadata remain independently checkable.
        if source.is_file():
            assert _sha256(source) == item["source_sha256"], item["source_absolute_path"]
        assert item["source_git_head"] == manifest["source_head"]
        assert item["source_sha256"]
        assert item["source_absolute_path"]
        assert item["license"]
