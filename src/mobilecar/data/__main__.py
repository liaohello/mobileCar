"""CLI for validating and inspecting native mobileCar Episodes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .eligibility import evaluate_eligibility
from .g2a_quality import assess_g2a_eligibility
from .oracle import assess_grasp_oracle
from .replay import ReplayReader
from .validator import validate_episode


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m mobilecar.data")
    parser.add_argument("--root", type=Path, default=Path("runs"))
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("list")
    for name in ("info", "validate", "eligibility", "quality", "g2a-eligibility"):
        command = subparsers.add_parser(name)
        command.add_argument("episode_id")
        command.add_argument("--run-id")
        if name in {"quality", "g2a-eligibility"}:
            command.add_argument(
                "--profile",
                type=Path,
                default=Path("configs/profiles/g2a_rgbd_5hz.yaml"),
            )
            command.add_argument(
                "--action-profile",
                type=Path,
                default=Path("configs/profiles/g2a_action_teleop_v1.yaml"),
            )
            command.add_argument("--output-root", type=Path)
    assess = subparsers.add_parser("assess")
    assess.add_argument("episode_id")
    assess.add_argument("--run-id")
    assess.add_argument("--samples", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.action == "list":
        print(json.dumps(ReplayReader(args.root).list_episodes(), ensure_ascii=False, indent=2))
        return 0
    if args.action == "validate":
        result = validate_episode(args.root, args.episode_id, run_id=args.run_id).model_dump(mode="json")
    elif args.action == "eligibility":
        result = evaluate_eligibility(
            args.root,
            args.episode_id,
            run_id=args.run_id,
        ).model_dump(mode="json")
    elif args.action == "assess":
        samples = None
        if args.samples:
            samples = [
                json.loads(line)
                for line in args.samples.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
        result = assess_grasp_oracle(
            args.root,
            args.episode_id,
            run_id=args.run_id,
            samples=samples,
        ).model_dump(mode="json")
    elif args.action in {"quality", "g2a-eligibility"}:
        quality, eligibility = assess_g2a_eligibility(
            args.root,
            args.episode_id,
            run_id=args.run_id,
            profile_path=args.profile,
            action_profile_path=args.action_profile,
            output_root=args.output_root,
        )
        result = (
            quality.model_dump(mode="json")
            if args.action == "quality"
            else eligibility.model_dump(mode="json")
        )
    else:
        result = ReplayReader(args.root).read_episode(args.episode_id)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
