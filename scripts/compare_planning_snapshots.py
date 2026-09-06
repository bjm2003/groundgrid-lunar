#!/usr/bin/env python3
"""Replay both production strategies on identical inputs. Does not launch ROS nodes."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys

from groundgrid.experiment_archive import load_json, seal_archive, write_json


def stable_signature(replay):
    result = dict(replay["result"])
    for key in ("total_ms", "snap_ms", "search_ms", "profile_ms"):
        result.pop(key, None)
    return {"result": result, "export_safety_ok": replay["export_safety_ok"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", required=True, help="snapshot file or archive directory")
    parser.add_argument("--out-dir", required=True, help="NEW comparison directory")
    parser.add_argument("--expansions", type=int, default=5000)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--goal-id", nargs="+", type=int, help="optional explicit goal subset")
    parser.add_argument("--executable", help="standalone binary; otherwise uses rosrun")
    args = parser.parse_args()
    if args.expansions <= 0 or args.repeat < 2:
        parser.error("expansions >0 and repeat >=2 required for a deterministic check")
    source = Path(args.inputs).expanduser().resolve()
    files = [source] if source.is_file() else sorted(source.rglob("*.ggsnap"))
    if args.goal_id:
        files = [p for p in files if load_json(p.with_suffix(".json"))["goal_id"] in args.goal_id]
    if not files:
        parser.error("no snapshots matched; no successful empty report")
    root = Path(args.out_dir).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=False)
    command_prefix = [args.executable] if args.executable else ["rosrun", "groundgrid", "replay_planning_snapshot"]
    # Identify the executable checkout separately from each captured input's source commit.
    version = None
    try:
        repo = subprocess.check_output(["rospack", "find", "groundgrid"], text=True).strip()
        version = subprocess.check_output(["git", "-C", repo, "rev-parse", "HEAD"], text=True).strip()
    except (OSError, subprocess.SubprocessError):
        pass  # standalone Windows replay is labelled explicitly, never a ROS baseline
    write_json(root / "environment.json", {"arguments": vars(args), "replay_commit": version,
                                          "executable": command_prefix, "python": sys.version})
    comparisons = []
    errors = []
    try:
        for n, snapshot in enumerate(files):
            folder = root / ("%05d-%s" % (n, snapshot.stem))
            folder.mkdir()
            item = {"snapshot": str(snapshot),
                    "sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest(),
                    "replays": {}}
            original = snapshot.with_suffix(".json")
            if original.is_file():
                item["recorded"] = load_json(original)
            # Alternate order to expose, rather than select away, process/cache effects.
            strategies = ["legacy_nearest", "reachable_cost"]
            if n % 2:
                strategies.reverse()
            for strategy in strategies:
                for mode in ("expansions", "wall_time"):
                    command = command_prefix + [str(snapshot), "--strategy", strategy,
                                                "--repeat", str(args.repeat), "--trajectory"]
                    if mode == "expansions":
                        command += ["--expansions", str(args.expansions)]
                    name = strategy + "-" + mode
                    output = folder / (name + ".jsonl")
                    print("%d/%d %s" % (n + 1, len(files), name), flush=True)
                    with output.open("x", encoding="utf-8") as out, (folder / (name + ".stderr")).open("x") as err:
                        rc = subprocess.call(command, stdout=out, stderr=err)
                    record = {"command": shlex.join(command), "returncode": rc}
                    if rc:
                        errors.append("%s %s: replay tool failed" % (snapshot, name))
                    else:
                        lines = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
                        replays = [line for line in lines if "result" in line]
                        trajectories = [line["trajectory"] for line in lines if "trajectory" in line]
                        if len(replays) != args.repeat or len(trajectories) != args.repeat:
                            raise ValueError("missing replay results/trajectories")
                        safe = all(r["export_safety_ok"] for r in replays)
                        record.update({"results": replays, "all_exported_outputs_safe": safe})
                        if not safe:
                            errors.append("%s %s: exported sweep/conformance audit failed" % (snapshot, name))
                        if mode == "expansions":
                            stable = all(stable_signature(r) == stable_signature(replays[0]) for r in replays)
                            stable = stable and all(t == trajectories[0] for t in trajectories)
                            record["deterministic"] = stable
                            if not stable:
                                errors.append("%s %s: fixed-budget output is not deterministic" % (snapshot, name))
                        if original.is_file() and item["recorded"].get("snap_strategy", "legacy_nearest") == strategy:
                            original_path = snapshot.with_name(snapshot.stem + "-trajectory.json")
                            if original_path.is_file():
                                old = load_json(original_path)["trajectory"]
                                record["matches_recorded_trajectory"] = [t == old for t in trajectories]
                    item["replays"][name] = record
            comparisons.append(item)
            write_json(folder / "comparison.json", item)
    except (KeyboardInterrupt, Exception) as exc:
        errors.append("comparison interrupted/incomplete: " + str(exc))
    complete = len(comparisons) == len(files) and not errors
    write_json(root / "comparison.json", {
        "complete_and_safe": complete, "errors": errors, "inputs": len(files),
        "comparisons": comparisons,
        "note": "Planning ok=false is a result, not a tool error. Wall-time variability and all repeats are retained. "
                "Recorded ROS total_ms includes adaptation; offline total_ms excludes file I/O and output audit. "
                "A safe offline route is not proof of closed-loop mission completion."})
    print("ARCHIVE=" + str(seal_archive(root)), flush=True)
    return 0 if complete else 1


if __name__ == "__main__":
    sys.exit(main())
