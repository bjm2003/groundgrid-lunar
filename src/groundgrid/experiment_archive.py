"""ROS-free archive validation. A process exit code is not a baseline verdict."""
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import tarfile
import xml.etree.ElementTree as ET


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".partial")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, ensure_ascii=False)
    os.replace(str(temporary), str(path))


def isolated_environment(base, directory, run_id, commit):
    env = dict(base)
    root = Path(directory).resolve()
    for key, folder in (("ROS_HOME", "ros-home"), ("ROS_LOG_DIR", "roslog"),
                        ("ROS_TEST_RESULTS_DIR", "test_results")):
        target = root / folder
        target.mkdir(parents=True, exist_ok=False)
        env[key] = str(target)
    env["GROUNDGRID_RUN_ID"] = run_id
    env["GROUNDGRID_RUN_COMMIT"] = commit
    return env


def load_json(path):
    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream)


def validate_run(directory, run_id, commit, scenario, strategy, primitive_mode, capture):
    """Return missing/inconsistent evidence separately from ROS assertion failures."""
    root = Path(directory)
    errors = []
    report = None
    try:
        report = load_json(root / ("planner_metrics_%s.json" % scenario))
        if report.get("run_id") != run_id or report.get("commit") != commit:
            errors.append("metrics identity mismatch (not this run/commit)")
        if report.get("scenario") != scenario:
            errors.append("metrics scenario mismatch")
        parameters = report.get("effective_parameters", {})
        planner = parameters.get("/state_lattice_planner", {})
        if planner.get("snap_strategy") != strategy:
            errors.append("effective strategy mismatch")
        if bool(planner.get("use_dynamics_primitives")) != (primitive_mode == "dynamics"):
            errors.append("requested primitive mode mismatch")
        trials = report["trials"]
        attempts = [a for t in trials for a in t.get("planning_attempts", [])]
        ids = [a["attempt_id"] for a in attempts]
        if len(ids) != len(set(ids)) or not ids:
            errors.append("missing or duplicate planning attempts")
        for trial in trials:
            for a in trial.get("planning_attempts", []):
                if a["goal_id"] != trial["goal_id"] or a["goal_stamp_ns"] != trial["goal_stamp_ns"]:
                    errors.append("planning attempt belongs to another goal")
                if a.get("snap_strategy") != strategy or a.get("primitive_mode") != primitive_mode:
                    errors.append("actual core mode mismatch (including silent dynamics fallback)")
                if not math.isfinite(a["total_ms"]) or a["total_ms"] < 0:
                    errors.append("invalid complete-attempt duration")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        errors.append("metrics unavailable/invalid: %s" % exc)
    xml_paths = list((root / "test_results").rglob("*.xml"))
    if not xml_paths:
        errors.append("missing rostest XML")
    if not list((root / "test_results").rglob("rosunit-lunar_pipeline_test.xml")):
        errors.append("missing lunar pipeline assertion XML (launcher XML alone is insufficient)")
    for path in xml_paths:
        try:
            ET.parse(str(path))
        except ET.ParseError:
            errors.append("invalid XML: %s" % path.name)
    if capture:
        try:
            snapshots = root / "snapshots"
            stats = load_json(snapshots / "writer-summary.json")
            if not (stats["submitted"] > 0 and stats["submitted"] == stats["written"]
                    and stats["dropped"] == 0 and stats["failed"] == 0):
                errors.append("snapshot writer incomplete: %s" % stats)
            binaries = list(snapshots.glob("attempt-*.ggsnap"))
            if len(binaries) != stats["written"] or list(snapshots.glob("*.partial")):
                errors.append("snapshot payload count/incomplete files")
            for binary in binaries:
                summary = load_json(binary.with_suffix(".json"))
                trajectory = load_json(binary.with_name(binary.stem + "-trajectory.json"))
                if summary["attempt_id"] != trajectory["attempt_id"]:
                    errors.append("snapshot/trajectory identity mismatch")
            if report:
                for t in report.get("trials", []):
                    for a in t.get("planning_attempts", []):
                        if not (snapshots / ("attempt-%d.ggsnap" % a["attempt_id"])).is_file():
                            errors.append("missing consumed-attempt snapshot %s" % a["attempt_id"])
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append("snapshot archive invalid: %s" % exc)
    return sorted(set(errors)), report


def acceptance_gaps(report):
    """Task-book gaps are not hidden by looser scenario-specific rostest gates."""
    gaps = ["target-platform perception+planning CPU <40% not established",
            "autonomous speed >=5 km/h not established", "field validation not performed"]
    if not report:
        return gaps + ["no attributable metrics"]
    rates = report.get("rates", {})
    for key in ("plan_success_all", "reach_tour", "completion_tour"):
        value = rates.get(key)
        if value is None or not math.isfinite(value) or value < 0.99:
            gaps.append("%s below task-book 99%%: %s" % (key, rates.get(key)))
    attempts = {a["attempt_id"]: a for t in report.get("trials", [])
                for a in t.get("planning_attempts", [])}
    times = [a["total_ms"] for a in attempts.values() if math.isfinite(a["total_ms"])]
    if not times or max(times) >= 1000:
        gaps.append("complete planning-attempt latency <1s not met/proven")
    return gaps


def attempt_statistics(report):
    attempts = {a["attempt_id"]: a for t in (report or {}).get("trials", [])
                for a in t.get("planning_attempts", [])}
    times = sorted(a["total_ms"] for a in attempts.values() if math.isfinite(a["total_ms"]))
    return {"scope": "goal-correlated metric trials; diagnostic prelude excluded",
            "unique_attempts": len(attempts),
            "mean_ms": statistics.mean(times) if times else None,
            "p95_ms": times[max(0, math.ceil(0.95 * len(times)) - 1)] if times else None,
            "max_ms": max(times) if times else None,
            "mission_durations_s": [t["duration_s"] for t in (report or {}).get("trials", [])]}


IDENTIFICATION_TRUTH = {"alpha_v": 0.90, "alpha_w": 0.85, "x_icr": 0.15,
                        "slope_slip_gain": 0.30, "slope_grade_gain": 0.40}


def identification_result(parameters):
    errors = {key: abs(float(parameters[key]) - value) for key, value in IDENTIFICATION_TRUTH.items()}
    finite = all(math.isfinite(error) for error in errors.values())
    return {"truth": IDENTIFICATION_TRUTH, "identified": parameters, "errors": errors,
            "max_abs_error": max(errors.values()) if finite else None,
            "passed": finite and max(errors.values()) <= 0.02}


def seal_archive(directory):
    """Hash collected files and create an adjacent, never-overwritten tar."""
    root = Path(directory).resolve()
    manifest = {}
    for path in sorted(root.rglob("*")):
        # ROS convenience symlinks must not traverse outside this archive.
        if path.is_symlink() or not path.is_file() or path.name == "sha256.json":
            continue
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        manifest[str(path.relative_to(root)).replace(os.sep, "/")] = digest.hexdigest()
    write_json(root / "sha256.json", manifest)
    archive = root.with_name(root.name + ".tar.gz")
    with archive.open("xb") as stream:
        with tarfile.open(fileobj=stream, mode="w:gz", dereference=False) as tar:
            tar.add(str(root), arcname=root.name)
    return archive
