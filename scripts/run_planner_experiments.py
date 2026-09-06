#!/usr/bin/env python3
"""Run fresh ROS processes; archive every success AND failure, never reuse old JSON."""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import platform
import shlex
import signal
import socket
import subprocess
import sys
import time
import uuid

from groundgrid.experiment_archive import (
    acceptance_gaps, attempt_statistics, isolated_environment, seal_archive,
    validate_run, write_json, IDENTIFICATION_TRUTH, identification_result)

SCENARIOS = ["mixed", "flat", "dense", "slope", "negative"]
SELFCHECKS = ["skidsteer_selfcheck", "trajectory_tracking_selfcheck",
              "planner_safety_selfcheck", "backout_recovery_selfcheck",
              "blind_zone_ground_selfcheck", "planning_core_selfcheck",
              "planning_snapshot_selfcheck", "planning_grid_parity_selfcheck"]


def command_output(command, cwd=None):
    return subprocess.check_output(command, cwd=cwd, text=True, stderr=subprocess.STDOUT).strip()


def run_logged(command, log_path, env):
    """Keep console short; complete output lives in the archive. Ctrl-C stops this group."""
    print("$ " + shlex.join(command), flush=True)
    with Path(log_path).open("x", encoding="utf-8") as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        try:
            return process.wait()
        except KeyboardInterrupt:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--n-trials", type=int, default=3)
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=SCENARIOS)
    parser.add_argument("--repeat", type=int, default=1, help="independent processes per scenario")
    parser.add_argument("--out-dir", required=True, help="NEW result directory (must not exist)")
    parser.add_argument("--snap-strategy", choices=["legacy_nearest", "reachable_cost"],
                        default="legacy_nearest")
    parser.add_argument("--primitive-mode", choices=["arcs", "dynamics"], default="arcs")
    parser.add_argument("--capture-inputs", action="store_true")
    parser.add_argument("--debug-control", action="store_true")
    parser.add_argument("--identify-first", action="store_true",
                        help="run the five-parameter simulator identification in an isolated master")
    args = parser.parse_args()
    if args.n_trials < 1 or args.repeat < 1:
        parser.error("n-trials and repeat must be positive")
    if os.environ.get("CONDA_PREFIX"):
        parser.error("exit Conda and source ROS Noetic before running")
    if os.environ.get("ROS_DISTRO") != "noetic" or sys.platform != "linux":
        parser.error("this runner requires Ubuntu / ROS Noetic")
    repo = Path(command_output(["rospack", "find", "groundgrid"])).resolve()
    commit = command_output(["git", "rev-parse", "HEAD"], cwd=str(repo))
    status = command_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=str(repo))
    if status:
        parser.error("tracked checkout changes prevent commit attribution:\n" + status)
    root = Path(args.out_dir).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=False)
    suite_id = uuid.uuid4().hex
    environment = {key: os.environ.get(key, "") for key in (
        "ROS_DISTRO", "ROS_VERSION", "ROS_MASTER_URI", "ROS_HOME", "ROS_LOG_DIR",
        "ROS_TEST_RESULTS_DIR", "CMAKE_PREFIX_PATH", "PYTHONPATH", "PATH", "CONDA_PREFIX")}
    metadata = {"schema_version": 1, "suite_id": suite_id, "commit": commit,
                "repository": str(repo), "tracked_status": status, "arguments": vars(args),
                "started_utc": datetime.now(timezone.utc).isoformat(),
                "platform": platform.platform(), "python": sys.version, "environment": environment}
    write_json(root / "environment.json", metadata)
    source_dir = root / "source-inputs"
    source_dir.mkdir()
    tracked = command_output(["git", "ls-files", "config", "launch", "test/lunar_pipeline.test",
                              "msg", "package.xml", "CMakeLists.txt"], cwd=str(repo)).splitlines()
    for relative in tracked:
        source = repo / relative
        target = source_dir / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
    results = []
    checks = []
    interrupted = False
    exception = None
    identification = None
    try:
        for check in SELFCHECKS:
            rc = run_logged(["rosrun", "groundgrid", check], root / (check + ".log"), os.environ.copy())
            checks.append({"name": check, "returncode": rc})
            if rc:
                raise RuntimeError("preflight failed: %s; inspect its archived log" % check)
        if args.identify_first:
            import yaml  # ROS Noetic runtime dependency, not required for offline archive tests
            directory = root / "identification"
            directory.mkdir()
            env = isolated_environment(os.environ, directory, uuid.uuid4().hex, commit)
            # The identifier publishes /cmd_vel: never attach it to a user's existing master.
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", 0))
                port = probe.getsockname()[1]
            env["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
            output = directory / "skid_steer_model.yaml"
            command = ["roslaunch", "-p", str(port), "groundgrid", "IdentifySkidSteer.launch",
                       "output:=" + str(output)]
            command += [key + ":=" + str(value) for key, value in IDENTIFICATION_TRUTH.items()]
            rc = run_logged(command, directory / "identify.log", env)
            with output.open(encoding="utf-8") as stream:
                identification = identification_result(yaml.safe_load(stream)["skid_steer_model"])
            identification["returncode"] = rc
            identification["command"] = command
            write_json(directory / "result.json", identification)
            if rc or not identification["passed"]:
                raise RuntimeError("five-parameter recovery failed (maximum error must be <=0.02)")
        for scenario in args.scenarios:
            for repetition in range(1, args.repeat + 1):
                run_id = uuid.uuid4().hex
                run = root / ("%s-%s-%02d" % (scenario, args.primitive_mode, repetition))
                run.mkdir()
                env = isolated_environment(os.environ, run, run_id, commit)
                metrics = run / ("planner_metrics_%s.json" % scenario)
                command = ["rostest", "groundgrid", "lunar_pipeline.test",
                           "scenario:=" + scenario, "n_trials:=" + str(args.n_trials),
                           "metrics_out:=" + str(metrics), "snap_strategy:=" + args.snap_strategy,
                           "use_dynamics_primitives:=" + str(args.primitive_mode == "dynamics").lower(),
                           "motion_primitive_file:=" + str(repo / "config/motion_primitives.dat"),
                           "debug_control:=" + str(args.debug_control).lower()]
                if args.capture_inputs:
                    command.append("planning_snapshot_directory:=" + str(run / "snapshots"))
                record = {"run_id": run_id, "commit": commit, "scenario": scenario,
                          "repetition": repetition, "directory": str(run), "command": command,
                          "ros_environment": {k: env[k] for k in (
                              "ROS_HOME", "ROS_LOG_DIR", "ROS_TEST_RESULTS_DIR")}}
                write_json(run / "run.json", record)
                started = time.monotonic()
                try:
                    record["rostest_rc"] = run_logged(command, run / "full.log", env)
                finally:
                    record["wall_duration_s"] = time.monotonic() - started
                    write_json(run / "run.json", record)
                record["results_rc"] = run_logged(
                    ["catkin_test_results", str(run / "test_results")], run / "catkin-test-results.txt", env)
                errors, report = validate_run(run, run_id, commit, scenario, args.snap_strategy,
                                              args.primitive_mode, args.capture_inputs)
                record["archive_errors"] = errors
                record["task_book_gaps"] = acceptance_gaps(report)
                record["attempt_statistics"] = attempt_statistics(report)
                record["passed"] = record["rostest_rc"] == 0 and record["results_rc"] == 0 and not errors
                write_json(run / "run.json", record)
                results.append(record)
                print("%s repeat=%d ROS=%d XML=%d archive_errors=%d (%.1fs)" %
                      (scenario, repetition, record["rostest_rc"], record["results_rc"],
                       len(errors), record["wall_duration_s"]), flush=True)
    except KeyboardInterrupt:
        interrupted = True
    except Exception as exc:
        # Preserve evidence even for an unexpected adapter/schema error, with failure
        # visible to the caller. This never turns an exception into a successful run.
        exception = type(exc).__name__ + ": " + str(exc)
        print(exception, file=sys.stderr, flush=True)
    expected = len(args.scenarios) * args.repeat
    passed = len(results) == expected and all(r["passed"] for r in results) and not interrupted and not exception
    write_json(root / "suite.json", {"suite_id": suite_id, "commit": commit, "preflight": checks,
                                   "runs": results, "expected_runs": expected, "passed": passed,
                                   "interrupted": interrupted, "error": exception,
                                   "identification": identification,
                                   "formal_baseline": False,
                                   "note": "This suite alone is not task-book acceptance; compare all required suites."})
    archive = seal_archive(root)
    print("ARCHIVE=" + str(archive), flush=True)
    print("SUITE_RC=" + ("0" if passed else "1"), flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
