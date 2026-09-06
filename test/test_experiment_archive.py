#!/usr/bin/env python3
"""Offline regression for attribution, completeness and task-book gap reporting."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
from unittest import mock
import importlib.util

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from groundgrid.experiment_archive import (
    acceptance_gaps, attempt_statistics, isolated_environment, load_json,
    seal_archive, validate_run, write_json, identification_result, IDENTIFICATION_TRUTH)

SPEC = importlib.util.spec_from_file_location(
    "experiment_runner", Path(__file__).resolve().parents[1] / "scripts/run_planner_experiments.py")
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


class ArchiveTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "run"
        self.root.mkdir()
        self.env = isolated_environment({"ROS_LOG_DIR": "/old/log", "ROS_HOME": "/old/home"},
                                        self.root, "fresh", "sha")
        (self.root / "test_results" / "rosunit-lunar_pipeline_test.xml").write_text(
            '<testsuite tests="2" errors="0" failures="0"/>', encoding="utf-8")
        self.report = {"run_id": "fresh", "commit": "sha", "scenario": "mixed",
                       "effective_parameters": {"/state_lattice_planner": {
                           "snap_strategy": "legacy_nearest", "use_dynamics_primitives": False}},
                       "rates": {"plan_success_all": 1.0, "reach_tour": 1.0, "completion_tour": 1.0},
                       "trials": [{"goal_id": 4, "goal_stamp_ns": 100, "duration_s": 12,
                                   "planning_attempts": [{"attempt_id": 12, "goal_id": 4,
                                       "goal_stamp_ns": 100, "total_ms": 123.0,
                                       "snap_strategy": "legacy_nearest", "primitive_mode": "arcs"}]}]}
        self.metrics = self.root / "planner_metrics_mixed.json"
        write_json(self.metrics, self.report)

    def validate(self, capture=False):
        return validate_run(self.root, "fresh", "sha", "mixed", "legacy_nearest", "arcs", capture)[0]

    def test_isolation_and_no_directory_reuse(self):
        self.assertTrue(self.env["ROS_LOG_DIR"].startswith(str(self.root)))
        self.assertEqual(self.env["GROUNDGRID_RUN_COMMIT"], "sha")
        with self.assertRaises(FileExistsError):
            isolated_environment({}, self.root, "again", "sha")
        self.assertEqual(self.validate(), [])

    def test_stale_json_is_not_a_result(self):
        self.report["run_id"] = "old"
        write_json(self.metrics, self.report)
        self.assertIn("metrics identity mismatch", " ".join(self.validate()))

    def test_primitive_fallback_detected_from_actual_attempt(self):
        self.report["trials"][0]["planning_attempts"][0]["primitive_mode"] = "dynamics"
        write_json(self.metrics, self.report)
        self.assertIn("actual core mode mismatch", " ".join(self.validate()))

    def test_mismatched_or_duplicate_attempt_rejected(self):
        self.report["trials"].append(copy.deepcopy(self.report["trials"][0]))
        self.report["trials"][-1]["goal_id"] = 9
        write_json(self.metrics, self.report)
        errors = " ".join(self.validate())
        self.assertIn("duplicate", errors)
        self.assertIn("another goal", errors)

    def test_missing_xml_and_metrics_are_errors(self):
        self.metrics.unlink()
        (self.root / "test_results" / "rosunit-lunar_pipeline_test.xml").unlink()
        errors = self.validate()
        self.assertTrue(any("metrics unavailable" in e for e in errors))
        self.assertIn("missing rostest XML", errors)

    def snapshots(self):
        target = self.root / "snapshots"
        target.mkdir()
        (target / "attempt-12.ggsnap").write_bytes(b"fixture-not-a-real-snapshot")
        write_json(target / "attempt-12.json", {"attempt_id": 12})
        write_json(target / "attempt-12-trajectory.json", {"attempt_id": 12, "trajectory": []})
        write_json(target / "writer-summary.json", {"submitted": 1, "written": 1, "dropped": 0, "failed": 0})
        return target

    def test_capture_completeness(self):
        target = self.snapshots()
        self.assertEqual(self.validate(capture=True), [])
        write_json(target / "writer-summary.json", {"submitted": 2, "written": 1, "dropped": 1, "failed": 0})
        self.assertIn("incomplete", " ".join(self.validate(capture=True)))
        (target / "attempt-12-trajectory.json").unlink()
        self.assertIn("archive invalid", " ".join(self.validate(capture=True)))

    def test_complete_attempt_latency_and_gaps(self):
        self.assertEqual(attempt_statistics(self.report)["unique_attempts"], 1)
        self.assertEqual(attempt_statistics(self.report)["max_ms"], 123)
        self.assertEqual(attempt_statistics(self.report)["mission_durations_s"], [12])
        self.report["rates"]["completion_tour"] = 0.98
        self.report["trials"][0]["planning_attempts"][0]["total_ms"] = 1001
        gaps = " ".join(acceptance_gaps(self.report))
        self.assertIn("completion_tour", gaps)
        self.assertIn("latency", gaps)
        self.assertIn("CPU", gaps)
        self.assertIn("5 km/h", gaps)

    def test_hashes_and_no_archive_overwrite(self):
        path = seal_archive(self.root)
        manifest = load_json(self.root / "sha256.json")
        self.assertEqual(manifest["planner_metrics_mixed.json"],
                         hashlib.sha256(self.metrics.read_bytes()).hexdigest())
        with tarfile.open(path) as tar:
            self.assertIn("run/test_results/rosunit-lunar_pipeline_test.xml", tar.getnames())
        with self.assertRaises(FileExistsError):
            seal_archive(self.root)

    def test_identification_requires_all_five_finite_parameters(self):
        self.assertTrue(identification_result(dict(IDENTIFICATION_TRUTH))["passed"])
        wrong = dict(IDENTIFICATION_TRUTH, x_icr=0.18)
        self.assertFalse(identification_result(wrong)["passed"])
        wrong["x_icr"] = float("nan")
        self.assertFalse(identification_result(wrong)["passed"])
        with self.assertRaises(KeyError):
            identification_result({})

    def run_fake_suite(self, fail=False, interrupt=False):
        output = Path(self.temp.name) / "suite"
        repo = Path(self.temp.name) / "repo"
        repo.mkdir()

        def query(command, cwd=None):
            if command[0] == "rospack":
                return str(repo)
            if "rev-parse" in command:
                return "sha"
            return ""

        def execute(command, log_path, env):
            Path(log_path).write_text("fixture command\n", encoding="utf-8")
            if command[0] != "rostest":
                return 0
            self.assertNotIn("--text", command)  # disables XML aggregation in ROS Noetic
            if interrupt:
                raise KeyboardInterrupt()
            report = copy.deepcopy(self.report)
            report["run_id"] = env["GROUNDGRID_RUN_ID"]
            metrics = next(c.split(":=", 1)[1] for c in command if c.startswith("metrics_out:="))
            write_json(metrics, report)
            Path(env["ROS_TEST_RESULTS_DIR"], "rosunit-lunar_pipeline_test.xml").write_text(
                '<testsuite tests="2" errors="0" failures="0"/>', encoding="utf-8")
            return 2 if fail else 0

        argv = ["runner", "--scenarios", "mixed", "--repeat", "2", "--out-dir", str(output)]
        with mock.patch.object(sys, "argv", argv), mock.patch.object(RUNNER.sys, "platform", "linux"), \
             mock.patch.dict(RUNNER.os.environ, {"ROS_DISTRO": "noetic", "CONDA_PREFIX": ""}), \
             mock.patch.object(RUNNER, "command_output", side_effect=query), \
             mock.patch.object(RUNNER, "run_logged", side_effect=execute):
            rc = RUNNER.main()
        return rc, output, load_json(output / "suite.json")

    def test_sweep_keeps_both_failed_repeats(self):
        rc, output, suite = self.run_fake_suite(fail=True)
        self.assertEqual(rc, 1)
        self.assertEqual(len(suite["runs"]), 2)
        self.assertTrue(all(r["rostest_rc"] == 2 and not r["passed"] for r in suite["runs"]))
        self.assertNotEqual(suite["runs"][0]["run_id"], suite["runs"][1]["run_id"])
        self.assertTrue(output.with_name("suite.tar.gz").is_file())

    def test_sweep_success_is_not_formal_acceptance(self):
        rc, _, suite = self.run_fake_suite()
        self.assertEqual(rc, 0)
        self.assertTrue(suite["passed"])
        self.assertFalse(suite["formal_baseline"])

    def test_interrupted_run_still_archived(self):
        rc, output, suite = self.run_fake_suite(interrupt=True)
        self.assertEqual(rc, 1)
        self.assertTrue(suite["interrupted"])
        self.assertEqual(len(suite["runs"]), 0)
        self.assertTrue((output / "mixed-arcs-01" / "run.json").is_file())


if __name__ == "__main__":
    unittest.main()
