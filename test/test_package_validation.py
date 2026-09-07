#!/usr/bin/env python3
"""No ROS required: package runs cannot overwrite their own pipeline evidence."""
import json
from pathlib import Path
import re
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))
from groundgrid.experiment_archive import isolated_environment, write_json
from groundgrid.package_validation import (expected_package_xml, run_package_suite,
                                          validate_ctest_inventory)

CHECKS = [name + "_selfcheck" for name in ("skidsteer", "trajectory_tracking", "planner_safety",
          "backout_recovery", "blind_zone_ground", "planning_core", "planning_snapshot",
          "planning_grid_parity")]


class PackageTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.repo = self.base / "workspace" / "src" / "groundgrid"
        (self.repo / "test").mkdir(parents=True)
        shutil.copyfile(str(REPO / "CMakeLists.txt"), str(self.repo / "CMakeLists.txt"))
        for source in (REPO / "test").glob("test_*.py"):
            shutil.copyfile(str(source), str(self.repo / "test" / source.name))
        self.root = self.base / "package-output"
        self.root.mkdir()
        self.commands = []

    def run_fixture(self, issue=None):
        expected = expected_package_xml(self.repo)

        def execute(command, log, env, cwd=None):
            self.commands.append(command)
            log = Path(log)
            log.write_text("fixture\n", encoding="utf-8")
            build = self.repo.parent.parent / "build" / "groundgrid"
            if command[:2] == ["catkin", "build"]:
                build.mkdir(parents=True)
                cache = "CMAKE_BUILD_TYPE:STRING=Release\nCATKIN_ENABLE_TESTING:BOOL=ON\n"
                cache += "PYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3\n"
                cache += "CATKIN_TEST_RESULTS_DIR:INTERNAL=" + env["ROS_TEST_RESULTS_DIR"] + "\n"
                (build / "CMakeCache.txt").write_text(cache)
                return 1 if issue == "build" else 0
            if command[:2] == ["ctest", "-N"]:
                names = CHECKS + (["_ctest_groundgrid_rostest_test_lunar_pipeline.test"]
                                  if issue == "unfiltered" else [])
                log.write_text("\n".join("Test #%d: %s" % (i + 1, name) for i, name in enumerate(names)))
                return 0
            if command[0] == "ctest":
                self.assertEqual(command[-2:], ["-L", "^groundgrid_selfcheck$"])
                if issue == "empty-cpp":
                    log.write_text("No tests were found!!!\n")
                else:
                    log.write_text("\n".join("%d/8 Test #%d: %s ... Passed 0.00 sec" % (i+1, i+1, n)
                                             for i, n in enumerate(CHECKS)))
                full = build / "Testing" / "Temporary" / "LastTest.log"
                full.parent.mkdir(parents=True)
                full.write_text("eight complete C++ logs\n")
                return 8 if issue == "cpp" else 0
            if command[:2] == ["catkin", "test"]:
                if issue == "interrupt":
                    raise KeyboardInterrupt()
                metrics = Path(env["GROUNDGRID_METRICS_OUT"])
                self.assertEqual(metrics.parent, self.root / "package")
                self.assertFalse(metrics.exists())  # never copied from ~/.ros
                write_json(metrics, {"run_id": "old" if issue == "stale" else env["GROUNDGRID_RUN_ID"],
                    "commit": "sha", "scenario": "mixed", "rates": {},
                    "effective_parameters": {"/state_lattice_planner": {
                        "snap_strategy": "reachable_cost", "use_dynamics_primitives": False}},
                    "trials": [{"goal_id": 2, "goal_stamp_ns": 10, "duration_s": 5,
                        "planning_attempts": [{"attempt_id": 1, "goal_id": 2, "goal_stamp_ns": 10,
                            "total_ms": 20, "snap_strategy": "reachable_cost", "primitive_mode": "arcs"}]}]})
                xml_dir = Path(env["ROS_TEST_RESULTS_DIR"]) / "groundgrid"
                xml_dir.mkdir()
                for name, count in expected.items():
                    if issue == "missing-unit" and name.startswith("nosetests-"):
                        continue
                    fail = issue == "xml-failure" and name == "rosunit-lunar_pipeline_test.xml"
                    (xml_dir / name).write_text('<testsuite tests="%d" errors="0" failures="%d"/>'
                                               % (count, int(fail)))
                snapshots = Path(env["GROUNDGRID_PLANNING_SNAPSHOT_DIRECTORY"])
                snapshots.mkdir()
                (snapshots / "attempt-1.ggsnap").write_bytes(b"fixture")
                write_json(snapshots / "attempt-1.json", {"attempt_id": 1})
                write_json(snapshots / "attempt-1-trajectory.json", {"attempt_id": 1})
                write_json(snapshots / "writer-summary.json", {"submitted": 1, "written": 1,
                    "failed": 0, "dropped": int(issue == "dropped")})
                return 0  # reproduce catkin's misleading empty internal results summary
            self.assertEqual(command[0], "catkin_test_results")
            self.assertEqual(command[1], env["ROS_TEST_RESULTS_DIR"])
            return 0

        rc = run_package_suite(self.repo, self.root, "sha", execute, CHECKS)
        record = json.loads((self.root / "package" / "run.json").read_text())
        self.assertTrue(self.root.with_name(self.root.name + ".tar.gz").is_file())
        return rc, record

    def test_single_pipeline_run_and_separate_cpp_evidence(self):
        rc, record = self.run_fixture()
        self.assertEqual(rc, 0)
        self.assertTrue(record["passed"])
        self.assertEqual(sum(c[:2] == ["catkin", "test"] for c in self.commands), 1)
        self.assertEqual(sum(c[0] == "ctest" for c in self.commands), 2)  # list + run labelled only
        self.assertTrue((self.root / "package" / "ctest-cpp-full.log").is_file())

    def test_actual_xml_failure_overrides_zero_command_codes(self):
        rc, record = self.run_fixture("xml-failure")
        self.assertEqual(rc, 1)
        self.assertIn("failed assertions", " ".join(record["archive_errors"]))

    def test_missing_python_xml_fails(self):
        rc, record = self.run_fixture("missing-unit")
        self.assertEqual(rc, 1)
        self.assertIn("missing/duplicate package XML", " ".join(record["archive_errors"]))

    def test_stale_metrics_fails(self):
        rc, record = self.run_fixture("stale")
        self.assertEqual(rc, 1)
        self.assertIn("identity mismatch", " ".join(record["archive_errors"]))

    def test_dropped_snapshot_fails(self):
        rc, record = self.run_fixture("dropped")
        self.assertEqual(rc, 1)
        self.assertIn("writer incomplete", " ".join(record["archive_errors"]))

    def test_inventory_rejects_pipeline_and_does_not_run_tests(self):
        rc, record = self.run_fixture("unfiltered")
        self.assertEqual(rc, 1)
        self.assertIn("exactly the eight", record["error"])
        self.assertFalse(any(c[:2] == ["catkin", "test"] for c in self.commands))

    def test_build_failure_is_archived_without_running_tests(self):
        rc, record = self.run_fixture("build")
        self.assertEqual(rc, 1)
        self.assertIn("build failed", record["error"])
        self.assertEqual(len(self.commands), 1)

    def test_cpp_failure_stops_before_pipeline(self):
        rc, record = self.run_fixture("cpp")
        self.assertEqual(rc, 1)
        self.assertIn("C++ preflight failed", record["error"])
        self.assertFalse(any(c[:2] == ["catkin", "test"] for c in self.commands))

    def test_zero_tests_cpp_return_code_is_not_a_pass(self):
        rc, record = self.run_fixture("empty-cpp")
        self.assertEqual(rc, 1)
        self.assertIn("eight executed", record["error"])
        self.assertFalse(any(c[:2] == ["catkin", "test"] for c in self.commands))

    def test_interruption_is_archived(self):
        rc, record = self.run_fixture("interrupt")
        self.assertEqual(rc, 1)
        self.assertTrue(record["interrupted"])

    def test_label_and_output_hooks_match_production(self):
        cmake = (REPO / "CMakeLists.txt").read_text(encoding="utf-8")
        match = re.search(r"set_tests_properties\((.*?)PROPERTIES LABELS \"groundgrid_selfcheck\"\)",
                          cmake, re.S)
        self.assertEqual(set(match.group(1).split()), set(CHECKS))
        launch = ET.parse(str(REPO / "test/lunar_pipeline.test")).getroot()
        for arg, variable in (("metrics_out", "GROUNDGRID_METRICS_OUT"),
                              ("planning_snapshot_directory", "GROUNDGRID_PLANNING_SNAPSHOT_DIRECTORY")):
            self.assertEqual(launch.find("arg[@name='%s']" % arg).get("default"),
                             "$(optenv %s)" % variable)
        self.assertFalse(validate_ctest_inventory("Total Tests: 0", CHECKS))

    def test_independent_run_removes_inherited_package_destinations(self):
        keys = ("GROUNDGRID_METRICS_OUT", "GROUNDGRID_PLANNING_SNAPSHOT_DIRECTORY", "GROUNDGRID_DEBUG_CONTROL")
        env = isolated_environment(dict.fromkeys(keys, "/old"), self.root / "isolated", "id", "sha")
        self.assertTrue(all(k not in env for k in keys))


if __name__ == "__main__":
    unittest.main()
