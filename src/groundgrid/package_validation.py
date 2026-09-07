"""Isolated package gate: a labelled CTest preflight followed by ONE ROS package run."""
import ast
import os
from pathlib import Path
import re
import shutil
import time
import uuid

from .experiment_archive import (acceptance_gaps, attempt_statistics, isolated_environment,
                                 seal_archive, validate_run, validate_test_xml, write_json)


def expected_package_xml(repo):
    """Derive minimum unit counts from tracked test methods without importing ROS/tests."""
    repo = Path(repo)
    cmake = (repo / "CMakeLists.txt").read_text(encoding="utf-8")
    files = re.findall(r"catkin_add_nosetests\((test/[\w.]+\.py)\)", cmake)
    if not files or len(files) != len(set(files)):
        raise ValueError("missing/duplicate Python test registrations")
    expected = {"rosunit-lunar_pipeline_test.xml": 2, "rostest-test_lunar_pipeline.xml": 1}
    for relative in files:
        tree = ast.parse((repo / relative).read_text(encoding="utf-8"))
        count = sum(isinstance(node, ast.FunctionDef) and node.name.startswith("test_")
                    for node in ast.walk(tree))
        if not count:
            raise ValueError("no test methods in " + relative)
        expected["nosetests-" + relative.replace("/", ".") + ".xml"] = count
    return expected


def validate_package_xml(directory, expected):
    errors = []
    for name, minimum in expected.items():
        found = list(Path(directory).rglob(name))
        if len(found) != 1:
            errors.append("missing/duplicate package XML: " + name)
        else:
            errors.extend(validate_test_xml(found[0], minimum_tests=minimum))
    return errors


def validate_ctest_inventory(text, expected):
    names = re.findall(r"Test\s+#\d+:\s+(\S+)", text)
    return len(names) == len(expected) and set(names) == set(expected)


def run_package_suite(repo, root, commit, execute, selfchecks):
    """execute is injectable for no-ROS regression; logs/results stay in one new root."""
    repo, root = Path(repo).resolve(), Path(root).resolve()
    package = root / "package"
    package.mkdir(exist_ok=False)
    run_id = uuid.uuid4().hex
    env = isolated_environment(os.environ, package, run_id, commit)
    env["GROUNDGRID_METRICS_OUT"] = str(package / "planner_metrics_mixed.json")
    env["GROUNDGRID_PLANNING_SNAPSHOT_DIRECTORY"] = str(package / "snapshots")
    env["GROUNDGRID_DEBUG_CONTROL"] = "true"
    record = {"schema_version": 1, "run_id": run_id, "commit": commit, "passed": False,
              "commands": [], "archive_errors": [], "interrupted": False, "error": None,
              "ros_environment": {key: env[key] for key in (
                  "ROS_HOME", "ROS_LOG_DIR", "ROS_TEST_RESULTS_DIR", "GROUNDGRID_METRICS_OUT",
                  "GROUNDGRID_PLANNING_SNAPSHOT_DIRECTORY", "GROUNDGRID_DEBUG_CONTROL")}}
    started = time.monotonic()

    def command(name, args, cwd):
        event = {"phase": name, "command": args, "cwd": str(cwd), "returncode": None}
        record["commands"].append(event)
        write_json(package / "run.json", record)
        event["returncode"] = execute(args, package / (name + ".log"), env, cwd=str(cwd))
        write_json(package / "run.json", record)
        return event["returncode"]

    try:
        if repo.parent.name != "src":
            raise ValueError("package-only requires the supported WORKSPACE/src/groundgrid checkout")
        workspace = repo.parent.parent
        build = workspace / "build" / "groundgrid"
        record["expected_xml_minimums"] = expected_package_xml(repo)
        rc = command("build", ["catkin", "build", "groundgrid", "--no-deps", "--cmake-args",
                               "-DCMAKE_BUILD_TYPE=Release", "-DCATKIN_ENABLE_TESTING=ON",
                               "-DPYTHON_EXECUTABLE=/usr/bin/python3",
                               "-DCATKIN_TEST_RESULTS_DIR=" + env["ROS_TEST_RESULTS_DIR"]], workspace)
        if (build / "CMakeCache.txt").is_file():
            shutil.copyfile(str(build / "CMakeCache.txt"), str(package / "CMakeCache.txt"))
        if rc:
            raise RuntimeError("Release build failed")
        cache = (package / "CMakeCache.txt").read_text(encoding="utf-8")
        values = dict(re.findall(r"^(\w+):\w+=(.*)$", cache, re.M))
        for key, value in {"CMAKE_BUILD_TYPE": "Release", "CATKIN_ENABLE_TESTING": "ON",
                           "PYTHON_EXECUTABLE": "/usr/bin/python3",
                           "CATKIN_TEST_RESULTS_DIR": env["ROS_TEST_RESULTS_DIR"]}.items():
            if values.get(key) != value:
                raise ValueError("unexpected CMake setting: " + key)
        label = ["-L", "^groundgrid_selfcheck$"]
        if command("ctest-inventory", ["ctest", "-N"] + label, build):
            raise RuntimeError("CTest inventory failed")
        if not validate_ctest_inventory((package / "ctest-inventory.log").read_text(), selfchecks):
            raise ValueError("CTest label must select exactly the eight C++ selfchecks, no ROS/Python")
        rc = command("ctest-cpp", ["ctest", "--output-on-failure"] + label, build)
        full_cpp = build / "Testing" / "Temporary" / "LastTest.log"
        if full_cpp.is_file():
            shutil.copyfile(str(full_cpp), str(package / "ctest-cpp-full.log"))
        if rc:
            raise RuntimeError("C++ preflight failed; ROS package test not started")
        cpp_output = (package / "ctest-cpp.log").read_text(errors="replace")
        passed_names = re.findall(r"Test\s+#\d+:\s+(\S+)[^\n]*\bPassed\b", cpp_output)
        if len(passed_names) != len(selfchecks) or set(passed_names) != set(selfchecks):
            raise RuntimeError("CTest did not report eight executed, passing selfchecks")
        if not full_cpp.is_file():
            raise RuntimeError("missing full C++ test output")
        test_rc = command("package-test", ["catkin", "test", "groundgrid", "--no-deps"], workspace)
        results_rc = command("catkin-test-results", ["catkin_test_results", env["ROS_TEST_RESULTS_DIR"]], build)
        errors, report = validate_run(package, run_id, commit, "mixed", "reachable_cost", "arcs", True)
        errors += validate_package_xml(env["ROS_TEST_RESULTS_DIR"], record["expected_xml_minimums"])
        record["archive_errors"] = sorted(set(errors))
        record["attempt_statistics"] = attempt_statistics(report)
        record["task_book_gaps"] = acceptance_gaps(report)
        record["passed"] = test_rc == 0 and results_rc == 0 and not errors
    except KeyboardInterrupt:
        record["interrupted"] = True
    except Exception as exc:
        record["error"] = type(exc).__name__ + ": " + str(exc)
        print(record["error"], flush=True)
    record["wall_duration_s"] = time.monotonic() - started
    write_json(package / "run.json", record)
    write_json(root / "suite.json", {"commit": commit, "kind": "package", "package": record,
                                     "passed": record["passed"], "formal_baseline": False})
    print("ARCHIVE=" + str(seal_archive(root)), flush=True)
    print("SUITE_RC=" + ("0" if record["passed"] else "1"), flush=True)
    return 0 if record["passed"] else 1
