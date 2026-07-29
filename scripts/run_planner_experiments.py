#!/usr/bin/env python3
"""Run the planner metric suite once per terrain class and collect the results.

One `rostest` invocation exercises one scenario, so the task book's "每种场景重复≥10次"
is two loops: `--n-trials` inside the test, the scenario list out here. A scenario that
fails is recorded and the sweep continues -- stopping on the first failure would throw
away the four terrain classes that did work, and the marginal classes are expected to be
the interesting ones.

Runtime is the reason this is a script and not a single rostest: at --n-trials 10 the
full sweep is roughly 1.5-2 hours, so CI runs 3 and a review run is started deliberately.

    rosrun groundgrid run_planner_experiments.py --n-trials 3
    rosrun groundgrid summarize_planner_metrics.py
"""

import argparse
import os
import subprocess
import sys
import time

SCENARIOS = ["mixed", "flat", "dense", "slope", "negative"]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--n-trials", type=int, default=3,
                        help="repetitions of the goal tour per scenario (task book asks 10)")
    parser.add_argument("--scenarios", nargs="+", default=SCENARIOS, choices=SCENARIOS,
                        help="subset to run; defaults to all five")
    parser.add_argument("--out-dir", default=os.path.expanduser("~/.ros"),
                        help="where the per-scenario JSON lands")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    results = []
    for scenario in args.scenarios:
        out = os.path.join(args.out_dir, "planner_metrics_%s.json" % scenario)
        cmd = ["rostest", "groundgrid", "lunar_pipeline.test",
               "scenario:=%s" % scenario,
               "n_trials:=%d" % args.n_trials,
               "metrics_out:=%s" % out]
        print("\n=== %s ===\n$ %s" % (scenario, " ".join(cmd)), flush=True)
        started = time.time()
        code = subprocess.call(cmd)
        elapsed = time.time() - started
        results.append((scenario, code, elapsed, out if os.path.exists(out) else None))
        print("--- %s finished rc=%d in %.0f s" % (scenario, code, elapsed), flush=True)

    print("\n=== sweep summary ===")
    for scenario, code, elapsed, out in results:
        print("  %-9s rc=%-3d %6.0f s  %s"
              % (scenario, code, elapsed, out or "(no metrics file)"))
    failed = [s for s, code, _e, _o in results if code != 0]
    if failed:
        print("\nfailed: %s" % ", ".join(failed))
        print("logs: ~/.ros/log/latest/lunar_pipeline_test*.log")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
