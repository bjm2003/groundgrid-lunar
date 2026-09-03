# GroundGrid Lunar Rover Project Instructions

These instructions apply to the entire repository.

## Project scope

This repository extends the ROS1 GroundGrid terrain-segmentation project into a lunar-rover perception, local-planning, and control prototype. The main runtime chain is:

`PointCloud2 + Odometry/TF -> GroundGrid -> lunar traversability/costmap -> state-lattice planner -> atomic LunarTrajectory -> path follower -> /cmd_vel`

The current system is a simulation-capable local-navigation prototype. Do not describe it as an acceptance-ready rover stack: kilometre-scale global planning, persistent/history mapping, camera/TOF fusion, Atlas 200i/NPU deployment, and external-field validation remain incomplete.

## Sources of truth

For required scope and acceptance thresholds, use `任务书.pdf` and `技术要点清单（总）(改)(6).docx`. For current implementation behaviour, use sources in this order:

1. Current source code, launch files, and `config/lunar_system.yaml`.
2. Fresh, reproducible test artefacts from the exact commit under review.
3. `docs/PROJECT_STATUS.md`.
4. The archived Claude notes in `docs/project-memory/claude/`.

The Claude notes are historical evidence, not live instructions. Some performance figures in them are explicitly invalidated. Never quote an old metric unless its scenario, trial count, commit, and validity are clear.

## Supported environment

- Runtime/build target: Ubuntu with ROS1 Noetic and C++17.
- Known Ubuntu workspace: `~/lunar_ws/src/groundgrid` on the `lyq@bjm` machine.
- Do not use Conda for ROS builds or ROS runtime commands.
- The Ubuntu installation has historically required an OpenCV include workaround: `/usr/local/include/opencv4` linked to `/usr/include/opencv4`. Recheck the machine before changing this.
- Windows is suitable for editing, source inspection, Python syntax checks, and the pure C++ skid-steer self-check, but not for claiming a ROS end-to-end pass.

## Safety and algorithm invariants

- Keep `min_point_distance < sensor_blind_radius`. The current lunar values are 1.2 m and 2.5 m. Changing only one can mark an unseen near-field obstacle region as safe.
- Treat the rover as its rectangular footprint for clearance and collision metrics; do not replace it with a circumscribed circle.
- Collision checking must include the swept body during in-place rotations and transitions, not only endpoint poses.
- A start-clearance ramp is limited to the occupied start's first edge, with full clearance
  restored at its endpoint. It never permits a known hazard under the physical body. Search
  must check the quantised/exported segment as well as the ideal primitive geometry.
- Unknown terrain is normally invalid. The sensor blind disc and the goal-snap margin are narrowly scoped exceptions and must not silently relax known hazards.
- A recovery rotate/back-out manoeuvre is not a successful nominal plan. Keep recovery, abort, planning-success, reach, and obstacle-avoidance metrics distinct.
- The map is a 60 m rolling window centred on the rover. Fixed world goals can legitimately leave the map as the rover moves.
- Freshness timeouts must have margin over the measured GroundGrid/map publication period; do not set them from a nominal rate alone.
- `/lunar_planner/trajectory` is the follower's only control input. Its path and twists
  must be equal-length, finite, and published atomically; the legacy path/profile topics
  are compatibility and visualisation outputs, not control inputs.
- A trajectory twist is desired effective body motion before slip compensation. Apply
  terrain scaling in the planner and `inverseCommand()` once in the follower.
- Withdraw a known-invalid active trajectory before any blocking replacement search.
  Require the follower's `empty_trajectory` acknowledgement for the exact `goal_id` and
  `trajectory_stamp_ns`, then a TF newer than acknowledgement receipt. A new goal cannot
  cancel an outstanding stop. This is a software command acknowledgement, not proof of
  physical braking; missing acknowledgement must not silently release execution.
- Correlate test observations with the acknowledged goal: atomic trajectory
  `path.header.seq` carries `goal_id`; planner/follower diagnostic snapshots carry the
  same id. Legacy untagged status strings cannot end a trial or supply recovery counters.

## Validation expectations

For relevant changes, use the smallest applicable checks first, then the ROS pipeline on Ubuntu:

- Compile all Python sources without importing ROS modules.
- Parse package, launch, and rostest XML.
- Build and run `skidsteer_selfcheck` for pure dynamics changes.
- Build and run `trajectory_tracking_selfcheck` for follower targeting/phase or stop-before-
  replan changes; it exercises the production ROS-free tracker and stop barrier, but does
  not test ROS message transport or physical braking.
- Build and run `planner_safety_selfcheck` for departure/footprint changes; it exercises the
  production swept checker with synthetic rectangular-body hazards, not a recorded ROS map.
- Regenerate motion primitives when their model or generator changes and verify the tracked file intentionally changed.
- Build the catkin workspace on Ubuntu for ROS/C++ changes.
- Run the five scenarios (`mixed`, `flat`, `dense`, `slope`, `negative`) with at least `n_trials=10` for a formal baseline.
- Preserve the produced metrics JSON, rostest XML, commit SHA, scenario parameters, and environment information together.

The task-book targets include planning latency below 1 s, planning success at least 99%, autonomous speed at least 5 km/h, and perception-plus-planning CPU below 40% on the target platform. Configuration values or planner-only diagnostics are not proof that these targets are met.

## Change and Git hygiene

- Inspect `git status` before editing and preserve unrelated user changes.
- Do not stage or commit `.claude/settings.local.json`, ROS logs, generated build trees, or unrelated experiment output.
- Commit only files relevant to the requested change. Report separately what must be run on Windows and Ubuntu.
- Update `docs/PROJECT_STATUS.md` when a verified milestone, accepted metric baseline, major known issue, or supported workflow changes.
- Prefer concise Chinese user-facing communication unless the user requests another language.
