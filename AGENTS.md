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

- Keep `min_point_distance < ground_support_radius == sensor_blind_radius`. The current
  lunar values are 1.2 m, 2.5 m and 2.5 m. `min_point_distance` rejects possible self
  returns; `ground_support_radius` anchors otherwise-empty ground across the LiDAR's direct
  ground-return blind disc; `sensor_blind_radius` defines the matching known-terrain
  exception downstream. Current usable returns and direct historical hazards must always
  take priority over support-plane fill. Changing only one can create either false slopes
  or a near-field region that is marked safe without corresponding terrain support.
- Treat the rover as its rectangular footprint for clearance and collision metrics; do not replace it with a circumscribed circle.
- Enumerate every grid square touched by the oriented footprint; do not substitute a
  rotated point lattice. On one map, increasing clearance must include all cells from
  the smaller footprint. Search and execution revalidation share this rasterisation.
- Collision checking must include the swept body during in-place rotations and transitions, not only endpoint poses.
- Normal search/rotation start-clearance ramps remain limited to the occupied start's
  first edge, with full clearance restored at its endpoint. Search must check the
  quantised/exported segment as well as the ideal primitive geometry.
- Dynamics root primitives additionally obey the execution gate's exported-sample
  rule: ordinary clearance is restored at the FIRST exported sample and held thereafter,
  while the original ideal-primitive/snap-clearance checks also remain mandatory. A long
  integration primitive is not permission to delay that existing execution requirement.
  If no compatible successor exists, report failure and let the bounded recovery ladder
  act; do not repeatedly publish a route that execution must immediately withdraw.
- Dynamics successors start at the preceding primitive's actual integrated endpoint.
  Cell/heading quantisation is for search indexing, not an extra physical motion back
  to a grid centre. Reconstruct from the same stored poses and check snap bounds and
  clearance at the actual exported endpoint; a cell-centre certificate cannot certify
  another continuous arrival in that cell. Exact-goal primitive prefixes use existing
  checked integration samples and the same shared budget; retain the full edge too.
- The user-authorised BackOut exception may restore clearance over ONE bounded retreat
  built from actual stamped localisation history, never from an old planned route. Its
  cumulative corner-motion budget is `recovery_backout_distance` (currently 1 m); its
  frozen clearance schedule cannot restart on a map update or republish. BackOut restores
  the ordinary `trajectory_clearance`; the larger `goal_snap_clearance` belongs to a later
  snapped nominal route and must not leak from a failed snap attempt into the escape gate.
  Preparation checks the whole retreat. Execution checks actual motion since the previous
  localisation sample, the current-to-route connector, and the whole unexecuted suffix;
  newly perceived hazards solely on an already traversed prefix do not revoke safe motion.
  The body and every current/future sweep always reject known hazards; the endpoint must
  have full ordinary clearance.
  Execution completion additionally requires full clearance at the actual rover pose.
  New goals revoke retreat execution with the exact stop handshake but retain independent
  observed history. Recovery completion is not mission completion or recovery confirmation.
  After a successful retreat, allow one final Relax window using `recovery_step_timeout`,
  starting after the stop acknowledgement and fresh pose. Unless a goal route satisfies
  the existing confirmation count in that window, advance to Abort; never restart the
  Rotate/BackOut ladder merely because retreat clearance was restored.
- Retreat history is local and bounded, not a persistent obstacle map. Discontinuous,
  wrongly framed, stale or invalid localisation must not create a synthetic connection.
  Software execution gates are sampled checks, not physical braking-distance guarantees.
- Unknown terrain is normally invalid. The sensor blind disc and the goal-snap margin are narrowly scoped exceptions and must not silently relax known hazards.
- A recovery rotate/back-out manoeuvre is not a successful nominal plan. Keep recovery, abort, planning-success, reach, and obstacle-avoidance metrics distinct.
- The map is a 60 m rolling window centred on the rover. Fixed world goals can legitimately leave the map as the rover moves.
- Freshness timeouts must have margin over the measured GroundGrid/map publication period; do not set them from a nominal rate alone.
- `/lunar_planner/trajectory` is the follower's only control input. Its path and twists
  must be equal-length, finite, and published atomically; the legacy path/profile topics
  are compatibility and visualisation outputs, not control inputs.
- A trajectory twist is desired effective body motion before slip compensation. Apply
  terrain scaling in the planner and `inverseCommand()` once in the follower.
- A runtime tracking-phase fault stops execution without forgetting progress for that
  geometry. Republishing the same retained route must not restart completed phases.
  Empty/invalid replacements and new goals still revoke it; a different route resets it.
- Mix unsaturated angular feedback with planned feed-forward before the common angular
  command limit. Pre-clipping feedback halves available correction at the 0.5 blend;
  raw diagnostic `feedback_w` may exceed the command limit, but output commands may not.
- The simulator must capture a cloud's pose and timestamp together under the dynamics
  lock. Ray-cast computation does not advance the measurement time; stamping old geometry
  with the latest TF time corrupts the perceived map while moving.
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
- Build and run `backout_recovery_selfcheck` for observed-history/retreat changes; it covers
  clearance scheduling, lifetime, discontinuities and production follower/stop-barrier
  interaction on synthetic maps. ROS callbacks and real braking still require runtime tests.
- For planning-core changes, run `planning_core_selfcheck` and
  `planning_snapshot_selfcheck` locally, plus `planning_grid_parity_selfcheck` against
  Ubuntu's installed grid_map. Snapshot replay calls production search, not an imitation.
  Fixed-expansion replay is for deterministic comparisons, not latency certification.
- Count planning time from unique goal-correlated `planning_attempt` events. Repeated
  diagnostic `plan_ms` values are not additional searches. Snapshot writer drops/failures
  invalidate completeness of the recorded experiment and must remain visible.
- Use isolated run_planner_experiments.py archives with a new output directory for ROS
  baselines. Never reuse JSON from another run/commit or use rostest --text as a formal
  XML verdict. Keep every repeated run, including failures; a primitive-mode fallback is
  not a dynamics-mode pass. Keep legacy_nearest as the capture default until the real
  snapshot parity/replay gate has passed.
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
