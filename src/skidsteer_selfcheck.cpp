// Standalone sanity check for the SkidSteerModel (no ROS). Returns non-zero on failure.
#include "groundgrid/SkidSteerModel.h"
#include "groundgrid/TrajectoryControl.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

using groundgrid::Pose2D;
using groundgrid::SkidSteerModel;
using groundgrid::SkidSteerParams;
using groundgrid::TrajectoryControlParams;
using groundgrid::blendTrajectoryCommand;
using groundgrid::geometricAngularFeedback;
using groundgrid::inPlaceRotationCompleted;
using groundgrid::missionGoalReached;
using groundgrid::requiresInPlaceRotationTracking;
using groundgrid::requiresTerminalWaypointTracking;
using groundgrid::retainedTrajectoryFallbackAllowed;
using groundgrid::selectTrajectoryCommandIndex;
using groundgrid::stableTrajectoryReuseAllowed;
using groundgrid::terminalTrajectoryReuseAllowed;
using groundgrid::trajectoryEndpointReached;

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++failures;
}

// Reference ideal differential-drive integration for comparison.
Pose2D idealStep(Pose2D p, double v, double w, double dt) {
    p.x += v * std::cos(p.yaw) * dt;
    p.y += v * std::sin(p.yaw) * dt;
    p.yaw += w * dt;
    return p;
}

} // namespace

int main() {
    // 1) Zero-slip params must reproduce ideal differential drive.
    {
        SkidSteerParams zero;  // defaults: alpha_v=alpha_w=1, x_icr=0, gains=0
        SkidSteerModel model(zero);
        Pose2D p{1.0, -2.0, 0.3};
        const double v = 0.8, w = 0.4, dt = 0.05;
        Pose2D got = model.integrate(p, v, w, dt);
        Pose2D ref = idealStep(p, v, w, dt);
        const bool ok = std::abs(got.x - ref.x) < 1e-9 &&
                        std::abs(got.y - ref.y) < 1e-9 &&
                        std::abs(got.yaw - ref.yaw) < 1e-9;
        check(ok, "zero-slip reduces to ideal differential drive");
    }

    // 2) Non-zero ICR offset produces lateral drift while turning.
    {
        SkidSteerParams p;
        p.x_icr = 0.3;
        SkidSteerModel model(p);
        auto tw = model.effectiveTwist(0.0, 0.5);  // pure rotation
        check(std::abs(tw.vy + p.x_icr * (p.alpha_w * 0.5)) < 1e-9,
              "ICR offset couples yaw rate into lateral velocity");
    }

    // 3) Uphill grade reduces forward progress; downslope lateral gradient adds side drift.
    {
        SkidSteerParams p;
        p.slope_grade_gain = 0.5;
        p.slope_slip_gain = 0.4;
        SkidSteerModel model(p);
        auto flat = model.effectiveTwist(1.0, 0.0, 0.0, 0.0);
        auto uphill = model.effectiveTwist(1.0, 0.0, 0.5, 0.0);
        check(uphill.vx < flat.vx, "uphill gradient slows forward velocity");
        auto side = model.effectiveTwist(1.0, 0.0, 0.0, 0.5);
        check(std::abs(side.vy) > 0.0, "lateral gradient induces side drift");
    }

    // 4) inverseCommand recovers the command under linear slip efficiencies.
    {
        SkidSteerParams p;
        p.alpha_v = 0.8;
        p.alpha_w = 0.9;
        SkidSteerModel model(p);
        double vc = 0.0, wc = 0.0;
        model.inverseCommand(0.8 * 1.0, 0.9 * 0.4, vc, wc);
        check(std::abs(vc - 1.0) < 1e-9 && std::abs(wc - 0.4) < 1e-9,
              "inverseCommand recovers commanded (v, w)");
    }

    // 5) Direction-frame pure pursuit must reinforce, not cancel, reverse feed-forward.
    {
        double forward_w = 0.0, reverse_w = 0.0;
        const bool forward_ok = geometricAngularFeedback(0.6, 0.2, 0.8, 0.8, forward_w);
        const bool reverse_ok = geometricAngularFeedback(-0.6, 0.2, 0.8, 0.8, reverse_w);
        check(forward_ok && reverse_ok && forward_w > 0.0 &&
              std::abs(forward_w-reverse_w) < 1e-12,
              "reverse geometric feedback preserves planner yaw-rate sign");
        check(!geometricAngularFeedback(-0.6, 0.2, -0.1, 0.8, reverse_w),
              "geometric feedback rejects invalid distance");
        for(int direction : {-1,1}) {
            double feedback = 0.0, v = 0.0, w = 0.0;
            groundgrid::TrajectoryControlParams limits;
            const bool ok = geometricAngularFeedback(direction*.533484,-1.2,.3,.8,feedback);
            check(ok && feedback < -1.6 &&
                  groundgrid::blendTrajectoryCommand(direction*.533484,0,feedback,limits,v,w) &&
                  std::abs(v-direction*.533484)<1e-12 && std::abs(w+.8)<1e-12,
                  direction < 0 ? "reverse correction saturates only after the 50/50 blend" :
                                  "forward correction saturates only after the 50/50 blend");
        }
        double feedback = 0.0, v = 0.0, w = 0.0;
        groundgrid::TrajectoryControlParams limits;
        check(groundgrid::blendTrajectoryCommand(.5,.6,-1.4,limits,v,w) &&
              std::abs(w+.4)<1e-12,
              "opposing feedforward and feedback cancel before command saturation");
        check(!geometricAngularFeedback(std::numeric_limits<double>::max(),1.0,.1,.8,feedback),
              "overflowing geometric demand is rejected instead of disguised by clipping");
    }

    // 6) Lookahead stops at unfinished co-located rotations, then advances when aligned.
    {
        check(requiresInPlaceRotationTracking(0.0, 0.3, 0.17),
              "lookahead preserves unfinished in-place rotation");
        check(!requiresInPlaceRotationTracking(0.0, 0.1, 0.17) &&
              !requiresInPlaceRotationTracking(0.45, 0.3, 0.17),
              "lookahead advances after rotation or along translation");
    }

    // 7) Completed rotation boundaries are monotonic within one trajectory.
    {
        check(inPlaceRotationCompleted(0.0, 0.1, 0.17) &&
              !inPlaceRotationCompleted(0.0, 0.3, 0.17) &&
              !inPlaceRotationCompleted(0.45, 0.1, 0.17),
              "completed in-place rotations create a monotonic path boundary");
        check(!inPlaceRotationCompleted(NAN, 0.1, 0.17),
              "rotation completion rejects invalid input");
    }

    // 8) The final waypoint is exposed only after the penultimate one is acquired.
    {
        check(requiresTerminalWaypointTracking(true, 0.25, 0.20),
              "lookahead cannot skip a distant penultimate waypoint");
        check(!requiresTerminalWaypointTracking(true, 0.19, 0.20) &&
              !requiresTerminalWaypointTracking(false, 0.8, 0.20),
              "lookahead releases an acquired terminal waypoint");
    }

    // 9) Lookahead must not apply a future zero-speed boundary before reaching it.
    {
        const std::vector<double> speeds{0.0, 0.6, 0.0, 0.0};
        std::size_t command = 99;
        const auto speed_at = [&speeds](std::size_t i) { return speeds[i]; };
        check(selectTrajectoryCommandIndex(3, speeds.size(), 0.8, 0.2,
                                           speed_at, command) && command == 1,
              "distant zero-speed boundary retains translation");
        check(selectTrajectoryCommandIndex(3, speeds.size(), 0.1, 0.2,
                                           speed_at, command) && command == 3,
              "acquired zero-speed boundary is released");
        check(selectTrajectoryCommandIndex(3, speeds.size(), 0.24, 0.25,
                                           speed_at, command) && command == 3,
              "goal position tolerance releases translation for terminal yaw capture");
        const std::vector<double> rotation_only{0.0, 0.0};
        check(!selectTrajectoryCommandIndex(
                  1, rotation_only.size(), 0.8, 0.2,
                  [&rotation_only](std::size_t i) { return rotation_only[i]; }, command),
              "distant pose without translation is rejected");
    }

    // 10) Terminal trajectory locking is allowed only for a fresh, valid nominal remainder.
    {
        check(terminalTrajectoryReuseAllowed(0.6, 0.8, true, true),
              "valid terminal trajectory can be retained");
        check(!terminalTrajectoryReuseAllowed(0.9, 0.8, true, true) &&
              !terminalTrajectoryReuseAllowed(0.6, 0.8, false, true) &&
              !terminalTrajectoryReuseAllowed(0.6, 0.8, true, false),
              "terminal reuse never bypasses replanning or recovery safety gates");
        check(!terminalTrajectoryReuseAllowed(NAN, 0.8, true, true),
              "terminal reuse rejects non-finite input");
    }

    // 11) A transient search failure may keep a revalidated nominal route, never an unsafe
    // route or a route while recovery owns the controller.
    {
        check(retainedTrajectoryFallbackAllowed(true, true),
              "valid retained trajectory survives a transient replan failure");
        check(!retainedTrajectoryFallbackAllowed(false, true) &&
              !retainedTrajectoryFallbackAllowed(true, false),
              "retained fallback never bypasses recovery or map validation");
    }

    // 12) Snapped routes remain stable before the terminal region, but neither snapped nor
    // terminal reuse may hide no-progress, recovery ownership, or a failed map check.
    {
        check(stableTrajectoryReuseAllowed(true, false, false, true, true) &&
              stableTrajectoryReuseAllowed(false, true, false, true, true),
              "snapped and terminal routes remain stable while making progress");
        check(!stableTrajectoryReuseAllowed(true, false, true, true, true) &&
              !stableTrajectoryReuseAllowed(true, false, false, false, true) &&
              !stableTrajectoryReuseAllowed(true, false, false, true, false),
              "stable reuse preserves progress, recovery, and map safety gates");
    }

    // 13) Planner and follower agree on when the retained endpoint is complete.
    {
        check(trajectoryEndpointReached(0.20, 0.10, 0.25, 0.17),
              "completed endpoint retires the planner goal");
        check(!trajectoryEndpointReached(0.25, 0.10, 0.25, 0.17) &&
              !trajectoryEndpointReached(0.20, 0.17, 0.25, 0.17) &&
              !trajectoryEndpointReached(NAN, 0.10, 0.25, 0.17),
              "endpoint completion preserves strict tolerances and rejects invalid input");
    }

    // 14) Mission completion excludes recovery and unfinished ordinary goals.
    {
        check(missionGoalReached(true, true, false, 0.40, 0.50) &&
              missionGoalReached(true, true, true, 1.20, 0.50),
              "ordinary and snapped routes use their intended mission completion gates");
        check(!missionGoalReached(true, false, false, 0.10, 0.50) &&
              !missionGoalReached(true, true, false, 0.60, 0.50) &&
              !missionGoalReached(false, true, true, 1.20, 0.50),
              "recovery and intermediate endpoints cannot complete the mission");
    }

    // 15) Trajectory control keeps planned v authoritative and actually consumes planned w.
    {
        TrajectoryControlParams p;
        p.angular_feedforward_weight = 0.5;
        p.max_linear_speed = 1.0;
        p.max_angular_speed = 0.8;
        double v = 0.0, w = 0.0;
        const bool ok = blendTrajectoryCommand(0.6, 0.4, -0.2, p, v, w);
        check(ok && std::abs(v-0.6) < 1e-9 && std::abs(w-0.1) < 1e-9,
              "trajectory controller blends planned angular feed-forward");

        check(blendTrajectoryCommand(-0.5, -0.3, -0.1, p, v, w) && v < 0.0 && w < 0.0,
              "trajectory controller preserves reverse command signs");
        check(blendTrajectoryCommand(0.0, 0.6, 0.2, p, v, w) &&
              std::abs(v) < 1e-9 && w > 0.0,
              "trajectory controller supports in-place rotation");
        check(blendTrajectoryCommand(3.0, 3.0, 3.0, p, v, w) &&
              std::abs(v-1.0) < 1e-9 && std::abs(w-0.8) < 1e-9,
              "trajectory controller clamps the command envelope");
        check(!blendTrajectoryCommand(NAN, 0.0, 0.0, p, v, w),
              "trajectory controller rejects non-finite input");
    }

    std::printf("skidsteer_selfcheck: %d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
