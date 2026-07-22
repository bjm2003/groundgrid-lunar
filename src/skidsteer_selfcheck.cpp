// Standalone sanity check for the SkidSteerModel (no ROS). Returns non-zero on failure.
#include "groundgrid/SkidSteerModel.h"

#include <cmath>
#include <cstdio>

using groundgrid::Pose2D;
using groundgrid::SkidSteerModel;
using groundgrid::SkidSteerParams;

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

    std::printf("skidsteer_selfcheck: %d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
