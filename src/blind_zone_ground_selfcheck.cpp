#include "groundgrid/BlindZoneGround.h"

#include <cmath>
#include <cstdio>
#include <limits>

using groundgrid::BlindZoneSupportPlane;

namespace {
int checks=0, failures=0;
void check(bool ok,const char* label) {
    ++checks;
    std::printf("[%s] %s\n",ok ? "PASS" : "FAIL",label);
    if(!ok) ++failures;
}
bool near(double a,double b,double tolerance=1e-9) {
    return std::abs(a-b)<=tolerance;
}
}

int main() {
    double height=0.0;
    const double nan=std::numeric_limits<double>::quiet_NaN();
    const auto flat=BlindZoneSupportPlane::fromPose(2.0,-3.0,.7,0,0,0,1);
    check(flat.valid(),"finite unit pose defines a support plane");
    check(flat.heightForUnmeasuredCell(2.6,-2.6,2,-3,1.2,0,nan,height) && near(height,.7),
          "unmeasured self-mask cell uses the flat base support height");
    check(flat.heightForUnmeasuredCell(2.6,-2.6,2,-3,1.2,0,1.45,height) && near(height,1.45),
          "direct historical terrain wins over the support-plane fallback");
    check(!flat.heightForUnmeasuredCell(3.3,-3.0,2,-3,1.2,0,nan,height),
          "cell outside the self-mask is not overwritten");
    check(!flat.heightForUnmeasuredCell(2.2,-3.0,2,-3,1.2,1,nan,height),
          "cell with a usable current point keeps measured interpolation");
    check(flat.heightForUnmeasuredCell(3.2,-3.0,2,-3,1.2,0,nan,height),
          "self-mask boundary is included despite floating-point roundoff");
    check(flat.heightForUnmeasuredCell(2.5,-3.0,2.5,-3.0,1.2,0,nan,height) && near(height,.7),
          "sensor-offset mask still evaluates height on the base support plane");
    check(flat.heightForUnmeasuredCell(3.8,-3.0,2,-3,2.5,0,nan,height) && near(height,.7),
          "empty ground-return blind cell beyond the self-mask uses vehicle support");
    check(flat.heightForUnmeasuredCell(3.8,-3.0,2,-3,2.5,0,.63,height) && near(height,.63),
          "direct history still wins in the larger ground-return blind disc");
    check(!flat.heightForUnmeasuredCell(3.8,-3.0,2,-3,2.5,1,nan,height) &&
          !flat.heightForUnmeasuredCell(4.6,-3.0,2,-3,2.5,0,nan,height),
          "current returns and cells outside the ground-return blind disc are unchanged");

    const double pitch=.2;
    const auto tilted=BlindZoneSupportPlane::fromPose(
        -1.2,-3.25,-.05,0,std::sin(pitch/2),0,std::cos(pitch/2));
    check(tilted.heightForUnmeasuredCell(-.7,-3.25,-1.2,-3.25,1.2,0,nan,height) &&
          near(height,-.05-std::tan(pitch)*.5),
          "pitch projects the base xy support plane into map height");
    const double yaw=.9;
    const auto yawed=BlindZoneSupportPlane::fromPose(
        0,0,.4,0,0,std::sin(yaw/2),std::cos(yaw/2));
    check(yawed.heightForUnmeasuredCell(.3,.4,0,0,1.2,0,nan,height) && near(height,.4),
          "yaw alone does not create a false terrain gradient");

    check(!BlindZoneSupportPlane::fromPose(0,0,0,0,0,0,0).valid() &&
          !BlindZoneSupportPlane::fromPose(nan,0,0,0,0,0,1).valid(),
          "invalid pose or quaternion fails closed");
    check(!flat.heightForUnmeasuredCell(nan,0,0,0,1.2,0,nan,height) &&
          !flat.heightForUnmeasuredCell(0,0,0,0,-1,0,nan,height) &&
          !flat.heightForUnmeasuredCell(0,0,0,0,1.2,nan,nan,height),
          "invalid cell inputs are never anchored");

    std::printf("blind_zone_ground_selfcheck: %d checks, %d failures\n",checks,failures);
    return failures ? 1 : 0;
}
