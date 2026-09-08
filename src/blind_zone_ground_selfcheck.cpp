#include "groundgrid/BlindZoneGround.h"
#include "groundgrid/TerrainGradient.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <array>
#include <vector>

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

    // Real bf17ff5 rejection patch at (-5.325,-5.875), stamp
    // 1788795371148578643: a retained direct height was overwritten by smoothing.
    const std::array<double,9> measured_ground={.23663191497325897,.21908679604530334,
        .2216925024986267,.5796235203742981,.1866529881954193,.5538346171379089,
        .42218077182769775,.474795937538147,.32975509762763977};
    const std::array<double,9> kernel={1./16,1./8,1./16,1./8,1./4,1./8,1./16,1./8,1./16};
    double smooth=0;
    for(size_t i=0;i<9;++i) smooth+=kernel[i]*measured_ground[i];
    const double confidence=std::pow(.010844792239367962,2.0); // recorded blend exponent
    auto corrected=measured_ground;
    corrected[4]=confidence*measured_ground[4]+(1-confidence)*smooth;
    check(near(corrected[4],.3507028818130493,1e-6),
          "recorded low-confidence smoothing reproduces 0.164m historical-height corruption");
    const std::vector<int> protected_cells={4};
    groundgrid::restoreMeasuredSupport(protected_cells,
        [&](int i) { return measured_ground[i]; },[&](int i) -> double& { return corrected[i]; });
    check(corrected==measured_ground,"protected historical support survives correction exactly");
    corrected[0]=.9; corrected[4]=1.2;
    groundgrid::restoreMeasuredSupport(protected_cells,
        [&](int i) { return measured_ground[i]; },[&](int i) -> double& { return corrected[i]; });
    check(corrected[0]==.9 && corrected[4]==measured_ground[4],
          "later correction passes preserve selected history without locking unmeasured neighbours");
    const auto saved=corrected;
    groundgrid::restoreMeasuredSupport(std::vector<int>{},
        [&](int i) { return measured_ground[i]; },[&](int i) -> double& { return corrected[i]; });
    check(corrected==saved,"empty selection leaves ordinary smoothing unchanged");
    groundgrid::restoreMeasuredSupport(protected_cells,
        [&](int i) { return i==4 ? 1.6 : measured_ground[i]; },
        [&](int i) -> double& { return corrected[i]; });
    check(corrected[4]==1.6 && corrected[0]==saved[0],
          "elevated historical evidence is preserved rather than flattened");
    for(double grade : {0.0, .2, -.2, 1.0, -1.0}) {
        const auto g=groundgrid::estimateTerrainGradient(.15,
            [&](int i,int j){return 12.0+grade*.15*i+.3*.15*j;});
        check(near(g.x,grade,1e-6) && near(g.y,.3,1e-6),
              "plane gradient exact, including steep and reversed slopes");
    }
    for(double sign : {-1.0,1.0}) {
        const auto g=groundgrid::estimateTerrainGradient(.15,
            [&](int i,int){return i>=0 ? sign*.5 : 0.0;});
        check(std::abs(g.x)>1.0 && near(g.y,0),
              "positive and negative half-metre steps remain steep");
    }
    const std::array<double,9> patch{{
        -.4477697014808655,-.4177834987640381,-.4198524057865143,
        -.4948926866054535,-.45798459649086,-.45544639229774475,
        -.506034255027771,-.5063286423683167,-.4795982837677002}};
    const auto patch_before=patch;
    const auto fitted=groundgrid::estimateTerrainGradient(.15000000596046448,
        [&](int i,int j){return patch[(i+1)*3+j+1];});
    check(near(fitted.x,-.22950619,1e-6) && near(fitted.y,.10422173,1e-6),
          "recorded f18ce9b slope patch uses all nine samples");
    check(patch==patch_before,"gradient estimation leaves height and hazard evidence untouched");
    for(int missing=0;missing<9;++missing) {
        const auto g=groundgrid::estimateTerrainGradient(.15,[&](int i,int j) {
            return (i+1)*3+j+1==missing ? std::numeric_limits<double>::quiet_NaN() : 1.0;
        });
        check(!std::isfinite(g.x) && !std::isfinite(g.y),"unknown sample is not a safe plane");
    }
    const auto invalid=groundgrid::estimateTerrainGradient(0,[](int,int){return 0.0;});
    check(!std::isfinite(invalid.x),"invalid resolution rejected");
    for(double grade : {-.8,0.0,.8}) {
        const auto g=groundgrid::estimateSupportedTerrainGradient(.15,
            [&](int i,int j){return 3+grade*.15*i-.4*.15*j;},true);
        check(near(g.x,grade,1e-6) && near(g.y,-.4,1e-6),
              "wide plane retains exact steep slope in either direction");
    }
    for(double sign : {-1.0,1.0}) {
        const auto g=groundgrid::estimateSupportedTerrainGradient(.15,
            [&](int i,int){return i>=0 ? sign*.5 : 0.0;},true);
        check(std::abs(g.x)>.9,"wide fit keeps positive and negative half-metre steps steep");
    }
    const auto fallback=groundgrid::estimateSupportedTerrainGradient(.15,[](int i,int j) {
        return std::abs(i)==2 || std::abs(j)==2 ? NAN : .15*i;
    },true);
    check(near(fallback.x,1,1e-6),"missing outer ring falls back without fabricating heights");
    const auto inner_missing=groundgrid::estimateSupportedTerrainGradient(.15,[](int i,int j) {
        return i==0 && j==0 ? NAN : .15*i;
    },true);
    check(!std::isfinite(inner_missing.x),"wide support does not bridge an unknown inner cell");
    bool accessed_outer=false;
    groundgrid::estimateSupportedTerrainGradient(.15,[&](int i,int j) {
        if(std::abs(i)>1 || std::abs(j)>1) accessed_outer=true;
        return .15*i;
    },false);
    check(!accessed_outer,"map border never reads unavailable outer cells");
    const std::array<double,25> wide_patch{{
        -.00475306,-.00179887,-.01514905,-.00753199,-.02985446,
        .03234244,.02838844,.03067188,.01349804,.03578265,
        .08888233,.06240825,.05576667,.06089792,.03991091,
        .10552239,.10329417,.10927762,.11642379,.07999519,
        .16129328,.13185880,.12520359,.11704297,.12656976}};
    const auto wide=groundgrid::estimateSupportedTerrainGradient(.15,
        [&](int i,int j){return wide_patch[(i+2)*5+j+2];},true);
    check(near(wide.x,.24212551,1e-6) && near(wide.y,-.03807823,1e-6),
          "recorded a995462 stopping patch reduces single-cell sampling bias");
    std::printf("blind_zone_ground_selfcheck: %d checks, %d failures\n",checks,failures);
    return failures ? 1 : 0;
}
