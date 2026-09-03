// Synthetic geometry tests of the production swept checker. These are not a replay of
// the rolling perception map; the logged pose only anchors the no-successor fixture.
#include "groundgrid/SweptFootprint.h"

#include <cstdio>
#include <limits>
#include <vector>

using namespace groundgrid;

namespace {
int failures = 0, checks = 0;
const double pi = std::acos(-1.0);
const double radius = std::hypot(.9,.75);
void check(bool ok, const char* label) {
    ++checks;
    std::printf("[%s] %s\n",ok ? "PASS" : "FAIL",label);
    if(!ok) ++failures;
}
struct Hazard { double x,y,r; };
struct RectangleMap {
    std::vector<Hazard> hazards;
    bool operator()(const Pose2D& p, double margin, bool, float& cost) const {
        cost = 10.0f;
        // Exact circle-to-oriented-RECTANGLE distance, not a circumscribed rover circle.
        for(const auto& h : hazards) {
            const double dx=h.x-p.x, dy=h.y-p.y;
            const double lx=std::abs(std::cos(p.yaw)*dx+std::sin(p.yaw)*dy);
            const double ly=std::abs(-std::sin(p.yaw)*dx+std::cos(p.yaw)*dy);
            if(std::hypot(std::max(0.0,lx-.9-margin),std::max(0.0,ly-.75-margin)) <= h.r)
                return false;
        }
        return true;
    }
};

// Old ideal-arc sample policy, only to establish the regression: the full 0.50 m band
// was demanded at every departure sample, although the root only required its body.
bool legacyEdge(const Pose2D& p,int direction,int turn,const RectangleMap& map) {
    const double dyaw=turn*pi/8;
    const int n=direction ? std::max(1,int(std::ceil(std::abs(dyaw)/.2)))
                         : std::max(1,int(std::ceil(std::abs(dyaw)*radius/.15)));
    float cost;
    for(int i=1;i<=n;++i) {
        const double q=double(i)/n;
        if(!map({p.x+direction*.45*q*std::cos(p.yaw+q*dyaw*.5),
                 p.y+direction*.45*q*std::sin(p.yaw+q*dyaw*.5),p.yaw+q*dyaw},
                 .50,false,cost)) return false;
    }
    return true;
}
}

int main() {
    float cost;
    const RectangleMap rock{{{0,-2,.65}}};
    const Pose2D stuck{-1.991314028154504,-2.89871102168998,-pi/8};
    check(rock(stuck,0,true,cost) && !rock(stuck,.5,true,cost),
          "fixture start body is clear but full snap band is unavailable");
    int old_successors=0,new_successors=0,full_band_successors=0;
    for(int direction : {-1,0,1}) for(int turn : {-1,0,1}) {
        if(direction==0 && turn==0) continue;
        const double dyaw=turn*pi/8;
        Pose2D end{stuck.x+direction*.45*std::cos(stuck.yaw+dyaw*.5),
                   stuck.y+direction*.45*std::sin(stuck.yaw+dyaw*.5),stuck.yaw+dyaw};
        // Standalone 0.15 m lattice; the live grid_map has its own rolling origin.
        if(direction) {
            end.x=stuck.x+std::round((end.x-stuck.x)/.15)*.15;
            end.y=stuck.y+std::round((end.y-stuck.y)/.15)*.15;
        }
        old_successors += legacyEdge(stuck,direction,turn,rock);
        const bool valid=latticeArcFootprintValid(stuck,end,direction*.45,dyaw,
                                                  radius,.15,.5,true,rock,cost);
        new_successors += valid;
        if(valid) {
            check(rock(end,.5,false,cost),"accepted departure endpoint holds full 0.50 m band");
            // All following edges start with the full band, never a renewed exception.
            check(!latticeArcFootprintValid(end,stuck,-direction*.45,-dyaw,
                                            radius,.15,.5,false,rock,cost),
                  "ordinary successor cannot return into the root clearance deficit");
        }
        full_band_successors += latticeArcFootprintValid(stuck,end,direction*.45,dyaw,
                                                          radius,.15,.5,false,rock,cost);
    }
    std::printf("fixture successors: old=%d departure=%d ordinary=%d\n",
                old_successors,new_successors,full_band_successors);
    check(old_successors==0 && new_successors>0,
          "reproduce zero-successor deadlock and recover collision-free first edges");
    check(full_band_successors==0,"departure exception is not applied to ordinary edges");
    check(departureClearance(stuck,.5,true,rock)==0.0 &&
          departureClearance({-4,-2,0},.5,true,rock)==.5 &&
          departureClearance(stuck,.5,false,rock)==.5,
          "starts that already have full clearance do not receive a reduced band");

    const RectangleMap wall{{{0,0,.01}}};
    check(!sweptFootprintValid({-2,0,0},{2,0,0},radius,.15,0,.5,true,wall,cost),
          "departure ramp cannot cross a known hazard");
    check(!sweptFootprintValid({0,0,0},{-2,0,0},radius,.15,0,.5,true,wall,cost),
          "known hazard under occupied start is never excused");
    const RectangleMap corner{{{1.02,0,.015}}};
    check(corner({0,0,0},0,false,cost) && corner({0,0,pi/2},0,false,cost),
          "rotation fixture endpoints are collision free");
    check(!sweptFootprintValid({0,0,0},{0,0,pi/2},radius,.15,0,0,false,corner,cost),
          "in-place swept corner collision is rejected");
    const RectangleMap narrow{{{0,1.28,.01}}};
    const Pose2D a{0,0,0}, b{.45,0,0}, shifted{.45,.15,0};
    check(sweptFootprintValid(a,b,radius,.15,.5,.5,false,narrow,cost),
          "unquantised edge fixture is valid");
    check(!latticeArcFootprintValid(a,shifted,.45,0,radius,.15,.5,false,narrow,cost),
          "unsafe quantised exported endpoint is rejected");
    const RectangleMap between{{{-1,0,.01}}};
    check(!sweptFootprintValid({0,0,0},{-2,0,0},radius,.15,0,.5,true,between,cost),
          "back-out must check connectors, not only historical endpoints");

    const std::vector<Pose2D> samples{{1.35,0,0},{1.5,0,0},{1.65,0,0}};
    const auto pose_at=[&](std::size_t i) { return samples[i]; };
    check(sampledFootprintValid({1.2,0,0},samples.size(),pose_at,radius,.15,.5,true,wall,cost),
          "sampled dynamics departure restores clearance across one whole primitive");
    check(!sampledFootprintValid({1.2,0,0},samples.size(),pose_at,radius,.15,.5,false,wall,cost),
          "ordinary dynamics primitive does not receive a new departure exception");
    const auto across=[](std::size_t) { return Pose2D{-2,0,0}; };
    check(!sampledFootprintValid({0,0,0},1,across,radius,.15,0,false,between,cost),
          "dynamics start-to-first-sample gap cannot jump across a hazard");
    check(!sampledFootprintValid({0,0,0},0,across,radius,.15,0,false,between,cost),
          "empty dynamics primitive fails closed");

    int unknown_calls=0,total_calls=0;
    auto unknown_at_start=[&](const Pose2D& p,double,bool allow_unknown,float& c) {
        c=2; ++total_calls; if(allow_unknown) ++unknown_calls;
        return p.x>0 || allow_unknown;
    };
    check(sweptFootprintValid({0,0,0},{.45,0,0},radius,.15,0,.5,true,unknown_at_start,cost) &&
          unknown_calls==1 && total_calls>1 && cost==2,
          "unknown exception applies only to the start sample and body cost is preserved");
    auto all_unknown=[](const Pose2D&,double,bool allow_unknown,float& c) {
        c=0; return allow_unknown;
    };
    check(!sweptFootprintValid({0,0,0},{.45,0,0},radius,.15,0,.5,true,all_unknown,cost),
          "unknown ground after the start remains forbidden");
    const auto free=[](const Pose2D&,double,bool,float& c) { c=0; return true; };
    const double nan=std::numeric_limits<double>::quiet_NaN();
    check(!sweptFootprintValid({nan,0,0},b,radius,.15,0,.5,true,free,cost) &&
          !sweptFootprintValid(a,b,radius,0,0,.5,true,free,cost) &&
          !sweptFootprintValid(a,b,radius,.15,-.1,.5,true,free,cost) &&
          !latticeArcFootprintValid(a,b,nan,0,radius,.15,.5,true,free,cost),
          "nonfinite or invalid geometry fails closed");
    const auto bad_cost=[nan](const Pose2D&,double,bool,float& c) { c=float(nan); return true; };
    check(!sweptFootprintValid(a,b,radius,.15,0,.5,true,bad_cost,cost),
          "nonfinite terrain cost fails closed");
    std::printf("planner_safety_selfcheck: %d checks, %d failures\n",checks,failures);
    return failures ? 1 : 0;
}
