// Synthetic geometry tests of the production swept checker. These are not a replay of
// the rolling perception map; logged poses only anchor the synthetic fixtures.
#include "groundgrid/SweptFootprint.h"
#include "groundgrid/FootprintRaster.h"

#include <cstdio>
#include <limits>
#include <set>
#include <utility>
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
using Cell=std::pair<int,int>;
using Cells=std::set<Cell>;
Cell cellAt(double x,double y,double ox=0,double oy=0) {
    return {int(std::floor((x-ox)/.15)),int(std::floor((y-oy)/.15))};
}
Cells rasterCells(const Pose2D& p,double margin,double ox=0,double oy=0) {
    Cells cells;
    visitFootprintCells(p,1.8,1.5,margin,.15,ox,oy,[&](double x,double y) {
        cells.insert(cellAt(x,y,ox,oy)); return true;
    });
    return cells;
}
// Exact pre-fix point-lattice policy: retain only in this regression test.
Cells legacyPointCells(const Pose2D& p,double margin) {
    Cells cells;
    for(double u=-.9-margin;u<=.9+margin+1e-6;u+=.15)
        for(double v=-.75-margin;v<=.75+margin+1e-6;v+=.15)
            cells.insert(cellAt(p.x+std::cos(p.yaw)*u-std::sin(p.yaw)*v,
                                p.y+std::sin(p.yaw)*u+std::cos(p.yaw)*v));
    return cells;
}

// Independent oracle: clip the oriented rectangle polygon against a grid square.
// Production uses separating axes instead, so a repeated implementation bug is less
// likely to pass both sides of the comparison. Test only, not on the runtime path.
bool clippedOverlap(const Pose2D& p,double margin,double cx,double cy) {
    using Point=std::pair<double,double>;
    std::vector<Point> poly;
    for(const auto& corner : std::vector<Point>{{-.9-margin,-.75-margin},
             {.9+margin,-.75-margin},{.9+margin,.75+margin},{-.9-margin,.75+margin}})
        poly.emplace_back(p.x+std::cos(p.yaw)*corner.first-std::sin(p.yaw)*corner.second,
                          p.y+std::sin(p.yaw)*corner.first+std::cos(p.yaw)*corner.second);
    for(int axis=0;axis<2;++axis) for(int sign : {-1,1}) {
        const double centre=axis==0 ? cx : cy;
        const double edge=centre+sign*.075;
        const auto distance=[&](const Point& a) {
            return sign*((axis==0 ? a.first : a.second)-edge);
        };
        std::vector<Point> clipped;
        if(poly.empty()) return false;
        Point a=poly.back();
        double da=distance(a);
        for(const auto& b : poly) {
            const double db=distance(b);
            const bool ai=da<=1e-12, bi=db<=1e-12;
            if(ai!=bi) {
                const double t=da/(da-db);
                clipped.emplace_back(a.first+t*(b.first-a.first),a.second+t*(b.second-a.second));
            }
            if(bi) clipped.push_back(b);
            a=b; da=db;
        }
        poly.swap(clipped);
    }
    return !poly.empty();
}
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
    {
        const Pose2D p{6.425,-6.525,-pi/4};
        const Cell lethal=cellAt(5.175,-6.075);
        const auto body=legacyPointCells(p,0), ordinary=legacyPointCells(p,.25);
        const auto full=legacyPointCells(p,.5);
        check(!body.count(lethal) && !full.count(lethal) && ordinary.count(lethal),
              "reproduce same-map 0.50 m pass but 0.25 m rejection from rotated point holes");
        const auto new_body=rasterCells(p,0), new_ordinary=rasterCells(p,.25), new_full=rasterCells(p,.5);
        check(!new_body.count(lethal) && new_ordinary.count(lethal) && new_full.count(lethal),
              "both rasterised clearance bands reject the known hazard outside the physical body");
        check(std::includes(new_full.begin(),new_full.end(),new_ordinary.begin(),new_ordinary.end()) &&
              std::includes(new_ordinary.begin(),new_ordinary.end(),new_body.begin(),new_body.end()),
              "grid coverage is nested between body, ordinary and snap clearance");
        const auto axis=rasterCells({0,0,0},0);
        check(axis.count(cellAt(.975,.075)) && axis.count(cellAt(-.975,-.075)),
              "raster includes cells touching both positive and negative rectangle edges");
        check(!axis.count(cellAt(1.125,.075)),
              "rectangle raster does not substitute the rover circumscribed circle");
        const auto shifted=rasterCells({p.x+1.05,p.y-.6,p.yaw},.5,1.05,-.6);
        check(shifted==new_full,"rolling grid origin and rover translation preserve cell coverage");
        bool monotone=true, oracle=true;
        std::size_t oracle_cells=0;
        for(int heading=-32;heading<=32;++heading) for(double offset : {0.0,.037,.149}) {
            const Pose2D pose{-.32+offset,.27-offset,heading*.1};
            const double ox=.031,oy=-.021;
            Cells previous;
            for(double margin : {0.0,.25,.5}) {
                const auto got=rasterCells(pose,margin,ox,oy);
                monotone &= std::includes(got.begin(),got.end(),previous.begin(),previous.end());
                Cells expected;
                for(int i=-22;i<=22;++i) for(int j=-22;j<=22;++j) {
                    const double cx=ox+(i+.5)*.15,cy=oy+(j+.5)*.15;
                    if(clippedOverlap(pose,margin,cx,cy)) expected.insert({i,j});
                    ++oracle_cells;
                }
                oracle &= got==expected;
                previous=got;
            }
        }
        check(monotone,"clearance nesting across 195 headings/subcell-offset fixtures");
        check(oracle,"raster agrees with independent polygon clipping for every fixture cell");
        std::printf("raster oracle: %zu rectangle/cell comparisons\n",oracle_cells);
        int calls=0;
        check(!visitFootprintCells(p,1.8,1.5,.5,.15,0,0,[&](double,double) {
                  ++calls; return false;
              }) && calls==1,"first rejected cell stops raster iteration immediately");
        calls=0;
        const auto count=[&](double,double) { ++calls; return true; };
        const double nan=std::numeric_limits<double>::quiet_NaN();
        check(!visitFootprintCells({nan,0,0},1.8,1.5,0,.15,0,0,count) &&
              !visitFootprintCells(p,1.8,1.5,-.1,.15,0,0,count) &&
              !visitFootprintCells(p,0,1.5,0,.15,0,0,count) &&
              !visitFootprintCells(p,1.8,1.5,0,0,0,0,count) &&
              !visitFootprintCells(p,1.8,1.5,0,.15,nan,0,count) &&
              !visitFootprintCells({1e30,0,0},1.8,1.5,0,.15,0,0,count) &&
              !visitFootprintCells(p,1e6,1e6,0,.15,0,0,count) && calls==0,
              "invalid or oversized raster geometry fails closed without cell callbacks");
        const auto grid_hazard=[&](const Pose2D& pose,double margin,bool,float& terrain) {
            terrain=0;
            return visitFootprintCells(pose,1.8,1.5,margin,.15,0,0,
                [&](double x,double y) { return cellAt(x,y)!=lethal; });
        };
        check(!sweptFootprintValid(p,p,radius,.15,.5,.5,false,grid_hazard,cost) &&
              !sweptFootprintValid(p,p,radius,.15,.25,.25,false,grid_hazard,cost),
              "search and retained swept checks reject the same rasterised hazard");
    }
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
