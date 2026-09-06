#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "groundgrid/LatticePlannerCore.h"

using namespace groundgrid;
static void check(bool ok,const char* message) {
    if(!ok) throw std::runtime_error(message);
}
static PlanningInput flatInput() {
    PlanningInput input;
    auto& m=input.map;
    m.rows=80; m.cols=80; m.resolution=0.15; m.length_x=12.0; m.length_y=12.0;
    m.cost.assign(6400,0.0f); m.gx=m.gy=m.slope=m.cost;
    input.start={-3.075,-1.575,0}; input.goal={0.075,-1.575,0};
    return input;
}
static void verifyPath(const PlanningInput& input,const PlanningResult& result) {
    check(result.ok,"expected a route");
    check(result.path.poses.size()*2==result.profile.data.size(),"profile length");
    LatticePlannerCore core(input);
    const double clearance=result.snapped ? input.config.goal_snap_clearance_ : input.config.trajectory_clearance_;
    for(std::size_t i=0;i<result.path.poses.size();++i) {
        const auto& p=result.path.poses[i];
        check(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw),"finite pose");
        check(std::isfinite(result.profile.data[2*i]) &&
              std::abs(result.profile.data[2*i])<=input.config.sp_.v_max+1e-6,"v bound");
        check(std::isfinite(result.profile.data[2*i+1]) &&
              std::abs(result.profile.data[2*i+1])<=input.config.sp_.w_max+1e-6,"w bound");
        if(i) {
            float cost;
            check(core.sweptSegmentValid(result.path.poses[i-1],p,
                clearance,clearance,false,cost),
                "exported segment safety");
        }
    }
}
int main() {
    try {
        auto input=flatInput();
        check(input.map.valid(),"valid map");
        for(int sr:{0,1,37,79}) for(int sc:{0,7,79}) {
            input.map.start_row=sr; input.map.start_col=sc;
            for(int r=0;r<80;++r) for(int c=0;c<80;++c) {
                PlanningPosition p; PlanningIndex i;
                check(input.map.getPosition({r,c},p) && input.map.getIndex(p,i) &&
                      i.a==r && i.b==c,"circular index round trip");
            }
        }
        input=flatInput();
        LatticePlannerCore core(input);
        const auto forward=core.planCore(input.start,input.goal,50000);
        verifyPath(input,forward);
        const auto again=core.planCore(input.start,input.goal,50000);
        check(forward.profile.data==again.profile.data && forward.expanded==again.expanded,
              "deterministic repeated query");
        input.goal={-4.425,-1.575,0};
        const auto reverse=core.planCore(input.start,input.goal,50000);
        verifyPath(input,reverse);
        check(reverse.reverse_length>1.0,"reverse route");
        input.goal={input.start.x,input.start.y,kPlannerPi/4.0};
        const auto rotate=core.planCore(input.start,input.goal,50000);
        verifyPath(input,rotate);
        check(rotate.path_length<1e-6 && rotate.path.poses.size()>2,"in-place profile");
        auto limited=core.planCore(input.start,{3.075,3.075,0},1);
        check(!limited.ok && limited.expanded==1,"fixed expansion budget");
        const auto invalid=core.planCore({std::numeric_limits<double>::quiet_NaN(),0,0},input.goal);
        check(!invalid.ok && invalid.reason=="invalid_input" && invalid.path.poses.empty(),"NaN input");
        auto unknown=flatInput();
        unknown.map.cost.assign(6400,std::numeric_limits<float>::quiet_NaN());
        LatticePlannerCore blind(unknown);
        check(!blind.planCore(unknown.start,unknown.goal,1000).ok,"unknown not a free route");
        auto malformed=flatInput(); malformed.map.cost.pop_back();
        check(!LatticePlannerCore(malformed).planCore(malformed.start,malformed.goal).ok,"array mismatch");
        auto dynamics=flatInput();
        dynamics.config.use_dynamics_primitives_=true;
        dynamics.primitives.generate(SkidSteerModel(dynamics.config.sp_),PrimitiveGenConfig{});
        dynamics.goal={-1.875,-1.575,0};
        const auto dynamic_result=LatticePlannerCore(dynamics).planCore(dynamics.start,dynamics.goal,10000);
        verifyPath(dynamics,dynamic_result);
        check(dynamic_result.path.poses.size()>2,"dynamics samples preserved");
        auto shifted=flatInput(); shifted.map.start_row=37;shifted.map.start_col=13;
        const auto shifted_result=LatticePlannerCore(shifted).planCore(shifted.start,shifted.goal,50000);
        verifyPath(shifted,shifted_result);
        check(shifted_result.profile.data==forward.profile.data,"buffer shift leaves straight route unchanged");
        auto wall=flatInput();
        wall.config.max_snap_distance_=3.0;
        wall.start={-3.075,0.075,0}; wall.goal={0.075,0.075,0};
        for(int r=0;r<wall.map.rows;++r) for(int c=0;c<wall.map.cols;++c) {
            PlanningPosition p; wall.map.getPosition({r,c},p);
            if(std::abs(p.x()+0.525)<0.01) wall.map.cost[r*wall.map.cols+c]=100.0f;
        }
        const auto legacy=LatticePlannerCore(wall).planCore(wall.start,wall.goal,300);
        check(!legacy.ok && legacy.snapped && legacy.selected_goal.x>0.0,"nearest snap across impassable wall");
        wall.config.reachable_snap_=true;
        const auto reachable=LatticePlannerCore(wall).planCore(wall.start,wall.goal,300);
        verifyPath(wall,reachable);
        check(reachable.snapped && reachable.selected_goal.x< -1.9 &&
              reachable.snap_distance<=wall.config.max_snap_distance_,"reachable near-side candidate");
        check(reachable.expanded<legacy.expanded,"shared frontier does not exhaust blocked endpoint budget");
        auto normal=flatInput(); normal.config.reachable_snap_=true;
        const auto normal_result=LatticePlannerCore(normal).planCore(normal.start,normal.goal,50000);
        check(normal_result.profile.data==forward.profile.data && normal_result.expanded==forward.expanded,
              "valid requested goal retains legacy search");
        auto backwards=wall; backwards.start.yaw=kPlannerPi; backwards.goal.yaw=kPlannerPi;
        const auto backed=LatticePlannerCore(backwards).planCore(backwards.start,backwards.goal,300);
        verifyPath(backwards,backed);
        check(backed.reverse_length>0.4,"reachable goal set supports reversing");
        double penalty=0.0,distance=0.0;
        LatticePlannerCore probe(wall); LatticePlannerCore::State request,near_side;
        check(probe.poseToState(wall.goal,request) && probe.poseToState(reachable.selected_goal,near_side),"candidate indices");
        near_side.t=(request.t+8)%16;
        check(!probe.snapCandidate(near_side,request,penalty,distance),"candidate heading range is not relaxed");
        auto tiny=wall; tiny.config.max_snap_distance_=0.15;
        const auto no_candidate=LatticePlannerCore(tiny).planCore(tiny.start,tiny.goal,100);
        check(!no_candidate.ok && !no_candidate.selected_goal_valid,"no valid endpoint is not a fake snap");
        auto timed=wall; timed.config.max_planning_time_=1e-9;
        const auto time_limit=LatticePlannerCore(timed).planCore(timed.start,timed.goal);
        check(!time_limit.ok && time_limit.budget_exhausted && time_limit.expanded==0,"single shared wall-time budget");
        float endpoint_cost;
        check(LatticePlannerCore(wall).footprintWithClearanceValid(reachable.selected_goal.x,
              reachable.selected_goal.y,reachable.selected_goal.yaw,endpoint_cost,false,
              wall.config.goal_snap_clearance_),"selected endpoint full clearance");
        std::cout<<"wall fixture: legacy expanded="<<legacy.expanded<<" ok="<<legacy.ok
                 <<" reachable expanded="<<reachable.expanded<<" ok="<<reachable.ok
                 <<" endpoint_x="<<reachable.selected_goal.x<<'\n';
        std::cout<<"planning_core_selfcheck passed; forward poses="<<forward.path.poses.size()
                 <<" expanded="<<forward.expanded<<" reverse_m="<<reverse.reverse_length<<'\n';
        return EXIT_SUCCESS;
    } catch(const std::exception& e) {
        std::cerr<<"planning_core_selfcheck failed: "<<e.what()<<'\n';
        return EXIT_FAILURE;
    }
}
