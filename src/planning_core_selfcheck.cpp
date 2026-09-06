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
                input.config.trajectory_clearance_,input.config.trajectory_clearance_,false,cost),
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
        std::cout<<"planning_core_selfcheck passed; forward poses="<<forward.path.poses.size()
                 <<" expanded="<<forward.expanded<<" reverse_m="<<reverse.reverse_length<<'\n';
        return EXIT_SUCCESS;
    } catch(const std::exception& e) {
        std::cerr<<"planning_core_selfcheck failed: "<<e.what()<<'\n';
        return EXIT_FAILURE;
    }
}
