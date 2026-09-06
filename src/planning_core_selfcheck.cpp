#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "groundgrid/LatticePlannerCore.h"
#include "groundgrid/PlanningReplayAudit.h"
#include "groundgrid/TrajectoryTracking.h"

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
    check(auditPlanningOutput(input,result),"independent exported-output audit");
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
        auto unsafe=forward; unsafe.path.poses[3].y=std::numeric_limits<double>::quiet_NaN();
        check(!auditPlanningOutput(input,unsafe),"audit rejects non-finite exported pose");
        auto hazard=flatInput();
        PlanningIndex blocked; hazard.map.getIndex({forward.path.poses[3].x,forward.path.poses[3].y},blocked);
        hazard.map.cost[blocked.a*hazard.map.cols+blocked.b]=100.0f;
        check(!auditPlanningOutput(hazard,forward),"audit rejects hazardous exported sweep");
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
        dynamics.goal={-4.125,-1.575,0};
        const auto dynamic_reverse=LatticePlannerCore(dynamics).planCore(dynamics.start,dynamics.goal,10000);
        verifyPath(dynamics,dynamic_reverse);
        check(dynamic_reverse.reverse_length>0.5,"dynamics reverse remains supported");
        dynamics.goal={dynamics.start.x,dynamics.start.y,kPlannerPi/4.0};
        const auto dynamic_rotate=LatticePlannerCore(dynamics).planCore(dynamics.start,dynamics.goal,10000);
        verifyPath(dynamics,dynamic_rotate);
        check(dynamic_rotate.path_length<1e-6,"dynamics in-place rotation remains supported");
        auto joined=dynamics;
        std::vector<MotionPrimitive> straight_and_rotate;
        for(int bin=0;bin<16;++bin) {
            straight_and_rotate.push_back(joined.primitives.primitivesFor(bin).at(0));
            straight_and_rotate.push_back(joined.primitives.primitivesFor(bin).at(14));
        }
        check(joined.primitives.restore(16,straight_and_rotate),"restricted dynamics fixture");
        joined.goal={-1.875,-1.575,kPlannerPi/4};
        const auto joined_result=LatticePlannerCore(joined).planCore(joined.start,joined.goal,1000);
        verifyPath(joined,joined_result);
        bool rotation_after_translation=false, seen_translation=false;
        std::vector<TrackingSample> track;
        for(std::size_t i=0;i<joined_result.path.poses.size();++i) {
            const auto& pose=joined_result.path.poses[i];
            const double v=joined_result.profile.data[2*i],w=joined_result.profile.data[2*i+1];
            track.push_back({pose,v,w});
            seen_translation |= std::abs(v)>1e-6;
            if(i && std::abs(v)<1e-6 && std::abs(w)>1e-6) {
                const auto& previous=joined_result.path.poses[i-1];
                check(std::hypot(pose.x-previous.x,pose.y-previous.y)<1e-10,
                      "zero-ICR rotation has no phantom grid-centre translation");
                rotation_after_translation |= seen_translation;
            }
        }
        check(rotation_after_translation,"fixture actually exercises a translation/rotation seam");
        const auto& actual_end=joined_result.path.poses.back();
        check(std::hypot(actual_end.x-joined_result.selected_goal.x,
                         actual_end.y-joined_result.selected_goal.y)<1e-10 &&
              std::abs(SkidSteerModel::wrap(actual_end.yaw-joined_result.selected_goal.yaw))<1e-10,
              "dynamics selected endpoint is the actual exported pose");
        TrajectoryTracking tracker;TrackingParams tracking;bool changed;
        check(tracker.setTrajectory(track,changed),"continuous dynamics trajectory accepted");
        Pose2D rover=joined.start;bool complete=false;
        for(int tick=0;tick<500;++tick) {
            if(tick%10==0) check(tracker.setTrajectory(track,changed) && !changed,
                                 "continuous route republish retains phase progress");
            const auto step=tracker.step(rover,tracking);
            check(step.status!=TrackingStatus::Invalid,"continuous seam has a usable phase command");
            if(step.status==TrackingStatus::GoalReached) { complete=true;break; }
            rover=SkidSteerModel{}.integrate(rover,step.desired_v,step.desired_w,0.05);
        }
        check(complete,"production follower completes continuous dynamics seam within 25s");
        auto prefix=dynamics;prefix.goal={-2.625,-1.575,0};
        const auto prefix_result=LatticePlannerCore(prefix).planCore(prefix.start,prefix.goal,1000);
        verifyPath(prefix,prefix_result);
        check(prefix_result.path.poses.size()>2 && prefix_result.path.poses.size()<19 &&
              !prefix_result.budget_exhausted,"exact goal on a primitive uses a checked nonzero prefix");
        // Reproduce the real dynamics failure: a slope cell is outside the body but
        // inside its first exported sample's clearance band. Ramping across all 18
        // integration samples used to approve a rotation that execution immediately
        // withdrew. The safe response is no successor/recovery, not a weaker gate.
        auto departure=dynamics;
        departure.start={0.075,0.075,kPlannerPi/4};
        departure.goal={0.075,0.075,kPlannerPi/2};
        PlanningIndex slope_cell;
        check(departure.map.getIndex({1.575,0.075},slope_cell),"departure slope index");
        const auto cell=slope_cell.a*departure.map.cols+slope_cell.b;
        departure.map.cost[cell]=61.4f;
        departure.map.gx[cell]=-0.2778f; departure.map.gy[cell]=0.1187f;
        departure.map.slope[cell]=16.811f;
        LatticePlannerCore departure_core(departure);
        const auto& turn_primitive=departure.primitives.primitivesFor(2).at(14);
        auto world_sample=[&](std::size_t i) {
            const auto& s=turn_primitive.samples[i];
            return Pose2D{departure.start.x+s.x,departure.start.y+s.y,
                          departure.start.yaw+s.yaw}; // in-place: x=y=0
        };
        float departure_cost; double ex,ey,eyaw;
        check(sampledFootprintValid(departure.start,turn_primitive.samples.size(),world_sample,
            departure_core.cornerRadius(),departure.map.resolution,
            departure.config.goal_snap_clearance_,true,
            [&](const Pose2D& p,double margin,bool unknown,float& terrain) {
                return departure_core.footprintWithClearanceValid(p.x,p.y,p.yaw,terrain,unknown,margin);
            },departure_cost),"old whole-primitive departure passed");
        check(!departure_core.executionDepartureValid(departure.start,
              turn_primitive.samples.size(),world_sample),"execution rejects first sample full margin");
        PlanningResult formerly_accepted;
        formerly_accepted.ok=true; formerly_accepted.snapped=true;
        formerly_accepted.path.poses.push_back(departure.start);
        for(std::size_t i=0;i<turn_primitive.samples.size();++i)
            formerly_accepted.path.poses.push_back(world_sample(i));
        formerly_accepted.departure_end_index=turn_primitive.samples.size();
        formerly_accepted.profile.data.assign(formerly_accepted.path.poses.size()*2,0.0f);
        check(!auditPlanningOutput(departure,formerly_accepted),
              "offline audit must catch the recorded planning/execution disagreement");
        check(!departure_core.primitiveValid(departure.start.x,departure.start.y,
              departure.start.yaw,turn_primitive,departure_cost,ex,ey,eyaw,true),
              "search must reject the same unexecutable departure");
        auto rejected_departure=departure_core.planCore(departure.start,departure.goal,100);
        check(!rejected_departure.ok && rejected_departure.reason=="start_no_successor" &&
              rejected_departure.expanded==1,"unexecutable root triggers recovery without false success");
        const auto nonroot=departure_core.primitiveValid(departure.start.x,departure.start.y,
              departure.start.yaw,turn_primitive,departure_cost,ex,ey,eyaw,false);
        check(!nonroot,"later primitives cannot restart the departure exception");
        auto clear_departure=departure; clear_departure.map.gx[cell]=clear_departure.map.gy[cell]=0;
        auto clear_result=LatticePlannerCore(clear_departure).planCore(
            clear_departure.start,clear_departure.goal,1000);
        verifyPath(clear_departure,clear_result);
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
        auto rolled_wall=wall;
        rolled_wall.map.start_row=57; rolled_wall.map.start_col=41;
        for(int r=0;r<80;++r) for(int c=0;c<80;++c) {
            PlanningPosition p; PlanningIndex original;
            rolled_wall.map.getPosition({r,c},p);
            check(wall.map.getIndex(p,original),"rolled non-uniform cache coordinate");
            rolled_wall.map.cost[r*80+c]=wall.map.cost[original.a*80+original.b];
        }
        const auto rolled_reachable=LatticePlannerCore(rolled_wall).planCore(wall.start,wall.goal,300);
        verifyPath(rolled_wall,rolled_reachable);
        check(rolled_reachable.selected_goal.x==reachable.selected_goal.x &&
              rolled_reachable.profile.data==reachable.profile.data,
              "non-uniform rolling cache retains world hazards and selected route");
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
        PlanningIndex inside_cell,outside_cell;
        check(wall.map.getIndex({-2.900,0.075},inside_cell) &&
              wall.map.getIndex({-2.950,0.075},outside_cell) && inside_cell.a==outside_cell.a &&
              inside_cell.b==outside_cell.b,"different integrated endpoints share one grid key");
        check(probe.snapPoseCandidate({-2.900,0.075,0},request,penalty,distance) &&
              !probe.snapPoseCandidate({-2.950,0.075,0},request,penalty,distance),
              "continuous endpoint range uses exact pose, never a cached centre certificate");
        check(!probe.snapPoseCandidate({-2.900,0.075,0.80},request,penalty,distance),
              "continuous endpoint heading cannot exceed the existing snap span");
        auto tiny=wall; tiny.config.max_snap_distance_=0.15;
        const auto no_candidate=LatticePlannerCore(tiny).planCore(tiny.start,tiny.goal,100);
        check(!no_candidate.ok && !no_candidate.selected_goal_valid,"no valid endpoint is not a fake snap");
        check(no_candidate.reason=="goal_invalid" && no_candidate.expanded==0 &&
              !no_candidate.budget_exhausted,"empty safe goal set fails before allocating/searching states");
        auto repeated_core=LatticePlannerCore(wall);
        const auto repeated_a=repeated_core.planCore(wall.start,wall.goal,300);
        repeated_core.goal_snap_clearance_=4.0;
        const auto repeated_b=repeated_core.planCore(wall.start,wall.goal,300);
        check(repeated_a.ok && !repeated_b.ok && repeated_b.expanded==0,
              "candidate certificates cannot outlive a plan or clearance change");
        auto one_candidate=wall;
        one_candidate.config.goal_snap_heading_span_=0;
        const auto one_heading=LatticePlannerCore(one_candidate).planCore(wall.start,wall.goal,300);
        verifyPath(one_candidate,one_heading);
        check(one_heading.selected_goal.yaw==0.0,"viability probe preserves exact candidate heading range");
        auto timed=wall; timed.config.max_planning_time_=1e-9;
        const auto time_limit=LatticePlannerCore(timed).planCore(timed.start,timed.goal);
        check(!time_limit.ok && time_limit.budget_exhausted && time_limit.expanded==0,"single shared wall-time budget");
        check(time_limit.reason=="snap_timeout","incomplete viability scan is not proof of an empty goal set");
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
