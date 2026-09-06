#pragma once
#include "groundgrid/LatticePlannerCore.h"

namespace groundgrid {
// Offline verification of the ACTUAL exported polyline, not just ideal primitives.
// Does not replace runtime map revalidation, and does not certify physical braking.
inline bool auditPlanningOutput(const PlanningInput& input,const PlanningResult& result) {
    if(!result.ok) return result.path.poses.empty() && result.profile.data.empty();
    const auto& path=result.path.poses;
    if(path.empty() || result.profile.data.size()!=2*path.size() ||
       result.departure_end_index>=path.size()) return false;
    LatticePlannerCore checker(input);
    const double clearance=result.snapped ? input.config.goal_snap_clearance_
                                          : input.config.trajectory_clearance_;
    float cost;
    if(!checker.sweptSegmentValid(input.start,path.front(),0,0,true,cost)) return false;
    for(std::size_t i=0;i<path.size();++i) {
        const auto& p=path[i];
        const float v=result.profile.data[2*i],w=result.profile.data[2*i+1];
        if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
           !std::isfinite(v) || !std::isfinite(w) ||
           std::abs(v)>input.config.sp_.v_max+1e-6 || std::abs(w)>input.config.sp_.w_max+1e-6)
            return false;
    }
    if(result.departure_end_index) {
        if(input.config.use_dynamics_primitives_ && !input.primitives.empty() &&
           !checker.executionDepartureValid(path.front(),result.departure_end_index,
                [&](std::size_t i){ return path[i+1]; })) return false;
        if(!sampledFootprintValid(path.front(),result.departure_end_index,
            [&](std::size_t i){ return path[i+1]; },checker.cornerRadius(),
            input.map.resolution,clearance,true,
            [&](const Pose2D& p,double margin,bool unknown,float& terrain) {
                return checker.footprintWithClearanceValid(p.x,p.y,p.yaw,terrain,unknown,margin);
            },cost)) return false;
    }
    for(std::size_t i=result.departure_end_index+1;i<path.size();++i)
        if(!checker.sweptSegmentValid(path[i-1],path[i],clearance,clearance,false,cost)) return false;
    return checker.footprintWithClearanceValid(path.back().x,path.back().y,path.back().yaw,
                                              cost,false,clearance);
}
} // namespace groundgrid
