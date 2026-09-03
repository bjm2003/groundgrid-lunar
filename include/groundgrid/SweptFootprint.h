#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "groundgrid/SkidSteerModel.h"

namespace groundgrid {

// Optional observation of the first failed swept sample. It never changes acceptance;
// a failed full-band departure probe is not an edge failure and is not recorded here.
struct SweptFootprintRejection {
    bool has_sample = false;
    Pose2D pose;
    double clearance = 0.0;
    bool allow_unknown = false;
};

template<typename CheckFootprint>
double departureClearance(const Pose2D& start, double full_clearance,
                           bool departure, CheckFootprint check) {
    if(!std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.yaw) ||
       !std::isfinite(full_clearance) || full_clearance<0.0)
        return std::numeric_limits<double>::quiet_NaN();
    float cost=0.0f;
    // Do not grant an exception to a start that already satisfies the whole band.
    return departure && !check(start,full_clearance,true,cost) ? 0.0 : full_clearance;
}

// Shared by search edges, retained paths and recovery. The checker must validate the
// physical rectangle independently of its extra clearance band. Unknown body cells are
// permitted only at the initial pose, never at any subsequent swept sample.
//
// A departure from the *current* start may grow clearance from zero to the normal value
// over ONE edge. This is not permission to cross a hazard: every physical body is checked,
// the edge endpoint must have full clearance, and all later edges keep full clearance.
template<typename CheckFootprint>
bool sweptFootprintValid(const Pose2D& from, const Pose2D& to,
                         double corner_radius, double resolution,
                         double clearance0, double clearance1,
                         bool allow_start_unknown, CheckFootprint check,
                         float& mean_cost, SweptFootprintRejection* rejection = nullptr) {
    mean_cost = 0.0f;
    if(rejection) *rejection = SweptFootprintRejection{};
    if(!std::isfinite(from.x) || !std::isfinite(from.y) || !std::isfinite(from.yaw) ||
       !std::isfinite(to.x) || !std::isfinite(to.y) || !std::isfinite(to.yaw) ||
       !std::isfinite(corner_radius) || corner_radius < 0.0 ||
       !std::isfinite(resolution) || resolution <= 0.0 ||
       !std::isfinite(clearance0) || clearance0 < 0.0 ||
       !std::isfinite(clearance1) || clearance1 < 0.0) return false;
    const double dx = to.x-from.x, dy = to.y-from.y;
    const double dyaw = SkidSteerModel::wrap(to.yaw-from.yaw);
    // Bound travel of inflated corners, including growth of the clearance band. An
    // endpoint-only translation or rotation check can miss an obstacle between samples.
    const double radius = corner_radius + std::sqrt(2.0)*std::max(clearance0, clearance1);
    const double sweep = std::hypot(dx,dy) + std::abs(dyaw)*radius +
                         std::sqrt(2.0)*std::abs(clearance1-clearance0);
    const double steps = std::max(1.0, std::ceil(sweep/resolution));
    if(!std::isfinite(steps) || steps > 1000000.0) return false;
    const int n = static_cast<int>(steps);
    double sum = 0.0;
    for(int i=0; i<=n; ++i) {
        const double q = static_cast<double>(i)/n;
        const Pose2D pose{from.x+q*dx, from.y+q*dy,
                          SkidSteerModel::wrap(from.yaw+q*dyaw)};
        float cost = 0.0f;
        const double margin = clearance0+q*(clearance1-clearance0);
        const bool unknown = allow_start_unknown && i==0;
        if(!check(pose, margin, unknown, cost) || !std::isfinite(cost)) {
            if(rejection) *rejection = {true, pose, margin, unknown};
            return false;
        }
        if(i) sum += cost;
    }
    mean_cost = static_cast<float>(sum/n);
    return true;
}

// Validate both the ideal arc used to generate a lattice successor and the straight
// interpolation of its quantised endpoint actually sent to the follower. Keeping this
// in the production core lets the no-successor regression run without ROS/grid_map.
template<typename CheckFootprint>
bool latticeArcFootprintValid(const Pose2D& from, const Pose2D& lattice_end,
                              double signed_length, double heading_delta,
                              double corner_radius, double resolution,
                              double clearance, bool departure,
                              CheckFootprint check, float& mean_cost,
                              SweptFootprintRejection* rejection = nullptr) {
    mean_cost = 0.0f;
    if(rejection) *rejection = SweptFootprintRejection{};
    if(!std::isfinite(signed_length) || !std::isfinite(heading_delta) ||
       !std::isfinite(resolution) || resolution <= 0.0) return false;
    const double steps = std::max({1.0, std::ceil(std::abs(signed_length)/resolution),
                                   std::ceil(std::abs(heading_delta)/0.2)});
    if(!std::isfinite(steps) || steps > 1000000.0) return false;
    const int n = static_cast<int>(steps);
    const double initial_clearance=departureClearance(from,clearance,departure,check);
    Pose2D previous = from;
    double sum = 0.0;
    for(int i=1; i<=n; ++i) {
        const double q = double(i)/n;
        const Pose2D current{
            from.x+signed_length*q*std::cos(from.yaw+q*heading_delta*0.5),
            from.y+signed_length*q*std::sin(from.yaw+q*heading_delta*0.5),
            from.yaw+q*heading_delta};
        const double margin0 = initial_clearance+(clearance-initial_clearance)*double(i-1)/n;
        const double margin1 = initial_clearance+(clearance-initial_clearance)*q;
        float cost;
        if(!sweptFootprintValid(previous,current,corner_radius,resolution,
                                margin0,margin1,departure && i==1,check,cost,rejection)) return false;
        sum += cost;
        previous = current;
    }
    float exported_cost;
    if(!sweptFootprintValid(from,lattice_end,corner_radius,resolution,
                            initial_clearance,clearance,departure,
                            check,exported_cost,rejection)) return false;
    mean_cost = static_cast<float>(sum/n);
    return true;
}

// Uniformly timed primitive samples, transformed into world coordinates by pose_at.
// Include the start-to-first-sample gap and never restart the ramp at each sample.
template<typename PoseAt, typename CheckFootprint>
bool sampledFootprintValid(const Pose2D& start, std::size_t count, PoseAt pose_at,
                           double corner_radius, double resolution, double clearance,
                           bool departure, CheckFootprint check, float& mean_cost,
                           SweptFootprintRejection* rejection = nullptr) {
    mean_cost = 0.0f;
    if(rejection) *rejection = SweptFootprintRejection{};
    if(count==0 || count>1000000) return false;
    const double initial_clearance=departureClearance(start,clearance,departure,check);
    Pose2D previous=start;
    double sum=0.0;
    for(std::size_t i=0; i<count; ++i) {
        const Pose2D current=pose_at(i);
        const double margin0=initial_clearance+(clearance-initial_clearance)*double(i)/count;
        const double margin1=initial_clearance+(clearance-initial_clearance)*double(i+1)/count;
        float cost;
        if(!sweptFootprintValid(previous,current,corner_radius,resolution,
                                margin0,margin1,departure && i==0,check,cost,rejection)) return false;
        previous=current;
        sum+=cost;
    }
    mean_cost=static_cast<float>(sum/count);
    return true;
}

} // namespace groundgrid
